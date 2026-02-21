# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0
#
# NeuroCore Field Sensor — cocotb testbench
# Tests each stage of the digital signal-processing pipeline:
#   Reset → Idle → 2-FF Sync → LMS Filter → Haar DWT → Abs Magnitude →
#   Power Accumulator → Command Encoder → LSK Modulator
#
# Pipeline flow (per wake event):
#   S_IDLE → S_WAKE → S_LMS → S_WAIT_LMS → S_DWT → S_WAIT_DWT →
#   S_ABS → S_WAIT_ABS → S_ACCUM → S_WAIT_ACCUM → S_ENCODE →
#   S_LSK_TX → S_SLEEP → S_IDLE
#
# Pin map (project.v):
#   ui_in[3:0]  = adc_data        uo_out[2:0] = cmd_out
#   ui_in[4]    = adc_valid        uo_out[3]   = cmd_valid
#   ui_in[5]    = wake             uo_out[4]   = lsk_ctrl
#                                  uo_out[5]   = lsk_tx
#                                  uo_out[6]   = pwr_gate_ctrl
#                                  uo_out[7]   = processing
#   uio_out[0]  = lms_busy
#   uio_out[1]  = dwt_busy
#   uio_out[2]  = cordic_busy (hard-wired 0)

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles


# ============================================================================
# Pin-map helpers
# ============================================================================

def build_ui(adc_data=0, adc_valid=0, wake=0):
    """Build the 8-bit ui_in value from individual fields."""
    return (adc_data & 0xF) | ((adc_valid & 1) << 4) | ((wake & 1) << 5)


def safe_int(sig, default=0):
    """Read a signal as int, returning *default* if it contains X / Z."""
    try:
        return int(sig.value)
    except ValueError:
        return default


# --- uo_out fields ---
def cmd_out(dut):       return safe_int(dut.uo_out) & 0x07
def cmd_valid(dut):     return (safe_int(dut.uo_out) >> 3) & 1
def lsk_ctrl(dut):      return (safe_int(dut.uo_out) >> 4) & 1
def lsk_tx(dut):        return (safe_int(dut.uo_out) >> 5) & 1
def pwr_gate(dut):      return (safe_int(dut.uo_out) >> 6) & 1
def processing(dut):    return (safe_int(dut.uo_out) >> 7) & 1

# --- uio_out fields ---
def lms_busy(dut):      return safe_int(dut.uio_out) & 1
def dwt_busy(dut):      return (safe_int(dut.uio_out) >> 1) & 1
def cordic_busy(dut):   return (safe_int(dut.uio_out) >> 2) & 1


# ============================================================================
# Reusable coroutines
# ============================================================================

async def init(dut, period_ns=20):
    """Start a 50 MHz clock and apply a clean reset."""
    clock = Clock(dut.clk, period_ns, unit="ns")
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)


async def feed_sample(dut, data):
    """Pulse one 4-bit ADC sample.  Accounts for the 2-FF synchronizer."""
    dut.ui_in.value = build_ui(adc_data=(data & 0xF), adc_valid=1)
    await ClockCycles(dut.clk, 1)
    dut.ui_in.value = build_ui(adc_data=(data & 0xF), adc_valid=0)
    await ClockCycles(dut.clk, 3)          # let sync propagate + LMS shift


async def pulse_wake(dut):
    """Assert wake for 1 cycle, then wait for 2-FF sync propagation."""
    dut.ui_in.value = build_ui(wake=1)
    await ClockCycles(dut.clk, 1)
    dut.ui_in.value = 0
    await ClockCycles(dut.clk, 3)


async def wait_for(dut, fn, value, timeout=60000):
    """Poll *fn(dut)* until it equals *value*.  Returns True on match."""
    for _ in range(timeout):
        if fn(dut) == value:
            return True
        await ClockCycles(dut.clk, 1)
    return False


async def run_pipeline(dut, timeout=60000):
    """Trigger one full wake→…→sleep→idle cycle.  Returns True if it finishes."""
    await pulse_wake(dut)
    if not await wait_for(dut, pwr_gate, 1, timeout=20):
        return False
    if not await wait_for(dut, pwr_gate, 0, timeout=timeout):
        return False
    # Also wait for LSK modulator to fully deassert (can lag FSM by 1 cycle)
    if not await wait_for(dut, lsk_tx, 0, timeout=200):
        return False
    # Let S_SLEEP → S_IDLE settle
    await ClockCycles(dut.clk, 2)
    return True


# ============================================================================
# 1  RESET
# ============================================================================

@cocotb.test()
async def test_01_reset(dut):
    """All outputs must be zero after reset."""
    await init(dut)

    assert safe_int(dut.uo_out) == 0, \
        f"uo_out = {safe_int(dut.uo_out):#04x}, expected 0x00"
    assert (safe_int(dut.uio_out) & 0x07) == 0, \
        "busy signals should be 0 after reset"
    dut._log.info("PASS: all outputs zeroed after reset")


# ============================================================================
# 2  IDLE STABILITY
# ============================================================================

@cocotb.test()
async def test_02_idle_stable(dut):
    """FSM must stay idle for 200 cycles when wake is not asserted."""
    await init(dut)
    await ClockCycles(dut.clk, 200)

    assert pwr_gate(dut) == 0,   "pwr_gate should be 0 in idle"
    assert lms_busy(dut) == 0,   "lms_busy  should be 0 in idle"
    assert dwt_busy(dut) == 0,   "dwt_busy  should be 0 in idle"
    assert lsk_tx(dut) == 0,     "lsk_tx    should be 0 in idle"
    assert processing(dut) == 0, "processing should be 0 in idle"
    dut._log.info("PASS: idle stable for 200 cycles")


# ============================================================================
# 3  CORDIC_BUSY HARD-WIRED ZERO (CORDIC removed)
# ============================================================================

@cocotb.test()
async def test_03_cordic_busy_always_zero(dut):
    """cordic_busy (uio_out[2]) must be 0 at all times (CORDIC removed)."""
    await init(dut)
    assert cordic_busy(dut) == 0, "cordic_busy != 0 at idle"

    # Also check during active processing
    await pulse_wake(dut)
    for _ in range(60):
        assert cordic_busy(dut) == 0, "cordic_busy != 0 during pipeline"
        await ClockCycles(dut.clk, 1)

    dut._log.info("PASS: cordic_busy always 0")


# ============================================================================
# 4  TWO-FF SYNCHRONIZER LATENCY
# ============================================================================

@cocotb.test()
async def test_04_sync_latency(dut):
    """Wake signal passes through 2-FF sync — not seen for at least 2 cycles."""
    await init(dut)

    dut.ui_in.value = build_ui(wake=1)

    # Cycles +1, +2: wake is still propagating through the 2-FF synchroniser
    await ClockCycles(dut.clk, 1)
    assert pwr_gate(dut) == 0, "cycle +1: should still be idle"
    await ClockCycles(dut.clk, 1)
    assert pwr_gate(dut) == 0, "cycle +2: should still be idle"

    # The FSM should become active within the next few cycles
    ok = await wait_for(dut, pwr_gate, 1, timeout=5)
    assert ok, "FSM never left IDLE after wake sync"

    dut.ui_in.value = 0
    dut._log.info("PASS: 2-FF sync verified (idle for ≥2 cycles, active within 5)")


# ============================================================================
# 5  LMS FILTER — BUSY FLAG
# ============================================================================

@cocotb.test()
async def test_05_lms_busy(dut):
    """LMS busy goes high after wake and clears when tap scan finishes."""
    await init(dut)

    # Fill LMS delay line with data
    for s in [3, 7, 2, 5, 1, 6, 4, 0]:
        await feed_sample(dut, s)

    await pulse_wake(dut)

    ok = await wait_for(dut, lms_busy, 1, timeout=20)
    assert ok, "lms_busy never asserted"
    dut._log.info("  lms_busy HIGH")

    ok = await wait_for(dut, lms_busy, 0, timeout=50)
    assert ok, "lms_busy never de-asserted"
    dut._log.info("PASS: LMS busy lifecycle OK")


# ============================================================================
# 6  DWT ENGINE — BUSY FLAG
# ============================================================================

@cocotb.test()
async def test_06_dwt_busy(dut):
    """DWT busy asserts after LMS completes and clears when DWT is done."""
    await init(dut)

    for s in [1, 2, 3, 4, 5, 6, 7, 0]:
        await feed_sample(dut, s)

    await pulse_wake(dut)

    ok = await wait_for(dut, dwt_busy, 1, timeout=50)
    assert ok, "dwt_busy never asserted"
    dut._log.info("  dwt_busy HIGH")

    ok = await wait_for(dut, dwt_busy, 0, timeout=50)
    assert ok, "dwt_busy never de-asserted"
    dut._log.info("PASS: DWT busy lifecycle OK")


# ============================================================================
# 7  POWER-GATE LIFECYCLE
# ============================================================================

@cocotb.test()
async def test_07_pwr_gate_lifecycle(dut):
    """pwr_gate_ctrl rises on wake and falls after full pipeline."""
    await init(dut)

    assert pwr_gate(dut) == 0, "should start idle"

    await pulse_wake(dut)
    ok = await wait_for(dut, pwr_gate, 1, timeout=10)
    assert ok, "pwr_gate never asserted"

    ok = await wait_for(dut, pwr_gate, 0, timeout=60000)
    assert ok, "pwr_gate never de-asserted (pipeline stuck)"
    dut._log.info("PASS: pwr_gate lifecycle correct")


# ============================================================================
# 8  LSK MODULATOR — MANCHESTER ENCODING
# ============================================================================

@cocotb.test()
async def test_08_lsk_transmission(dut):
    """LSK transmitter produces Manchester-encoded output during S_LSK_TX."""
    await init(dut)

    for s in [5, 3, 7, 1, 6, 2, 4, 0]:
        await feed_sample(dut, s)

    await pulse_wake(dut)

    ok = await wait_for(dut, lsk_tx, 1, timeout=200)
    assert ok, "lsk_tx never asserted"
    dut._log.info("  LSK transmission started")

    # Count lsk_ctrl transitions during TX — Manchester must toggle
    transitions = 0
    prev_ctrl = lsk_ctrl(dut)
    for _ in range(4000):
        await ClockCycles(dut.clk, 1)
        cur_ctrl = lsk_ctrl(dut)
        if cur_ctrl != prev_ctrl:
            transitions += 1
            prev_ctrl = cur_ctrl
        if lsk_tx(dut) == 0:
            break

    dut._log.info(f"  lsk_ctrl transitions: {transitions}")
    assert transitions >= 2, \
        f"Expected ≥2 lsk_ctrl transitions (Manchester), got {transitions}"
    assert lsk_tx(dut) == 0, "lsk_tx should be 0 after TX"
    dut._log.info("PASS: LSK Manchester encoding observed")


# ============================================================================
# 9  COMMAND ENCODER — cmd_valid PULSE
# ============================================================================

@cocotb.test()
async def test_09_cmd_valid_pulse(dut):
    """cmd_valid must pulse during the pipeline; cmd_out in [0,7]."""
    await init(dut)

    for s in [2, 4, 6, 7, 5, 3, 1, 0]:
        await feed_sample(dut, s)

    await pulse_wake(dut)

    ok = await wait_for(dut, cmd_valid, 1, timeout=200)
    assert ok, "cmd_valid never asserted"

    cmd = cmd_out(dut)
    dut._log.info(f"  cmd_out = {cmd}")
    assert 0 <= cmd <= 7, f"cmd_out out of range: {cmd}"
    dut._log.info("PASS: cmd_valid pulsed, cmd_out in range")


# ============================================================================
# 10  FULL PIPELINE — ZERO DATA
# ============================================================================

@cocotb.test()
async def test_10_full_pipeline_zero_data(dut):
    """Pipeline completes cleanly with zero-initialised buffers."""
    await init(dut)

    ok = await run_pipeline(dut)
    assert ok, "Pipeline did not complete (zero data)"
    dut._log.info("PASS: full pipeline (zero data) completed")


# ============================================================================
# 11  FULL PIPELINE — REAL ADC DATA
# ============================================================================

@cocotb.test()
async def test_11_full_pipeline_with_data(dut):
    """Feed 8 ADC samples then run one full pipeline."""
    await init(dut)

    samples = [2, 6, 1, 5, 3, 7, 0, 4]
    for s in samples:
        await feed_sample(dut, s)
    dut._log.info(f"  Fed ADC samples: {samples}")

    ok = await run_pipeline(dut)
    assert ok, "Pipeline did not complete with real data"
    dut._log.info("PASS: full pipeline (real data) completed")


# ============================================================================
# 12  MULTIPLE CONSECUTIVE WAKE CYCLES
# ============================================================================

@cocotb.test()
async def test_12_consecutive_pipelines(dut):
    """Three back-to-back pipeline runs must all complete."""
    await init(dut)

    for n in range(3):
        for s in range(8):
            await feed_sample(dut, (s + n * 3) & 0xF)
        ok = await run_pipeline(dut)
        assert ok, f"Pipeline stuck on run {n}"
        # run_pipeline already waits for pwr_gate=0, lsk_tx=0, plus settle
        assert pwr_gate(dut) == 0, f"Run {n}: pwr_gate not 0"
        assert lms_busy(dut) == 0, f"Run {n}: lms_busy not 0"
        assert dwt_busy(dut) == 0, f"Run {n}: dwt_busy not 0"
        assert lsk_tx(dut) == 0,   f"Run {n}: lsk_tx not 0"
        dut._log.info(f"  Run {n}: OK")

    dut._log.info("PASS: 3 consecutive pipelines completed")


# ============================================================================
# 13  DWT BUFFER ACCUMULATION (9 runs)
# ============================================================================

@cocotb.test()
async def test_13_dwt_buffer_fill(dut):
    """Run 9 pipeline cycles so the Haar DWT buffer holds real LMS outputs."""
    await init(dut)

    for run_idx in range(9):
        base = run_idx * 2
        for s in range(8):
            await feed_sample(dut, (base + s) & 0xF)
        ok = await run_pipeline(dut)
        assert ok, f"Pipeline stuck on run {run_idx}"

    dut._log.info("PASS: 9 pipeline runs (DWT buffer fully populated)")


# ============================================================================
# 14  PROCESSING FLAG = lms_busy | dwt_busy
# ============================================================================

@cocotb.test()
async def test_14_processing_flag(dut):
    """processing output (uo_out[7]) reflects lms_busy | dwt_busy."""
    await init(dut)
    assert processing(dut) == 0, "processing set before wake"

    for s in [1, 2, 3, 4, 5, 6, 7, 0]:
        await feed_sample(dut, s)

    await pulse_wake(dut)

    ok = await wait_for(dut, processing, 1, timeout=20)
    assert ok, "processing never went HIGH"
    dut._log.info("  processing went HIGH")

    # After pipeline finishes it should be 0 again
    await wait_for(dut, pwr_gate, 0, timeout=60000)
    assert processing(dut) == 0, "processing should be 0 after pipeline"
    dut._log.info("PASS: processing flag lifecycle correct")


# ============================================================================
# 15  UIO_OE DIRECTION REGISTER
# ============================================================================

@cocotb.test()
async def test_15_uio_oe(dut):
    """uio_oe must be 0x07 (pins 0-2 output, 3-7 input)."""
    await init(dut)

    oe = safe_int(dut.uio_oe)
    assert oe == 0x07, f"uio_oe = {oe:#04x}, expected 0x07"
    dut._log.info("PASS: uio_oe = 0x07")


# ============================================================================
# 16  SIGN EXTENSION — NEGATIVE ADC VALUES
# ============================================================================

@cocotb.test()
async def test_16_negative_adc(dut):
    """Pipeline handles negative 4-bit ADC codes (bit 3 = 1 → sign extend)."""
    await init(dut)

    # 4-bit two's complement negatives: 8→-8, 9→-7, … 15→-1
    neg = [8, 10, 12, 14, 9, 11, 13, 15]
    for s in neg:
        await feed_sample(dut, s)
    dut._log.info(f"  Fed negative samples: {neg}")

    ok = await run_pipeline(dut)
    assert ok, "Pipeline failed with negative data"
    dut._log.info("PASS: negative ADC data handled")


# ============================================================================
# 17  SIGN EXTENSION — MIXED POLARITY
# ============================================================================

@cocotb.test()
async def test_17_mixed_polarity(dut):
    """Pipeline handles a mix of positive and negative ADC samples."""
    await init(dut)

    mixed = [3, 12, 7, 8, 1, 14, 5, 10]
    for s in mixed:
        await feed_sample(dut, s)
    dut._log.info(f"  Fed mixed samples: {mixed}")

    ok = await run_pipeline(dut)
    assert ok, "Pipeline failed with mixed polarity data"
    dut._log.info("PASS: mixed polarity data processed")
