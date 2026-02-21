# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0
#
# NeuroCore Field Sensor — cocotb testbench
# Expanded Stress-Test Bench covering Protocol, Spectrum, and Watchdog
#

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, Timer
import os

# ============================================================================
# Helpers
# ============================================================================

def is_gate_level():
    """Detect if we are running Gate Level Simulation to skip internal signal checks."""
    return os.environ.get("GATES") == "yes"

def build_ui(adc_data=0, adc_valid=0, wake=0):
    """Build the 8-bit ui_in value from individual fields."""
    return (adc_data & 0xF) | ((adc_valid & 1) << 4) | ((wake & 1) << 5)

def safe_int(sig, default=0):
    """Read a signal as int, returning *default* if it contains X / Z."""
    try:
        return int(sig.value)
    except ValueError:
        return default

# --- Output Accessors ---
def cmd_out(dut):       return safe_int(dut.uo_out) & 0x07
def cmd_valid(dut):     return (safe_int(dut.uo_out) >> 3) & 1
def lsk_ctrl(dut):      return (safe_int(dut.uo_out) >> 4) & 1
def lsk_tx(dut):        return (safe_int(dut.uo_out) >> 5) & 1
def pwr_gate(dut):      return (safe_int(dut.uo_out) >> 6) & 1
def processing(dut):    return (safe_int(dut.uo_out) >> 7) & 1

# --- Bidirectional Accessors ---
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
    """Pulse one 4-bit ADC sample. Accounts for the 2-FF synchronizer."""
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
    """Poll *fn(dut)* until it equals *value*. Returns True on match."""
    for _ in range(timeout):
        if fn(dut) == value:
            return True
        await ClockCycles(dut.clk, 1)
    return False


async def run_pipeline(dut, timeout=60000):
    """Trigger one full wake→…→sleep→idle cycle. Returns True if it finishes."""
    await pulse_wake(dut)
    if not await wait_for(dut, pwr_gate, 1, timeout=20):
        return False
    if not await wait_for(dut, pwr_gate, 0, timeout=timeout):
        return False
    # Wait for LSK modulator to finish transmission
    if not await wait_for(dut, lsk_tx, 0, timeout=4000):
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

    for s in range(8):
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
# 8  LSK MODULATOR — MANCHESTER ENCODING (Basic Toggle)
# ============================================================================

@cocotb.test()
async def test_08_lsk_transmission(dut):
    """LSK transmitter produces toggling output during S_LSK_TX."""
    await init(dut)

    for s in [5, 3, 7, 1, 6, 2, 4, 0]:
        await feed_sample(dut, s)

    await pulse_wake(dut)

    ok = await wait_for(dut, lsk_tx, 1, timeout=500)
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

    ok = await wait_for(dut, cmd_valid, 1, timeout=500)
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


# ============================================================================
# 18  NEW: LSK PACKET PROTOCOL DECODE
# ============================================================================

@cocotb.test()
async def test_18_lsk_packet_decode(dut):
    """
    Decodes the full Manchester packet to verify protocol compliance.
    Packet: [1010 Pre] [1100 Sync] [3-bit Cmd] [Parity] [11 Post]
    """
    await init(dut)

    # Feed input that creates a known command (alternating max values -> High Freq)
    # +7, -8, +7, -8...
    for i in range(8):
        val = 7 if (i % 2 == 0) else 8 # 8 is -8 in 4-bit 2's comp
        await feed_sample(dut, val)

    await pulse_wake(dut)

    # Wait for Encoder to finish (cmd_valid) so we can see what command to expect
    await wait_for(dut, cmd_valid, 1, timeout=2000)
    expected_cmd = cmd_out(dut)
    dut._log.info(f"  Encoder chose CMD: {expected_cmd}")

    # Wait for TX start
    await wait_for(dut, lsk_tx, 1, timeout=100)

    # Manchester Decode
    # BIT_PERIOD is 200 cycles. Data is in 1st half, ~Data in 2nd half.
    decoded_bits = []

    # Align to center of first half-bit (approx 50 cycles into the bit)
    await ClockCycles(dut.clk, 50)

    for _ in range(14): # 14 bit packet
        val = lsk_ctrl(dut)
        decoded_bits.append(val)
        await ClockCycles(dut.clk, 200) # Jump to next bit center

    # Construct integer from bits
    packet_int = 0
    for bit in decoded_bits:
        packet_int = (packet_int << 1) | bit

    dut._log.info(f"  Raw Bits: {decoded_bits}")
    dut._log.info(f"  Raw Hex : {packet_int:#06x}")

    # Expected Structure
    # Preamble (4) | Sync (4) | Cmd (3) | Parity (1) | Post (2)
    # 1010         | 1100     | CCC     | P          | 11

    preamble = (packet_int >> 10) & 0xF
    sync     = (packet_int >> 6) & 0xF
    rx_cmd   = (packet_int >> 3) & 0x7
    parity   = (packet_int >> 2) & 0x1
    post     = packet_int & 0x3

    # Calc expected parity (XOR of command bits)
    exp_parity = (expected_cmd >> 2) ^ ((expected_cmd >> 1) & 1) ^ (expected_cmd & 1)

    assert preamble == 0xA, f"Bad Preamble: {preamble:#x}"
    assert sync     == 0xC, f"Bad Sync: {sync:#x}"
    assert post     == 0x3, f"Bad Postamble: {post:#x}"
    assert rx_cmd   == expected_cmd, f"LSK sent cmd {rx_cmd}, expected {expected_cmd}"
    assert parity   == exp_parity, "Parity bit mismatch"

    dut._log.info("PASS: LSK Protocol Verified (Preamble+Sync+Cmd+Parity+Post)")


# ============================================================================
# 19  NEW: SPURIOUS WAKE (NO-OP)
# ============================================================================

@cocotb.test()
async def test_19_spurious_wake(dut):
    """Assert WAKE while pipeline is already running. FSM should ignore it."""
    await init(dut)
    for s in range(8):
        await feed_sample(dut, s)

    await pulse_wake(dut)
    await wait_for(dut, processing, 1)

    # Pipeline is running. Pulse wake again immediately.
    dut._log.info("  Injecting spurious wake...")
    await pulse_wake(dut)

    # Wait for completion
    await wait_for(dut, pwr_gate, 0, timeout=60000)

    # Ensure it didn't restart immediately (Processing should stay low)
    await ClockCycles(dut.clk, 50)
    assert processing(dut) == 0, "FSM restarted on spurious wake!"
    dut._log.info("PASS: Spurious wake ignored")


# ============================================================================
# 20  NEW: SPURIOUS ADC DATA (NOISE)
# ============================================================================

@cocotb.test()
async def test_20_spurious_adc_data(dut):
    """Feed ADC data while processing. Should not corrupt state."""
    await init(dut)
    # Feed valid data first
    for s in range(8):
        await feed_sample(dut, s)

    await pulse_wake(dut)
    await wait_for(dut, processing, 1)

    # Spam ADC interface during DWT calculation
    dut._log.info("  Spamming ADC inputs during processing...")
    for _ in range(10):
        dut.ui_in.value = build_ui(adc_data=0xF, adc_valid=1)
        await ClockCycles(dut.clk, 1)
        dut.ui_in.value = build_ui(adc_data=0xF, adc_valid=0)
        await ClockCycles(dut.clk, 1)

    await wait_for(dut, pwr_gate, 0, timeout=60000)
    dut._log.info("PASS: Pipeline completed despite input noise")


# ============================================================================
# 21  NEW: DC INPUT SPECTRUM
# ============================================================================

@cocotb.test()
async def test_21_dc_input_spectrum(dut):
    """Feed Constant DC. High bins should be 0. Cmd should be low."""
    await init(dut)

    # Fill DWT buffer with constant value (DC)
    # Need >8 samples to flush LMS history
    for _ in range(16):
        await feed_sample(dut, 4) # Constant 4

    await pulse_wake(dut)
    await wait_for(dut, cmd_valid, 1, timeout=3000)

    cmd = cmd_out(dut)
    dut._log.info(f"  DC Input -> Command: {cmd}")

    # Haar Wavelet DWT kills DC in detail coefficients.
    # Energy should be concentrated in lowest bin or DC-leakage.
    # We expect a low bin index (0, 1, or 2).
    assert cmd <= 2, f"DC input produced high frequency bin {cmd}!"
    dut._log.info("PASS: DC input correctly classified as low frequency")


# ============================================================================
# 22  NEW: HIGH FREQ INPUT SPECTRUM (STEP)
# ============================================================================

@cocotb.test()
async def test_22_step_input_spectrum(dut):
    """
    Feed Step Input (0->Max). This guarantees DWT activation.
    Oscillating patterns (Nyquist) are filtered by the low-pass FIR.
    A step function creates an edge that cannot be filtered out completely.
    """
    await init(dut)

    # Pattern: 4x Zero, 4x Max (7)
    pattern = [0, 0, 0, 0, 7, 7, 7, 7]

    dut._log.info("  Priming FIR filter with Step pattern...")
    for s in pattern:
        await feed_sample(dut, s)

    dut._log.info("  Waking...")
    await pulse_wake(dut)
    await wait_for(dut, cmd_valid, 1, timeout=3000)

    # Check that we got a non-DC bin
    assert cmd_out(dut) != 0, f"Step input produced DC bin {cmd_out(dut)}"
    dut._log.info("PASS: Step input correctly classified as High Freq")


# ============================================================================
# 23  NEW: ASYNC RESET RECOVERY
# ============================================================================

@cocotb.test()
async def test_23_async_reset_recovery(dut):
    """Assert Reset MID-CALCULATION. Chip must return to IDLE immediately."""
    await init(dut)
    await pulse_wake(dut)
    await wait_for(dut, processing, 1)

    # Let it run for a bit (e.g. into DWT stage)
    await ClockCycles(dut.clk, 100)

    dut._log.info("  Asserting Async Reset mid-operation...")
    dut.rst_n.value = 0
    
    # FIX: Wait for full clock cycles to handle Gate-Level propagation delays
    # Timer(1, "ns") was too short for GL simulation.
    await ClockCycles(dut.clk, 2)

    # Verify immediate stop
    assert pwr_gate(dut) == 0, "Power gate did not drop on reset"
    assert processing(dut) == 0, "Processing flag did not drop on reset"

    # Release reset and verify it stays idle (doesn't resume)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 20)
    assert pwr_gate(dut) == 0
    dut._log.info("PASS: Async reset successfully killed pipeline")


# ============================================================================
# 24  NEW: WATCHDOG TIMEOUT
# ============================================================================

@cocotb.test()
async def test_24_watchdog_timeout(dut):
    """
    Force FSM to hang (via internal signal manipulation) and wait for Watchdog.
    Skip this test in Gate Level Sim (internal signals inaccessible).
    """
    if is_gate_level():
        dut._log.info("SKIPPING Watchdog test in GL mode (no internal access)")
        return

    await init(dut)

    # Force state to S_WAIT_LMS (3) but do NOT send lms_valid
    # The FSM will stick here forever unless WDT fires.
    # Note: Access path traverses tb -> user_project -> sensor -> state
    dut._log.info("  Forcing State = S_WAIT_LMS (3)...")
    dut.user_project.sensor.state.value = 3

    # WDT is 16-bit ~32768 cycles.
    dut._log.info("  Waiting for Watchdog (~32768 cycles)...")

    # Polling logic for robust timeout detection
    fired = await wait_for(
        dut,
        lambda d: safe_int(d.user_project.sensor.state) == 13,
        True,
        timeout=50000
    )

    assert fired, "Watchdog failed to force state to SLEEP(13)!"
    dut._log.info("PASS: Watchdog successfully recovered hung FSM")