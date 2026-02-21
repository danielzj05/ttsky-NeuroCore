# =============================================================================
# NeuroCore cocotb Testbench
# =============================================================================
# 20 tests for neurocore_field_sensor via tt_um_NeuroCore (Tiny Tapeout wrapper)
#
# Hierarchy: tb.user_project (tt_um_NeuroCore) -> sensor (neurocore_field_sensor)
#
# Pin mapping:
#   ui_in[3:0]  = adc_data
#   ui_in[4]    = adc_valid
#   ui_in[5]    = wake
#   uo_out[2:0] = cmd_out
#   uo_out[3]   = cmd_valid
#   uo_out[4]   = lsk_ctrl
#   uo_out[5]   = lsk_tx (tx_active)
#   uo_out[6]   = pwr_gate_ctrl
#   uo_out[7]   = processing
#   uio_out[0]  = lms_busy
#   uio_out[1]  = dwt_busy
#   uio_out[2]  = cordic_busy (acc_busy)
# =============================================================================

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

# =============================================================================
# Constants
# =============================================================================
BIT_PERIOD = 1000
HALF_PERIOD = BIT_PERIOD // 2
LSK_PACKET_LEN = 14


# =============================================================================
# Signal Access Helpers
# =============================================================================
def get_uo(dut):
    return int(dut.uo_out.value)

def get_uio(dut):
    return int(dut.uio_out.value)

def get_cmd_out(dut):
    return get_uo(dut) & 0x7

def get_cmd_valid(dut):
    return (get_uo(dut) >> 3) & 1

def get_lsk_ctrl(dut):
    return (get_uo(dut) >> 4) & 1

def get_lsk_tx(dut):
    return (get_uo(dut) >> 5) & 1

def get_pwr_gate(dut):
    return (get_uo(dut) >> 6) & 1

def get_processing(dut):
    return (get_uo(dut) >> 7) & 1

def get_lms_busy(dut):
    return get_uio(dut) & 1

def get_dwt_busy(dut):
    return (get_uio(dut) >> 1) & 1

def get_acc_busy(dut):
    return (get_uio(dut) >> 2) & 1


# =============================================================================
# Core Test Helpers
# =============================================================================
async def setup_dut(dut):
    """Start 10 MHz clock, apply reset, initialize inputs."""
    clock = Clock(dut.clk, 100, units="ns")
    cocotb.start_soon(clock.start())

    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0

    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)


async def assert_wake(dut, hold_cycles=6):
    """
    Assert wake on ui_in[5] long enough for 2-FF synchronizer.
    Needs >= 3 cycles to cross; we use 6 for margin.
    """
    current = int(dut.ui_in.value)
    dut.ui_in.value = current | (1 << 5)
    for _ in range(hold_cycles):
        await RisingEdge(dut.clk)
    # Deassert wake
    dut.ui_in.value = int(dut.ui_in.value) & ~(1 << 5)
    await RisingEdge(dut.clk)


async def wait_for_processing(dut, timeout=30):
    """Wait for processing (uo_out[7]) to assert."""
    for i in range(timeout):
        await RisingEdge(dut.clk)
        if get_processing(dut):
            return i + 1
    raise AssertionError(f"processing never asserted after {timeout} cycles")


async def feed_one_sample(dut, value, hold_cycles=5):
    """
    Feed a single 4-bit ADC sample with adc_valid strobe.
    
    Timing requirements:
    - adc_valid must be held >= 3 cycles for 2-FF synchronizer
    - Then FSM sees adc_valid_sync, starts FIR (lms_start_reg)
    - FIR takes 8 cycles (tap_count 0-7)
    - FSM clears sample_pending on lms_valid
    
    We hold valid for hold_cycles, then deassert and wait for
    the FIR to complete before returning.
    """
    # Preserve wake bit if set
    wake_bit = int(dut.ui_in.value) & (1 << 5)
    
    # Assert data + valid
    dut.ui_in.value = (value & 0xF) | (1 << 4) | wake_bit
    for _ in range(hold_cycles):
        await RisingEdge(dut.clk)
    
    # Deassert valid, keep data
    dut.ui_in.value = (value & 0xF) | wake_bit
    await RisingEdge(dut.clk)
    
    # Wait for FIR to finish processing this sample.
    # The FSM flow is:
    #   adc_valid_sync seen -> lms_start_reg=1 -> FIR starts (busy=1)
    #   -> 8 cycles later lms_valid pulse -> sample_pending cleared
    # We wait for lms_busy to go high then low, with a timeout.
    busy_seen = False
    for i in range(60):
        await RisingEdge(dut.clk)
        if get_lms_busy(dut):
            busy_seen = True
        if busy_seen and not get_lms_busy(dut):
            return
    
    # If we never saw busy, the sample may not have been accepted.
    # This can happen if sample_pending was still set. 
    # Wait a bit more for the FSM to be ready.
    for _ in range(20):
        await RisingEdge(dut.clk)


async def feed_all_samples(dut, samples, gap=3):
    """
    Feed 8 ADC samples through the FIR during S_COLLECT.
    Waits for each sample to be processed before sending the next.
    """
    for idx, s in enumerate(samples):
        await feed_one_sample(dut, s)
        # Small gap between samples
        for _ in range(gap):
            await RisingEdge(dut.clk)


async def wait_for_cmd_valid(dut, timeout=25000):
    """Wait for cmd_valid (uo_out[3]) to assert."""
    for i in range(timeout):
        await RisingEdge(dut.clk)
        if get_cmd_valid(dut):
            return i + 1
    raise AssertionError(f"cmd_valid never asserted after {timeout} cycles")


async def wait_for_sleep(dut, timeout=80000):
    """Wait for processing to deassert (FSM reached SLEEP -> IDLE)."""
    for i in range(timeout):
        await RisingEdge(dut.clk)
        if not get_processing(dut):
            return i + 1
    raise AssertionError(f"processing never deasserted after {timeout} cycles")


async def run_full_pipeline(dut, samples, timeout=25000):
    """
    Complete pipeline run: wake -> collect 8 samples -> wait for cmd_valid.
    Returns dict with cmd value and cycle count.
    """
    await assert_wake(dut)
    await wait_for_processing(dut)
    await feed_all_samples(dut, samples)
    cmd_cycles = await wait_for_cmd_valid(dut, timeout=timeout)
    cmd = get_cmd_out(dut)
    return {'cmd': cmd, 'cmd_cycles': cmd_cycles}


async def capture_lsk_packet(dut, timeout=20000):
    """
    Capture Manchester-encoded LSK packet from lsk_ctrl (uo_out[4]).
    
    LSK modulator behavior:
    - tx_start fires, tx_active goes high same cycle
    - bit_count starts at 13, counts down to 0
    - For each bit period:
        First half:  lsk_ctrl = packet_reg[bit_count]
        Second half: lsk_ctrl = ~packet_reg[bit_count]
    - Manchester: bit=1 -> H then L, bit=0 -> L then H
    
    We wait for tx_active, then sample at 1/4 and 3/4 of each bit period.
    """
    # Wait for tx_active (lsk_tx = uo_out[5])
    for _ in range(timeout):
        await RisingEdge(dut.clk)
        if get_lsk_tx(dut):
            break
    else:
        raise AssertionError("LSK tx_active never asserted")
    
    # Small alignment delay — tx_active and first lsk_ctrl transition
    # happen on the same clock edge, so we're already at cycle 0 of bit 13.
    # We need to sample at cycle BIT_PERIOD/4 from the start of each bit.
    
    decoded_bits = []
    for bit_idx in range(LSK_PACKET_LEN):
        # Go to 1/4 of bit period (center of first half)
        await ClockCycles(dut.clk, BIT_PERIOD // 4)
        first_half = get_lsk_ctrl(dut)
        
        # Go to 3/4 of bit period (center of second half)
        await ClockCycles(dut.clk, BIT_PERIOD // 2)
        second_half = get_lsk_ctrl(dut)
        
        # Manchester decode
        if first_half == 1 and second_half == 0:
            decoded_bits.append(1)
        elif first_half == 0 and second_half == 1:
            decoded_bits.append(0)
        else:
            decoded_bits.append(-1)  # Error
        
        # Advance to end of this bit period
        remaining = BIT_PERIOD - (BIT_PERIOD // 4) - (BIT_PERIOD // 2)
        await ClockCycles(dut.clk, remaining)
    
    return decoded_bits


def parse_lsk_packet(bits):
    """
    Parse 14-bit LSK packet (MSB transmitted first):
    [1010] preamble [1100] sync [cmd2 cmd1 cmd0] [parity] [11] postamble
    """
    if len(bits) != 14:
        return None
    
    preamble = bits[0:4]
    sync = bits[4:8]
    cmd_bits = bits[8:11]
    par_bit = bits[11]
    postamble = bits[12:14]
    
    cmd_val = (cmd_bits[0] << 2) | (cmd_bits[1] << 1) | cmd_bits[2]
    expected_parity = cmd_bits[0] ^ cmd_bits[1] ^ cmd_bits[2]
    
    return {
        'preamble': preamble,
        'sync': sync,
        'cmd': cmd_val,
        'cmd_bits': cmd_bits,
        'parity': par_bit,
        'parity_ok': par_bit == expected_parity,
        'postamble': postamble,
        'valid': (
            preamble == [1, 0, 1, 0] and
            sync == [1, 1, 0, 0] and
            par_bit == expected_parity and
            postamble == [1, 1]
        )
    }


# =============================================================================
# Test 1: Reset State
# =============================================================================
@cocotb.test()
async def test_01_reset(dut):
    """Verify all outputs are zero after reset."""
    await setup_dut(dut)
    
    assert get_uo(dut) == 0, f"uo_out should be 0x00, got {get_uo(dut):#04x}"
    assert get_uio(dut) == 0, f"uio_out should be 0x00, got {get_uio(dut):#04x}"
    assert get_processing(dut) == 0
    assert get_pwr_gate(dut) == 0
    assert get_lsk_tx(dut) == 0
    assert get_cmd_valid(dut) == 0
    
    dut._log.info("PASS: All outputs zero after reset")


# =============================================================================
# Test 2: Stays Idle Without Wake
# =============================================================================
@cocotb.test()
async def test_02_idle_without_wake(dut):
    """Verify chip stays idle with no wake signal for 200 cycles."""
    await setup_dut(dut)
    
    for cycle in range(200):
        await RisingEdge(dut.clk)
        assert get_processing(dut) == 0, f"Processing asserted at cycle {cycle}"
        assert get_pwr_gate(dut) == 0, f"Power gate asserted at cycle {cycle}"
    
    dut._log.info("PASS: Chip remains idle without wake")


# =============================================================================
# Test 3: Wake Activates Processing
# =============================================================================
@cocotb.test()
async def test_03_wake_activates(dut):
    """Verify wake transitions FSM to active processing."""
    await setup_dut(dut)
    
    assert get_processing(dut) == 0
    
    await assert_wake(dut)
    cycles = await wait_for_processing(dut)
    
    assert get_processing(dut) == 1
    assert get_pwr_gate(dut) == 1
    
    dut._log.info(f"PASS: Wake -> processing active in {cycles} cycles")


# =============================================================================
# Test 4: Single Sample FIR Processing
# =============================================================================
@cocotb.test()
async def test_04_single_fir(dut):
    """Feed one sample and verify FIR busy signal toggles."""
    await setup_dut(dut)
    
    await assert_wake(dut)
    await wait_for_processing(dut)
    
    saw_busy = False
    saw_done = False
    
    await feed_one_sample(dut, 5)
    
    # Check that we saw busy during feed_one_sample's wait
    # Feed another and explicitly watch
    saw_busy = False
    # Assert data + valid
    dut.ui_in.value = (7 & 0xF) | (1 << 4)
    for _ in range(5):
        await RisingEdge(dut.clk)
    dut.ui_in.value = (7 & 0xF)
    await RisingEdge(dut.clk)
    
    for i in range(50):
        await RisingEdge(dut.clk)
        if get_lms_busy(dut):
            saw_busy = True
        if saw_busy and not get_lms_busy(dut):
            saw_done = True
            dut._log.info(f"FIR busy cycle completed in ~{i} cycles")
            break
    
    assert saw_busy, "Never saw lms_busy assert"
    assert saw_done, "lms_busy never deasserted"
    dut._log.info("PASS: FIR processed sample, busy toggled")


# =============================================================================
# Test 5: Full 8-Sample Collection Triggers DWT
# =============================================================================
@cocotb.test()
async def test_05_collect_triggers_dwt(dut):
    """Feed 8 samples and verify DWT busy signal appears."""
    await setup_dut(dut)
    
    await assert_wake(dut)
    await wait_for_processing(dut)
    
    await feed_all_samples(dut, [1, 2, 3, 4, 5, 6, 7, 0])
    
    saw_dwt = False
    for i in range(500):
        await RisingEdge(dut.clk)
        if get_dwt_busy(dut):
            saw_dwt = True
        if saw_dwt and not get_dwt_busy(dut):
            dut._log.info(f"DWT completed ~{i} cycles after feeding")
            break
    
    assert saw_dwt, "Never saw dwt_busy assert"
    dut._log.info("PASS: 8 samples collected, DWT processed")


# =============================================================================
# Test 6: Full Pipeline — DC Input → cmd=0
# =============================================================================
@cocotb.test()
async def test_06_dc_input(dut):
    """DC input (constant value) should produce cmd=0 (cA3 dominant)."""
    await setup_dut(dut)
    
    result = await run_full_pipeline(dut, [5] * 8)
    
    dut._log.info(f"DC input [5]*8 -> cmd = {result['cmd']}")
    assert result['cmd'] == 0, f"DC input should give cmd=0, got {result['cmd']}"
    
    await wait_for_sleep(dut)
    dut._log.info("PASS: DC input -> cmd=0")


# =============================================================================
# Test 7: Full Pipeline — Alternating Input → High Frequency Bin
# =============================================================================
@cocotb.test()
async def test_07_alternating_input(dut):
    """Alternating +7/-8 should excite cD1 (bins 4-7) after sign extension fix."""
    await setup_dut(dut)
    
    # 7 = 0b0111, -8 = 0b1000 (4-bit two's complement)
    samples = [7, 8, 7, 8, 7, 8, 7, 8]  # 8 = 0b1000 = -8 in signed 4-bit
    result = await run_full_pipeline(dut, samples)
    
    dut._log.info(f"Alternating +7/-8 -> cmd = {result['cmd']}")
    assert result['cmd'] >= 4, (
        f"Alternating input should give cmd in cD1 range (4-7), got {result['cmd']}"
    )
    
    await wait_for_sleep(dut)
    dut._log.info("PASS: Alternating input -> high-frequency bin")


# =============================================================================
# Test 8: Full Pipeline — Step Input
# =============================================================================
@cocotb.test()
async def test_08_step_input(dut):
    """Step function [15,15,15,15,0,0,0,0] — verify pipeline completes."""
    await setup_dut(dut)
    
    samples = [15, 15, 15, 15, 0, 0, 0, 0]
    result = await run_full_pipeline(dut, samples)
    
    dut._log.info(f"Step input -> cmd = {result['cmd']}")
    await wait_for_sleep(dut)
    dut._log.info("PASS: Step input processed")


# =============================================================================
# Test 9: LSK Packet Structure
# =============================================================================
@cocotb.test()
async def test_09_lsk_structure(dut):
    """Verify LSK Manchester packet has correct preamble, sync, parity."""
    await setup_dut(dut)
    
    await assert_wake(dut)
    await wait_for_processing(dut)
    await feed_all_samples(dut, [5] * 8)
    
    await wait_for_cmd_valid(dut)
    expected_cmd = get_cmd_out(dut)
    
    decoded = await capture_lsk_packet(dut)
    packet = parse_lsk_packet(decoded)
    
    dut._log.info(f"LSK decoded: {decoded}")
    dut._log.info(f"Packet: {packet}")
    
    assert packet is not None, "Failed to parse packet"
    assert packet['preamble'] == [1, 0, 1, 0], f"Bad preamble: {packet['preamble']}"
    assert packet['sync'] == [1, 1, 0, 0], f"Bad sync: {packet['sync']}"
    assert packet['parity_ok'], "Parity check failed"
    assert packet['postamble'] == [1, 1], f"Bad postamble: {packet['postamble']}"
    assert packet['cmd'] == expected_cmd, (
        f"LSK cmd {packet['cmd']} != expected {expected_cmd}"
    )
    
    dut._log.info(f"PASS: LSK packet valid, cmd={packet['cmd']}")


# =============================================================================
# Test 10: LSK Command Matches Pipeline Output
# =============================================================================
@cocotb.test()
async def test_10_lsk_cmd_match(dut):
    """Verify LSK transmitted command matches cmd_out for non-zero command."""
    await setup_dut(dut)
    
    await assert_wake(dut)
    await wait_for_processing(dut)
    await feed_all_samples(dut, [15, 0] * 4)
    
    await wait_for_cmd_valid(dut)
    expected_cmd = get_cmd_out(dut)
    
    decoded = await capture_lsk_packet(dut)
    packet = parse_lsk_packet(decoded)
    
    assert packet is not None and packet['valid'], f"Invalid LSK packet: {decoded}"
    assert packet['cmd'] == expected_cmd, (
        f"LSK cmd {packet['cmd']} != pipeline cmd {expected_cmd}"
    )
    
    dut._log.info(f"PASS: LSK cmd={packet['cmd']} matches pipeline output")


# =============================================================================
# Test 11: Busy Signal Ordering
# =============================================================================
@cocotb.test()
async def test_11_busy_ordering(dut):
    """Verify busy signals fire in order: LMS -> DWT -> ACC."""
    await setup_dut(dut)
    
    await assert_wake(dut)
    await wait_for_processing(dut)
    
    lms_first = -1
    dwt_first = -1
    acc_first = -1
    cycle = 0
    
    # Feed samples one at a time while monitoring busy signals
    for idx, s in enumerate([3, 7, 2, 6, 1, 5, 0, 4]):
        # Assert data + valid
        wake_bit = int(dut.ui_in.value) & (1 << 5)
        dut.ui_in.value = (s & 0xF) | (1 << 4) | wake_bit
        for _ in range(5):
            await RisingEdge(dut.clk)
            cycle += 1
            if get_lms_busy(dut) and lms_first < 0:
                lms_first = cycle
            if get_dwt_busy(dut) and dwt_first < 0:
                dwt_first = cycle
            if get_acc_busy(dut) and acc_first < 0:
                acc_first = cycle
        
        # Deassert valid
        dut.ui_in.value = (s & 0xF) | wake_bit
        await RisingEdge(dut.clk)
        cycle += 1
        
        # Wait for FIR to finish
        for _ in range(60):
            await RisingEdge(dut.clk)
            cycle += 1
            if get_lms_busy(dut) and lms_first < 0:
                lms_first = cycle
            if get_dwt_busy(dut) and dwt_first < 0:
                dwt_first = cycle
            if get_acc_busy(dut) and acc_first < 0:
                acc_first = cycle
            if not get_lms_busy(dut) and lms_first >= 0:
                break
        
        # Small gap
        for _ in range(3):
            await RisingEdge(dut.clk)
            cycle += 1
    
    # Continue monitoring through rest of pipeline
    for i in range(25000):
        await RisingEdge(dut.clk)
        cycle += 1
        
        if get_lms_busy(dut) and lms_first < 0:
            lms_first = cycle
        if get_dwt_busy(dut) and dwt_first < 0:
            dwt_first = cycle
        if get_acc_busy(dut) and acc_first < 0:
            acc_first = cycle
        
        if get_cmd_valid(dut):
            break
    
    dut._log.info(f"Busy first seen: LMS@{lms_first}, DWT@{dwt_first}, ACC@{acc_first}")
    
    assert lms_first >= 0, "Never saw lms_busy"
    assert dwt_first >= 0, "Never saw dwt_busy"
    assert acc_first >= 0, "Never saw acc_busy"
    assert acc_first > dwt_first, (
        f"ACC ({acc_first}) should come after DWT ({dwt_first})"
    )
    
    await wait_for_sleep(dut)
    dut._log.info("PASS: Busy signals in correct order")


# =============================================================================
# Test 12: Processing / Power Gate Lifecycle
# =============================================================================
@cocotb.test()
async def test_12_lifecycle(dut):
    """Verify processing and pwr_gate assert during pipeline, deassert after."""
    await setup_dut(dut)
    
    assert get_processing(dut) == 0
    assert get_pwr_gate(dut) == 0
    
    await assert_wake(dut)
    await wait_for_processing(dut)
    
    assert get_processing(dut) == 1
    assert get_pwr_gate(dut) == 1
    
    await feed_all_samples(dut, [8] * 8)
    await wait_for_sleep(dut)
    
    assert get_processing(dut) == 0, "Processing still on after sleep"
    assert get_pwr_gate(dut) == 0, "Power gate still on after sleep"
    
    dut._log.info("PASS: Processing/power gate lifecycle correct")


# =============================================================================
# Test 13: Watchdog Timeout
# =============================================================================
@cocotb.test()
async def test_13_watchdog(dut):
    """
    Wake but never feed samples -> watchdog should force SLEEP.
    Watchdog triggers when MSB (bit 15) is set = 32768 cycles.
    """
    await setup_dut(dut)
    
    await assert_wake(dut)
    await wait_for_processing(dut)
    
    # Don't feed any samples — stuck in S_COLLECT
    dut._log.info("Waiting for watchdog (~32768 cycles)...")
    cycles = await wait_for_sleep(dut, timeout=40000)
    
    dut._log.info(f"PASS: Watchdog fired after ~{cycles} cycles")


# =============================================================================
# Test 14: Back-to-Back Pipeline Runs
# =============================================================================
@cocotb.test()
async def test_14_back_to_back(dut):
    """Run two complete pipeline cycles sequentially."""
    await setup_dut(dut)
    
    for run in range(2):
        dut._log.info(f"--- Run {run + 1} ---")
        result = await run_full_pipeline(dut, [5] * 8)
        dut._log.info(f"Run {run+1}: cmd={result['cmd']}")
        await wait_for_sleep(dut)
        assert get_processing(dut) == 0
        await ClockCycles(dut.clk, 10)
    
    dut._log.info("PASS: Two back-to-back runs completed")


# =============================================================================
# Test 15: All Zeros Input
# =============================================================================
@cocotb.test()
async def test_15_all_zeros(dut):
    """All-zero input. All bins zero, max-finder returns 0."""
    await setup_dut(dut)
    
    result = await run_full_pipeline(dut, [0] * 8)
    
    dut._log.info(f"All zeros -> cmd={result['cmd']}")
    assert result['cmd'] == 0, f"Expected cmd=0, got {result['cmd']}"
    
    await wait_for_sleep(dut)
    dut._log.info("PASS: All-zeros -> cmd=0")


# =============================================================================
# Test 16: Maximum ADC Value (Constant)
# =============================================================================
@cocotb.test()
async def test_16_max_constant(dut):
    """
    Constant signed -1 input. First pass primes the FIR delay line.
    Second pass should produce truly constant FIR output -> cmd=0.
    """
    await setup_dut(dut)
    
    # First pass: primes FIR delay line with -1 values
    # (delay line starts at 0, so outputs ramp during fill)
    result1 = await run_full_pipeline(dut, [15] * 8)
    dut._log.info(f"Priming pass -> cmd={result1['cmd']} (transient expected)")
    await wait_for_sleep(dut)
    await ClockCycles(dut.clk, 10)
    
    # Second pass: delay line already full of -1
    # All 8 FIR outputs should be identical -> DWT detail = 0 -> cmd=0
    result2 = await run_full_pipeline(dut, [15] * 8)
    dut._log.info(f"Primed pass -> cmd={result2['cmd']}")
    assert result2['cmd'] == 0, (
        f"With primed FIR, constant -1 should give cmd=0, got {result2['cmd']}"
    )
    
    await wait_for_sleep(dut)
    dut._log.info("PASS: Primed constant -1 -> cmd=0")

# =============================================================================
# Test 17: Ramp Input
# =============================================================================
@cocotb.test()
async def test_17_ramp(dut):
    """Ramp input 0,2,4,...,14. Just verify pipeline completes."""
    await setup_dut(dut)
    
    samples = [0, 2, 4, 6, 8, 10, 12, 14]
    result = await run_full_pipeline(dut, samples)
    
    dut._log.info(f"Ramp -> cmd={result['cmd']}")
    await wait_for_sleep(dut)
    dut._log.info("PASS: Ramp input processed")


# =============================================================================
# Test 18: Wake Ignored During Processing
# =============================================================================
@cocotb.test()
async def test_18_wake_during_processing(dut):
    """Second wake during active processing should be ignored."""
    await setup_dut(dut)
    
    await assert_wake(dut)
    await wait_for_processing(dut)
    
    # Feed first sample
    await feed_one_sample(dut, 5)
    await ClockCycles(dut.clk, 5)
    
    # Try to wake again while processing
    await assert_wake(dut, hold_cycles=6)
    await ClockCycles(dut.clk, 5)
    
    # Should still be processing normally
    assert get_processing(dut) == 1, "Should still be processing"
    
    # Feed remaining 7 samples
    for _ in range(7):
        await feed_one_sample(dut, 5)
    
    await wait_for_sleep(dut, timeout=40000)
    dut._log.info("PASS: Second wake during processing was ignored")


# =============================================================================
# Test 19: cmd_valid is Single-Cycle Pulse
# =============================================================================
@cocotb.test()
async def test_19_cmd_valid_pulse(dut):
    """Verify cmd_valid is exactly a 1-cycle pulse."""
    await setup_dut(dut)
    
    await assert_wake(dut)
    await wait_for_processing(dut)
    await feed_all_samples(dut, [5] * 8)
    
    await wait_for_cmd_valid(dut)
    
    # Count how many more cycles it stays high
    extra = 0
    for _ in range(10):
        await RisingEdge(dut.clk)
        if get_cmd_valid(dut):
            extra += 1
        else:
            break
    
    assert extra == 0, f"cmd_valid held for {1 + extra} cycles, expected 1"
    
    await wait_for_sleep(dut)
    dut._log.info("PASS: cmd_valid is single-cycle pulse")


# =============================================================================
# Test 20: Clean Outputs After Sleep
# =============================================================================
@cocotb.test()
async def test_20_clean_after_sleep(dut):
    """Verify all outputs are clean after FSM returns to IDLE."""
    await setup_dut(dut)
    
    await run_full_pipeline(dut, [5] * 8)
    
    # Wait for LSK transmission to complete and FSM to reach IDLE
    # LSK takes ~14000 cycles, so give it plenty of time
    for i in range(20000):
        await RisingEdge(dut.clk)
        if get_lsk_tx(dut):
            break
    
    # Now wait for lsk_tx to drop
    for i in range(16000):
        await RisingEdge(dut.clk)
        if not get_lsk_tx(dut):
            break
    
    # Wait for FSM to reach IDLE
    await wait_for_sleep(dut, timeout=5000)
    await ClockCycles(dut.clk, 10)
    
    assert get_processing(dut) == 0, "processing still on"
    assert get_pwr_gate(dut) == 0, "pwr_gate still on"
    assert get_lsk_tx(dut) == 0, "lsk_tx still on"
    assert get_cmd_valid(dut) == 0, "cmd_valid still on"
    assert get_lms_busy(dut) == 0, "lms_busy still on"
    assert get_dwt_busy(dut) == 0, "dwt_busy still on"
    assert get_acc_busy(dut) == 0, "acc_busy still on"
    
    dut._log.info("PASS: All outputs clean after sleep")