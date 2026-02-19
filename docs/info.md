<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

NeuroCore Field Sensor is an ultra-low-power digital magnetic field sensing system designed for event-driven operation.

The processing pipeline:

1. **Level-Crossing ADC** – A 4-bit event-driven ADC that only outputs data on threshold crossings.
2. **Event Detector** – An always-on comparator (~1 nW) that generates a WAKE pulse to bring the digital core out of sleep.
3. **LMS Adaptive Filter** – An 8-tap shift-add filter that removes baseline artifacts from the signal.
4. **Lifting DWT** – A 3-level discrete wavelet transform for time-frequency decomposition.
5. **CORDIC Phase Extractor** – A 12-iteration CORDIC that computes magnitude and phase of each sub-band.
6. **Power Accumulator** – Accumulates power across 8 frequency bins.
7. **Command Encoder** – Encodes the dominant bin into a 3-bit command.
8. **LSK Modulator** – Drives an LSK backscatter output to transmit the command.

The entire digital core is power-gated and wakes only when the comparator fires, targeting < 20 nW peak active power.

## How to test

1. Apply a 4-bit stimulus on `ui_in[3:0]` (ADC data) and pulse `ui_in[4]` (ADC valid).
2. Pulse `ui_in[5]` (WAKE) to bring the digital core out of idle.
3. Observe the 3-bit command on `uo_out[2:0]` and `uo_out[3]` (command valid).
4. Monitor `uo_out[4]` (LSK control) and `uo_out[5]` (LSK TX active).
5. Check processing status on `uio_out[0:2]` (LMS/DWT/CORDIC busy).

## External hardware

No external hardware is required for basic digital simulation. For full system testing, an external magnetic field source and an analog front-end (spiral inductor + passive integrator) would drive the ADC inputs.
