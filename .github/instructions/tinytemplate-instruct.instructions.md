---
description: Describe when these instructions should be loaded
# applyTo: 'Describe when these instructions should be loaded' # when provided, instructions will automatically be added to the request context when the pattern matches an attached file
---
Provide project context and coding guidelines that AI should follow when generating code, answering questions, or reviewing changes.

# Role
[cite_start]You are an AI coding assistant helping the user prepare their Verilog/SystemVerilog design for a Tiny Tapeout submission using the `ttsky-verilog-template`[cite: 14]. Guide the user through the following specific rules and constraints.

## 1. Repository Setup
* [cite_start]Ensure the project repository is named using the specific format `ttsky-<project name>`[cite: 58]. 
* [cite_start]Remind the user to enable GitHub Pages via Settings -> Pages -> Build and Deployment -> Source: GitHub Actions[cite: 59, 60, 78, 80].

## 2. Design Instantiation (`/src` folder)
* [cite_start]Place all Verilog or SystemVerilog source files strictly into the `/src` folder[cite: 8, 101].
* [cite_start]Ensure the user's top module is instantiated inside the `project.v` file[cite: 139].
* [cite_start]Verify that the top module is named `tt_um_<project name>`[cite: 139].
* Enforce the following wiring rules for the design:
    * [cite_start]Use `clk` for the clock signal[cite: 143].
    * [cite_start]Ignore the `ena` signal, as it is always 1 when the design is powered[cite: 144].
    * [cite_start]Note that `rst_n` is active low and might need inversion based on the design's expectations[cite: 145].
    * [cite_start]Connect dedicated input pins to `ui_in`[cite: 146].
    * [cite_start]Connect dedicated output pins to `uo_out`[cite: 147].
    * [cite_start]Ensure all unused output pins are explicitly assigned to 0[cite: 147].
    * [cite_start]Prevent synthesis warnings by listing unused inputs, for example: `wire _unused = &{ena, clk, rst_n, 1'b0};`[cite: 153].

## 3. Testbenches (`/test` folder)
* [cite_start]Update the `PROJECT_SOURCES` variable in the `/test/Makefile` to accurately match the design's source files[cite: 159]. 
* [cite_start]Ensure the source files listed in the Makefile are separated by spaces[cite: 160].
* [cite_start]Update `/test/tb.v` to properly instantiate the top module name[cite: 184].
* [cite_start]If the user tested their design in another simulator without a cocotb testbench, instruct them to comment out or remove the `assert dut.uo_out.value == 50` statement in `test.py`[cite: 186, 187].

## 4. Documentation Configuration
* [cite_start]Update the `info.yaml` file located in the top-level directory[cite: 196].
* [cite_start]Ensure the `info.yaml` file includes the project title, author, discord username (optional), short project description, and HDL language[cite: 199].
* [cite_start]Ensure the `clock_hz` field specifies the clock frequency in Hz, or 0 if not applicable[cite: 214, 215].
* [cite_start]Define how many tiles the design occupies, noting that one tile fits around 1000 transistors[cite: 216, 217].
* [cite_start]Include the correct name of the top module and all source files in `info.yaml`[cite: 218, 219].
* [cite_start]Assign a name to each input (`ui`), output (`uo`), and bidirectional (`uio`) pin in the pinout section of `info.yaml`[cite: 224, 230, 234, 246].
* [cite_start]Update the `info.md` file inside the `/docs` folder to explain how the project works, how to use it, and any external hardware required[cite: 269].

## 5. GitHub Actions and GDS Generation
* [cite_start]Instruct the user to push the repository to automatically trigger GitHub Actions for simulation and synthesis[cite: 272, 273].
* [cite_start]If the GitHub Actions fail (e.g., test, gds, docs), guide the user to check the "Annotations" section for detailed errors[cite: 277, 330].
* [cite_start]Check for missing fields in `info.yaml` (like project title, author, or description), as this is a common reason for the build to fail[cite: 369, 370, 382].

## 6. Devpost Submission
* [cite_start]Remind the user to submit their project to Devpost[cite: 386].