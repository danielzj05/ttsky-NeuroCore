![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg) ![](../../workflows/fpga/badge.svg)

## Inspiration
Current TMS therapies are "open-loop" because doctors set a dial on the machine, but they do not actually know how the magnetic field behaves once it hits the physical tissue. To fix this, we need a way to measure the field from the inside. We envisioned NeuroCore: a battery-free, wireless ASIC that lives inside the cranium, harvesting energy from the TMS pulses to act as a real-time error controller, ensuring the tissue receives the exact magnetic dose intended.

## What it does
Monitors the Field: It uses an ADC interface (adc_data[3:0]) to capture the raw magnetic field output directly at the tissue level, reading exactly how the tissue is absorbing and distorting the TMS pulse.

Analyzes Distortion: A Haar-Lifting DWT Engine decomposes this field data into 8 frequency subbands to identify the dominant distortion bin.

Acts as an Error Controller: Instead of just reporting what it sees, the chip compares the measured dominant distortion bin against a desired target setting (target_idx[2:0]).

Auto-Adjusts Therapy: Based on that comparison, it issues a proportional 3-bit correction command (cmd_out) back to the TMS machine via LSK backscatter, telling the machine exactly how to adjust its output to hit the target.

Stays Safe: It solves the thermal issue by processing all the error-correction math locally, only transmitting tiny, sparse 14-bit command packets instead of a continuous data stream.

# NOTE
The files currently located in the /docs directory are legacy artifacts generated during the initial hackathon phase. They were not updated after the final layout and verification loop, so they may contain inaccuracies. Please treat the RTL and testbenches in the /src directory as the absolute ground truth for this project.

# Tiny Tapeout Verilog Project Template

- [Read the documentation for project](docs/info.md)

## What is Tiny Tapeout?

Tiny Tapeout is an educational project that aims to make it easier and cheaper than ever to get your digital and analog designs manufactured on a real chip.

To learn more and get started, visit https://tinytapeout.com.

## Set up your Verilog project

1. Add your Verilog files to the `src` folder.
2. Edit the [info.yaml](info.yaml) and update information about your project, paying special attention to the `source_files` and `top_module` properties. If you are upgrading an existing Tiny Tapeout project, check out our [online info.yaml migration tool](https://tinytapeout.github.io/tt-yaml-upgrade-tool/).
3. Edit [docs/info.md](docs/info.md) and add a description of your project.
4. Adapt the testbench to your design. See [test/README.md](test/README.md) for more information.

The GitHub action will automatically build the ASIC files using [LibreLane](https://www.zerotoasiccourse.com/terminology/librelane/).

## Enable GitHub actions to build the results page

- [Enabling GitHub Pages](https://tinytapeout.com/faq/#my-github-action-is-failing-on-the-pages-part)

## Resources

- [FAQ](https://tinytapeout.com/faq/)
- [Digital design lessons](https://tinytapeout.com/digital_design/)
- [Learn how semiconductors work](https://tinytapeout.com/siliwiz/)
- [Join the community](https://tinytapeout.com/discord)
- [Build your design locally](https://www.tinytapeout.com/guides/local-hardening/)

## What next?

- [Submit your design to the next shuttle](https://app.tinytapeout.com/).
- Edit [this README](README.md) and explain your design, how it works, and how to test it.
- Share your project on your social network of choice:
  - LinkedIn [#tinytapeout](https://www.linkedin.com/search/results/content/?keywords=%23tinytapeout) [@TinyTapeout](https://www.linkedin.com/company/100708654/)
  - Mastodon [#tinytapeout](https://chaos.social/tags/tinytapeout) [@matthewvenn](https://chaos.social/@matthewvenn)
  - X (formerly Twitter) [#tinytapeout](https://twitter.com/hashtag/tinytapeout) [@tinytapeout](https://twitter.com/tinytapeout)
  - Bluesky [@tinytapeout.com](https://bsky.app/profile/tinytapeout.com)
