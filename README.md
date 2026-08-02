# chromapi_motherboard

This repo contains a dedicated control HAT designed to manage the high-speed motion, power distribution, and sensor integration required for the quadruped robot Chromapi.

![Chromapi](https://custom-icon-badges.demolab.com/badge/Chromapi-motherboard-489648?logo=chromapi)
![KiCad](https://img.shields.io/badge/KiCad-10.0+-2849bf?logo=kicad)
![STM32](https://img.shields.io/badge/STM32-G431-3cb4e6?logo=stmicroelectronics)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache%202.0-lightgrey.svg)](https://opensource.org/licenses/Apache-2.0)
![Issues](https://img.shields.io/github/issues/Mowibox/chromapi_motherboard)
![Stars](https://img.shields.io/github/stars/Mowibox/chromapi_motherboard?style=social)

<p align="center">
  <img alt="chromapi_motherboard" src="./assets/chromapi_motherboard.png"/>
</p>

*Designed with KiCAD 10.*

## Table of contents

| Section                               | Description                                                               |
| ------------------------------------- | ------------------------------------------------------------------------- |
| [Project overview](#project-overview) | General description of the quadruped control board                        |
| [Authors](#authors)                   | Main contributors information                                             |
| [Documentation](#documentation)       | Links to datasheets, BOM, and design architecture                         |
| [Contributions](#contributions)       | How to contribute to the repository                                       |
| [License](#license)                   | Licensing information                                                     |

## Project overview

| Resource | Description | Location | Status |
| :--- | :--- | :--- | :--- |
| **💻 Firmware** | STM32 low-level drivers and communication protocol. | [`/firmware`](./firmware) | *Coming Soon* |
| **📐 Schematics** | PDF for hardware architecture | [`/hardware/schematic_pdf`](./hardware/schematic_pdf/chromapi_motherboard_v1.pdf) | **Available** |
| **📦 Manufacturing** | Gerbers, Drill files, BOM, and CPL for assembly. | [`/hardware/production`](./hardware/production/) | **Available** |

## Authors

| |
| :---: |
| <img src="https://github.com/Mowibox.png" width="100"> |
| [**Ousmane THIONGANE**](https://mowibox.github.io) |

## Sponsors & Acknowledgements

The hardware development of this board relies on the resources provided by the sponsors below.

### PCBWay

**[PCBWay](https://www.pcbway.com)** is a global manufacturer specializing in rapid prototyping and low-volume hardware production, offering advanced PCB fabrication, turnkey assembly, CNC machining, and 3D printing.

<p align="center">
  <a href="https://www.pcbway.com">
    <img src="./assets/pcbway.png" width="250" alt="PCBWay">
  </a>
</p>

For this project, I utilized their sponsored [PCB manufacturing](https://www.pcbway.com/orderonline.aspx) and [assembly](https://www.pcbway.fr/pcb-assembly.html) (PCBA) services. Beyond fast production, their engineering team excels at detailed file reviews by cross-checking design options, BOMs, and footprints to prevent errors early. Their reliable component sourcing and rigorous verification were of great help in building a high-quality prototype for the Chromapi's motherbard.

---

> Huge thanks to all the hardware folks who reviewed my schematics, or gave me layout tips. I really appreciate your time and support!

## Documentation

*Coming soon...*

## Contributions

Contributions are always welcome!

* **Report Issues:** Found an error or have a feature request? Create a new issue [here.](https://github.com/Mowibox/chromapi_motherboard/issues/new/choose)
* **Fix Bugs & Add Features:** Find out where you can lend a hand by checking out [existing issues.](https://github.com/Mowibox/chromapi_motherboard/issues)

## License

This project is licensed under the Apache 2.0 License. See the [LICENSE](https://github.com/Mowibox/chromapi_motherboard/blob/main/LICENSE) file for more details.
