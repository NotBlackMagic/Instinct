# Instinct Firmware
Instinct is a custom firmware stack tailored for robotics and drone controllers. Written in C++ with custom bare-metal drivers, it is designed to push modern MCU architectures to their absolute limits in autonomous robotics and drone applications.

The vision is to fully exploit the capabilities of the [STM32N6](https://www.st.com/en/microcontrollers-microprocessors/stm32n6-series.html) and see how far it can be pushed in an architecture without Linux and SBCs, handling both complex motion control, localization and positioning as well as vision pipelines entirely on a single microcontroller. 

## Target Hardware: PlumaN6 HD
Instinct is currently being developed for the custom **PlumaN6 HD** flight controller/robotics board. This board features:
- **Core:** STM32N6 (Cortex-M55 @ 800MHz + Neural Art NPU @ 1GHz)
- **Memory:** HyperBus PSRAM and NOR Flash, Micro-SD (UHS-I)
- **Vision:** 2-Lane MIPI-CSI, DCMI, Hardware ISP, H.264 Hard Encoder
- **Connectivity:** USB 2.0 HS, 1000Base-T1 Automotive Ethernet

Full specifications and details can be found on the board project [page](https://notblackmagic.com/projects/pluman6-hd/)

## Architecture & Stack
Instinct relies mostly on custom code, libraries and drivers. This to explore and learn about the miscellaneous modules and peripheral drivers.

Although the firmware is written for the **PlumaN6 HD** board, its modular design enables the use of some modules and especially drivers for your own custom **STM32N6** boards.

## Repository Structure

- **`Board/`:** Board-specific pin mappings, and hardware initialization.
- **`Boot/`:** Future location of Boot Loader (FSBL).
- **`Instinct/`:** Core firmware applications, RTOS threads, vision pipelines, and control loops.
	- **`Acquisition/`:** Sensor acquisition threads.
	- **`Control/`:** TBD
	- **`Estimator/`:** TBD
	- **`Messaging/`:** Custom PubSub messaging system.
	- **`System/`:** Basic system modules such as the console, shell and logger.
	- **`Vision/`:** Vision stack modules such as DCMI camera acquisition, image file writer, etc...
- **`Loader/`:** Flash loader for flashing a new binary to the external flash.
- **`SDK/`:** Core libraries, ThreadX/FileX sources, and custom bare-metal peripheral drivers.
	- **`Drivers/:`** Drivers for specific ICs (sensors, memories, etc...) and for higher levels of complex peripheral drivers.
	- **`MCU/:`** Custom bare-metal low-level peripheral drivers for the STM32N6.
	- **`Middleware/:`** External code and libraries (STM32 LL Drivers, CMSIS, ThreadX, FileX, etc...).

## Project Status: 🚧 Active Early Development
Instinct is currently in its very early stages. The primary target hardware, the **PlumaN6 HD** (Revision A) is fully assembled (in-house) and is currently on the bench for validation.

As a result of ongoing validation, this firmware is highly experimental. The codebase is under active development, and features, module structures, and peripheral drivers are subject to frequent breaking changes as the architecture is fleshed out.

## License
This project is licensed under the **GNU General Public License v3.0** (GPL-3.0). See the [`LICENSE`](LICENSE) file for full details.