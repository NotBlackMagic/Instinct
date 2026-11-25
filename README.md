# Instinct Firmware
Instinct is a custom C++ firmware stack tailored for robotics and drone controllers. Written in C++ with custom bare-metal drivers, it is designed to push modern MCU architectures to their absolute limits in autonomous robotics and drone applications.

The vision is to fully exploit the capabilities of the [STM32N6](https://www.st.com/en/microcontrollers-microprocessors/stm32n6-series.html) and see how far it can be pushed in an architecture without Linux and SBCs, handling both complex motion control, localization and positioning as well as vision pipelines entirely on a single microcontroller. 

> **Project Status: 🚧 Active Development**  
> Instinct is currently in active bring-up and development on the **PlumaN6 HD (Rev. A)** prototype. Core peripheral drivers, booting, storage, and initial vision/streaming pipelines are running on hardware, while higher-level estimation, control, and autonomy modules are under heavy development. Expect frequent breaking changes.

## Target Hardware: PlumaN6 HD
Instinct is currently being developed for the custom **PlumaN6 HD** flight controller/robotics board. This board features:
- **Core:** STM32N6 (Cortex-M55 @ 800 MHz + Neural-ART NPU @ 1 GHz / 600 GOPS)
- **Memory:** 4.2 MB internal SRAM, 64 MB HyperFlash (Octal), 32 MB HyperRAM (16-bit), Micro-SD (UHS-I)
- **Vision:** 2-Lane MIPI-CSI, 8-bit Parallel DCMI, Hardware ISP, H.264 Hardware Encoder (VENC), JPEG Codec
- **Connectivity:** USB 2.0 HS (480 Mbps), 1000BASE-T1 Automotive Ethernet, CAN-FD, I3C, I2C, SPI, UART
* **Sensors:** On-board 10-DoF (IMU/Mag/Baro) + dedicated interface for an external redundant, heated IMU (`PlumaIMU`)

Full specifications and details can be found on the board project [page](https://notblackmagic.com/projects/pluman6-hd/)

## Architecture & Stack
To maximize performance and maintain full control over the STM32N6 hardware, Instinct avoids the ST HAL and relies on custom C++ bare-metal drivers built directly on registers and minimal STM32 LL definitions.

The overall system architecture is:
* **RTOS & Storage:** Eclipse ThreadX (with runtime thread CPU/stack profiling over shell) and FileX.
* **Messaging:** Custom lightweight Publish-Subscribe (PubSub) bus decoupling sensor acquisition, vision, estimation, and control threads.
* **Standalone SDK:** Although developed for PlumaN6 HD, `SDK/MCU/` and `SDK/Drivers/` are decoupled from vehicle logic and can be reused as bare-metal drivers on other custom STM32N6 boards.

## Subsystem Status (Rev. A Silicon)

| Subsystem | Status | Notes |
| :--- | :--- | :--- |
| **Boot / FSBL & Flash Loader** | Working | Custom FSBL, external flash loader, and XIP/SRAM targets |
| **HyperBus (Flash & PSRAM)** | Working | Memory-mapped HyperFlash & HyperRAM tested up to 200 MHz |
| **DMA, UART, Shell & Logging** | Working | HPDMA/GPDMA arbitrary block transfers, interactive shell, ThreadX profiler |
| **SDMMC & FileX** | Working | Tested up to 50 MHz on Rev. A (UHS-I 1.8 V planned for Rev. B) |
| **USB-HS (CDC & UVC)** | Working | Custom USB-HS driver streaming JPEG and H.264 over UVC |
| **Vision Pipeline** | Active Dev | DCMI, MIPI-CSI, DCMIPP, OV7670/OV5645, JPEG, H.264 VENC, TCM Optical Flow |
| **Sensors & PubSub** | Active Dev | IMU/Baro/Mag drivers and acquisition threads publishing over internal PubSub |
| **Estimator & Control** | In Progress | Invariant EKF state estimator, kinematic commander, and control loops |
| **Navigation & NPU** | Planned | Trajectory generation, obstacle avoidance, and Neural-ART NPU integration |

## Repository Structure

```text
Instinct/
├── Board/                  # Board-specific pin mappings and hardware initialization (PlumaN6)
├── Boot/                   # Self-flashing First Stage Bootloader (FSBL) for external memory boot
├── Loader/                 # External flash loader for STM32CubeProgrammer
├── Instinct/               # Core robotics firmware application
│   ├── Config/             # Middleware configs (tx_user.h, fx_user.h, etc.)
│   ├── Modules/            # Functional vehicle controller modules:
│   │   ├── Acquisition/    #   Sensor acquisition threads
│   │   ├── Actuation/      #   Actuator and PWM output mapping
│   │   ├── Commander/      #   System state and RC commands to kinematic setpoints
│   │   ├── Control/        #   Control loops and PID controllers
│   │   ├── Estimator/      #   Invariant EKF state estimator
│   │   ├── Math/           #   Matrix, rotation, and math utilities
│   │   ├── Messaging/      #   Custom PubSub messaging system
│   │   ├── Navigation/     #   (TBD) Trajectory generation, obstacle avoidance
│   │   ├── Payloads/       #   (TBD) Gimbal and payload interfaces
│   │   ├── System/         #   Console, shell, logger, and ThreadX profiling
│   │   └── Vision/         #   Vision stack modules for camera capture, optical flow, video encoding, image file writer, etc...
│   └── Source/             # Application entry point and startup
├── SDK/                    # Custom reusable STM32N6 bare-metal SDK
│   ├── MCU/                # Bare-metal STM32N6 peripheral drivers (CSI, DCMIPP, VENC, HyperBus, USB, etc.)
│   ├── Drivers/            # External IC and class drivers (Sensors, Video, Storage, USB, Radio)
│   └── Middleware/         # External libraries (CMSIS, STM32 LL, ThreadX, FileX)
├── cmake/                  # ARM GCC toolchain definitions
└── docs/                   # Architecture, build, and driver documentation
```

## Building
The repository uses `CMakePresets.json` with two execution targets: SRAM (for fast volatile debugging in AXISRAM2) and XIP (for booting from external HyperFlash via the custom FSBL).

```bash
# Configure and build Instinct for SRAM debugging
cmake --preset Debug-SRAM
cmake --build --preset Build-Instinct-SRAM

# Configure and build Instinct for external flash (XIP)
cmake --preset Debug-XIP
cmake --build --preset Build-Instinct-XIP

# Build the First Stage Bootloader (FSBL) or External Flash Loader
cmake --build --preset Build-Boot-Debug
cmake --build --preset Build-Loader-Debug
```

**Note on Flashing:** STM32N6 binaries booted by the BootROM must be signed with `STM32_SigningTool_CLI`. See the [STM32N6 Boot & Security notes](https://notblackmagic.com/bitsnpieces/stm32n6/#boot--security) and `.vscode/` tasks for the automated USB DFU/hot-plug debug workflow. Flashing to external flash can either be done using the `Loader` or, preferably, the self-flashing `FSBL` and then the accompanying python script `flashUART.py` to load new firmware to the core by running this command:

```bash
& py "./Boot/Source/flashUART.py" COM7 ./build/Debug-XIP/Instinct/Instinct_Signed.bin
```

For loading the `FSBL`, either use the debugger (normal way) or use USB DFU, running the below commands from the build folder of the `FSBL` (`./build/Debug-XIP/Boot/`):

```bash
C:\ST\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe -c port=usb1 -d "Boot_Signed.bin" 0x01 -v

C:\ST\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe -c port=usb1 -detach
```

## Documentation
Firmware and driver specific documentation can be found under `docs/`. Besides that, you can find documentation and implementation notes related to the STM32N6 on the [STM32N6 notes page](https://notblackmagic.com/bitsnpieces/stm32n6/), and for the target hardware, the [PlumaN6 project page](https://notblackmagic.com/projects/pluman6-hd/).

## License
This project is licensed under the **GNU General Public License v3.0** (GPL-3.0). See the [`LICENSE`](LICENSE) file for full details.