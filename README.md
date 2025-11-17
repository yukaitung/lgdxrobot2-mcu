# LGDXRobot2 MCU

LGDXRobot2 MCU is an STM32-based controller board designed specifically for the LGDXRobot2 platform. It supports a four-wheeled Mecanum chassis configuration with a straightforward, minimalistic design.

* [Homepage](https://lgdxrobot.bristolgram.uk/lgdxrobot2/)
* [Documentation](https://docs.lgdxrobot.bristolgram.uk/lgdxrobot2/)
* LGDXRobot2 Design: ([GitLab](https://gitlab.com/lgdxrobotics/lgdxrobot2-design) | [GitHub](https://github.com/yukaitung/lgdxrobot2-design))
* LGDXRobot2 MCU: ([GitLab](https://gitlab.com/lgdxrobotics/lgdxrobot2-mcu) | [GitHub](https://github.com/yukaitung/lgdxrobot2-mcu))
* LGDXRobot2 ChassisTuner: ([GitLab](https://gitlab.com/lgdxrobotics/lgdxrobot2-chassistuner) | [GitHub](https://github.com/yukaitung/lgdxrobot2-chassistuner))
* LGDXRobot2 ROS2: ([GitLab](https://gitlab.com/lgdxrobotics/lgdxrobot2-ros2) | [GitHub](https://github.com/yukaitung/lgdxrobot2-ros2))

## Prerequisites

* [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html)
* ST-LINK V2

## Flashing

1. Download the latest firmware from [here](https://gitlab.com/lgdxrobotics/lgdxrobot2-mcu/-/releases).
2. Plug the BlackPill into the ST-LINK V2, then connect the ST-LINK V2 to your computer.
3. Press the BOOT0 button on the BlackPill, then reset it to enter bootloader mode.
4. Launch STM32CubeProgrammer.
5. Click the **Connect** button on the right side of the window.
6. Select **Erasing & Programming** from the left side of the window (arrow pointing to a device icon).
7. Click **Browse** in the middle of the window and locate the firmware file (`.elf`).
8. Press **Start Programming**.
9. Wait for the firmware to be flashed.
10. Close STM32CubeProgrammer and reset the BlackPill.
11. You should hear a click sound from the relay module.

## Building

Assume that you are using Ubuntu 24.04.

1. Install the dependencies.

```bash
sudo apt install --no-install-recommends cmake ninja-build gcc-arm-none-eabi libnewlib-arm-none-eabi
```

2. Clone the project repository.

```bash
git clone https://gitlab.com/lgdxrobotics/lgdxrobot2-mcu.git
```

3. Navigate to the project directory in terminal.
4. Build the firmware.

```bash
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=./cmake/gcc-arm-none-eabi.cmake -S ./ -B ./build/Release -G Ninja
cmake --build ./build/Release
```

5. Locate the firmware file (`.elf`) in the `build/Release` directory.
6. Flash the firmware using STM32CubeProgrammer.