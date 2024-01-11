# LGDX Robot 2 MCU

LGDX Robot 2 is a DIY universal mecanum wheel chassis project. The LGDX Robot 2 MCU controls 4 wheels on the chassis.

The goal of version 2 are reducing cost and hardware dependency. This project uses [BlackPill](https://github.com/WeActStudio/WeActStudio.MiniSTM32F4x1) for MCU, TB6612FNG for motors control and INA219 for measuring voltage of batteries. It removed PCA9685 because the puepose is different in [old project](https://gitlab.com/yukaitung/LGDXRobot-MCU).

Refer [doc/Chassis2.kicad_sch](doc/Chassis2.kicad_sch) for schematic design.

### Links

*   [LGDXRobot2-MCU](https://gitlab.com/yukaitung/lgdxrobot2-mcu)
*   [LGDXRobot2-ChassisTuner](https://gitlab.com/yukaitung/lgdxrobot2-chassistuner)

# How it works

The firmware is developed with STM32 HAL Library. It utilises Timer for encoders (TIM1, TIM3, TIM4, TIM5) and PWM generation (TIM2), I2C for communication with INA219 (I2C1), as well as Virtual Port Com for communication with PC.

### Hardware Communication

The communication is managed by STM32 Virtual Com Port, it is compatible to Serial Port programming. When initialising connection to MCU, keep all setting (Baud Rate, Data Bits, etc.) to default. To reduce overhead in communication, this project relys on raw data to communicate with MCU, below is an example to send command from PC. 

``` C++
int p = 1;
char ba[9];
// Command
ba[0] = 'V';
// Parameter 1
ba[1] = (p & 4278190080) >> 24;
ba[2] = (p & 16711680) >> 16;
ba[3] = (p & 65280) >> 8;
ba[4] = p & 255;
// Parameter 2
...
mySerial.write(ba, 9); // If you are using Qt
```

This project assumes char is 1 byte; int and float are 4 bytes.

### PC to MCU

#### Table Of Commands

| Description              | Command (char) | Parameter 1             | Parameter 2        | Parameter 3        | Parameter 4        |
|--------------------------|----------------|-------------------------|--------------------|--------------------|--------------------|
| Motor Inverse Kinematics | M              | X Velocity (float)      | Y Velocity (float) | w Velocity (float) |                    |
| Single Motor Velocity    | V              | Motor Number (int) (N1) | Velocity (float)   |                    |                    |
| Motor PID                | P              | Motor Number (int) (N1) | P Constant (float) | I Constant (float) | D Constant (float) |
| Software E-Stop          | E              | Enable (int) (N2)       |                    |                    |                    |
| Reset Odometry (Pending) | O              |                         |                    |                    |                    |

Note1: Motor number starting from 0

Note2: 0 = Disable, 1 = Enable

### MCU to PC

A message broadcast from MCU about every 20ms, below is the sequence of the message.

| Description              | Member                                          | Size (bytes) |
|--------------------------|-------------------------------------------------|--------------|
| 0xAA Pattern             | OxAA (char)                                     | 1            |
| Frame Total Size         | Size (char)                                     | 1            |
| Transform                | x, y, w (3 float)                               | 12           |
| Forward Kinematic        | x velocity, y velocity, w velocity (3 float)    | 12           |
| Target Wheels Velocity   | wheel1, wheel2, wheel3, wheel4 (4 float)        | 16           |
| Measured Wheels Velocity | wheel1, wheel2, wheel3, wheel4 (4 float)        | 16           |
| P Constant               | wheel1, wheel2, wheel3, wheel4 (4 float)        | 16           |
| I Constant               | wheel1, wheel2, wheel3, wheel4 (4 float)        | 16           |
| D Constant               | wheel1, wheel2, wheel3, wheel4 (4 float)        | 16           |
| Battery Voltage          | Battery 1, Battery 2 (N1) (unsigned short)      | 4            |
| E-Stop Enabled           | MSB Software bit (N2), Hardware bit (N3) (char) | 1            |
|                          | Total                                           | 111          |

Note1: he chassis has 2 power source, Battery 1 is moter, Battery 2 is for computer

Note2: 0 = Disable, 1 = Enable

Note3: Software E-Stop can triggers this because of circuit design

# Getting started

### Prerequisite

This project required a mecanum wheel chassis and control board (Refer [doc/Chassis2.kicad_sch](doc/Chassis2.kicad_sch) for schematic design).

In the schematic, R1 is 100Ω, R2 and R3 are 4.7KΩ. Choose the motor with gear ratio no lower than 1:90 to ensure enough torque.

Requeired software are [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html), [STSW-LINK009](https://www.st.com/en/development-tools/stsw-link009.html) ([Press here for ARM64 PC](https://community.st.com/t5/stm32-mcus-boards-and-hardware/stlink-stcubeprogrammer-support-on-windows-arm64/td-p/224127)) and an ARM IDE. This project uses [MDK-Community Edition](https://www2.keil.com/mdk5/editions/community).

A ST-LINK/V2 is required to download program.

### Build & Run

1. Launch STM32CubeMX to generate code and launch ARM IDE
2. Download program in the IDE
3. Test with [LGDXRobot2-ChassisTuner](https://gitlab.com/yukaitung/lgdxrobot2-chassistuner)

# Calculation

This section explains the decision for some values.

### PWM Gwnwration

The PWM is to control the speed for every motors. The system clock is 92MHz and PWM frequency is 1kHz. 

The ARR is 95999, from formula: Fpwm = (Fclk / (ARR + 1) * (PSC + 1))

### Wheel & Encoder

1 Revolution of wheel = 3960 counts in STM32 counter

The gear ratio is 1:90 and encoder PPR is 11 * 4 count in edge up/down = 11 * 4 * 90 = 3960

Each count = 2 * pi / 3960 = 0.00158666295 rad
