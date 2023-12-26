# LGDX Robot 2 MCU

LGDX Robot 2 is a DIY universal mecanum wheel chassis project. The LGDX Robot 2 MCU controls 4 wheels on the chassis.

The goal of version 2 are reducing cost and hardware dependency. This project uses [BlackPill](https://github.com/WeActStudio/WeActStudio.MiniSTM32F4x1) for MCU, TB6612FNG for motors control and INA219 for measuring voltage of batteries. It removed PCA9685 because the puepose is different in [old project](https://gitlab.com/yukaitung/LGDXRobot-MCU).

### Links

*   [LGDXRobot2-MCU](https://gitlab.com/yukaitung/lgdxrobot2-mcu)
*   [LGDXRobot2-ChassisTuner](https://gitlab.com/yukaitung/lgdxrobot2-chassistuner)

## Communication

The communication is managed by STM32 Virtual Com Port, it is compatible to Serial Port programming. When initialising connection to MCU, keep all setting (Baud Rate, Data Bits, etc.) in default. To reduce overhead in communication, this project relys on raw data to communicate with MCU, below is an example. 

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

Note1: Motor number starting from 0

Note2: 0 = Disable, 1 = Enable

### MCU to PC

A message broadcast from MCU about every 20ms

* 0xAA Pattern
* The length of the package in bytes (char), including 0xAA Pattern, data and '\0'
* Target Wheels Velocity (4 * float)
* Measured Wheels Velocity (4 * float)
* P Constant (4 * float)
* I Constant (4 * float)
* D Constant (4 * float)
* Battery Voltage (2 * int) (The chassis has 2 power source, first is moter, second is for computer)
* Software E-Stop Enabled (int) (N1)
* Hardware E-Stop Enabled (int) (N1) (Software E-Stop can triggers this because of circuit design)

Note1: 0 = Disable, 1 = Enable

## Calculation

### PWM Gwnwration

The PWM is to control the speed for every motors. The system clock is 72MHz and PWM frequency is 10kHz. 

The ARR is 71999, from formula: Fpwm = (Fclk / (ARR + 1) * (PSC + 1))

### Wheel & Encoder

1 Revolution of wheel = 3960 counts in STM32 counter

The gear ratio is 1:90 and encoder PPR is 11 * 4 count in edge up/down = 11 * 4 * 90 = 3960

Each count = 2 * pi / 3960 = 0.00158666295 rad
