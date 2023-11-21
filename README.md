# LGDX Robot 2 MCU

XXX

## Communication

The communication is managed by STM32 Virtual Com Port, it is compatible to Serial Port programming. When initialising connection to MCU, keep all setting (Baud Rate, Data Bits, etc.) in default. To reduce overhead in communication, this project relys on raw data to communicate with MCU, below is an example. 

``` C++
int p = 1;
char ba[5];
// Command
ba[0] = 'P';
// Parameter 1
ba[1] = (p & 4278190080) >> 24;
ba[2] = (p & 16711680) >> 16;
ba[3] = (p & 65280) >> 8;
ba[4] = p & 255;
mySerial.write(ba, 5); // If you are using Qt
```

This project assumes char is 1 byte; int and float are 4 bytes.

### PC to MCU

#### Table Of Commands

| Description              | Command (char) | Parameter 1        | Parameter 2        | Parameter 3        | Parameter 4        |
|--------------------------|----------------|--------------------|--------------------|--------------------|--------------------|
| Motor Inverse Kinematics | M              | X Velocity (float) | Y Velocity (float) | w Velocity (float) |                    |
| Single Motor Velocity    | V              | Motor Number (int) | Velocity (float)   |                    |                    |
| Motor PID                | P              | Motor Number (int) | P Constant (int)   | I Constant (int)   | D Constant (int)   |

Note: Motor number starting from 0.

### MCU to PC

#### Table Of Data


| Description     | Header (char) | Value Group 1                      | Value Group 2                        | Value Group 3             |
|-----------------|---------------|------------------------------------|--------------------------------------|---------------------------|
| Wheels Velocity | A             | Target Wheels Velocity (4 * float) | Measured Wheels Velocity (4 * float) |                           |
| PID Parameters  | B             | P Constant (4 * int)               | I Constant (4 * int)                 | D Constant (4 * int)      |
| Robot Status    | C             | Motor Enabled (int)                | E-Stop Triggered (int)               | Battery Voltage (2 * int) |

## Calculation

### PWM Gwnwration

The PWM is to control the speed for every motors. The system clock is 72MHz so the PWM resolution is ~10bit and frequency is 70kHz. 

The ARR is 1027, from formula: Fpwm = (Fclk / (ARR + 1) * (PSC + 1))

### Wheel & Encoder

1 Revolution of wheel = 3960 count in stm32 counter

The gear ratio is 1:90 and encoder PPR is 11 (* 4 count in edge up/down) --> 11*4*90 = 3960

each count = 2*pi / 3960 = 0.00158666295 rad
