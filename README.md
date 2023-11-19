# LGDX Robot 2 MCU

XXX

## Communication

XXX (Interface)

### PC --> MCU

#### Table Of Commands

| Description              | Command (char) | Parameter 1        | Parameter 2        | Parameter 3        |
|--------------------------|----------------|--------------------|--------------------|--------------------|
| Motor Inverse Kinematics | M              | X Velocity (float) | Y Velocity (float) | w Velocity (float) |
| Motor P                  | P              | Motor Number (int) | Constant (int)     |                    |
| Motor I                  | I              | Motor Number (int) | Constant (int)     |                    |
| Motor D                  | D              | Motor Number (int) | Constant (int)     |                    |

Note: Motor number start from 0.

### MCU --> PC

XXX

## Calculation

### PWM Gwnwration

The PWM is to control the speed for every motors. The system clock is 72MHz so the PWM resolution is ~10bit and frequency is 70kHz. 

The ARR is 1027, from formula: Fpwm = (Fclk / (ARR + 1) * (PSC + 1))

### Wheel & Encoder

1 Revolution of wheel = 3960 count in stm32 counter

The gear ratio is 1:90 and encoder PPR is 11 (* 4 count in edge up/down) --> 11*4*90 = 3960

each count = 2*pi / 3960 = 0.00158666295 rad
