# LGDXRobot2 MCU

LGDXRobot2 MCU is an STM32-based controller board designed specifically for the LGDXRobot2 platform. It supports a four-wheeled Mecanum chassis configuration with a straightforward, minimalistic design.

- [Homepage](https://lgdxrobot.bristolgram.uk/lgdxrobot2/)
- [Documentation](https://docs.lgdxrobot.bristolgram.uk/lgdxrobot2/mcu/)

## Notes

### PWM Gwnwration

The PWM is to control the speed for every motors. The system clock is 96MHz and PWM frequency is 10kHz. 

The ARR is 9599, from formula: Fpwm = (Fclk / (ARR + 1) * (PSC + 1))

### Wheel & Encoder

1 Revolution of wheel = 3960 counts in STM32 counter

The gear ratio is 1:90 and encoder PPR is 11 * 4 count in edge up/down = 11 * 4 * 90 = 3960

Each count = 2 * pi / 3960 = 0.00158666295 rad
