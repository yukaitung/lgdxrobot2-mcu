some notes:
the system clock is 72MHz so the pwm resolution is ~10bit and frequency is 70kHz
the ARR is = Fpwm = (Fclk / (ARR + 1) * (PSC + 1))

1 Revolution of wheel = 3960 count in stm32 counter
it is because the motor ratio is 1:90 and encoder PPR is 11 (* 4 count in edge up/down)
--> 11*4*90 = 3960
each count = 2*pi / 3960 = 0.00158666295 rad
