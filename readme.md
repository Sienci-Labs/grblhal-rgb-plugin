'DATRON LIKE' RGB INDICATOR LIGHTS
A Plugin for grblHAL

This plugin displays visual status by driving an onboard Neopixel on the SLB Black.  It uses simple bit-banging to generate the required waveforms and likely requires a core clock speed of at least 100 MHz.

Color states:
No connection:orange(TBD), Idle:white, Cycle/Running:green, Jogging:green, Hold:yellow,  Door:yellow, Homing:blue, Check:blue, Alarm:red, E-stop:red, Sleep:gray, Tool Change:purple
