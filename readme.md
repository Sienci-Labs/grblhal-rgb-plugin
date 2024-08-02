'DATRON LIKE' RGB INDICATOR LIGHTS
A Plugin for grblHAL

This plugin displays visual status by driving an onboard Neopixel on the SLB Black.  It uses simple bit-banging to generate the required waveforms and likely requires a core clock speed of at least 100 MHz.

Color states:
No connection:orange(TBD), Idle:white, Cycle/Running:green, Jogging:green, Hold:yellow,  Door:yellow, Homing:blue, Check:blue, Alarm:red, E-stop:red, Sleep:gray, Tool Change:purple

# License
This code has been created within the context of Open Hardware and is licensed under the "CERN-OHL-S v2". Please read more about this in the LICENSE file. You could redistribute and modify this source and make products using it under the terms of the CERN-OHL-S v2. This means stuff like licensing your changes reciprocally, giving attribution to us, and releasing hardware that this code runs on as CERN-OHL-S v2 too. Note that this LICENSE applies on all coding work done in prior commits to-date, and is now being released in good faith that no action will be taken to pursue past work in order to duplicate the work we've put in without having to abide by the License that's now being applied at its first public release.
