/*

  rgb.h - shared neopixel_rgb plugin symbols

  Part of grblHAL

  Copyright (c) 2022-2023 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

*/

#pragma once

#ifdef ARDUINO
#include "../../grbl/hal.h"
#include "../../grbl/protocol.h"
#include "../../grbl/state_machine.h"
#include "../../grbl/report.h"
#include "../../grbl/nvs_buffer.h"
#else
#include "grbl/hal.h"
#include "grbl/protocol.h"
#include "grbl/state_machine.h"
#include "grbl/report.h"
#include "grbl/nvs_buffer.h"
#endif

/*No connection:orange, Idle:white, Cycle/Running:green, Jogging:green, Hold:yellow,  Door:yellow, Homing:blue, Check:blue, Alarm:red, E-stop:red, Sleep:gray, Tool Change:purple*/

#if STATUS_LIGHT_ENABLE

#include "ws2812.h"

#define RGB_OFF     0 // All RGB Off
#define RGB_RED     1 // Red
#define RGB_GREEN   2 // Green
#define RGB_BLUE    3 // Blue
#define RGB_YELLOW  4 // Red + Green
#define RGB_MAGENTA 5 // Red + Bue
#define RGB_CYAN    6 // Green + Blue
#define RGB_WHITE   7 // Red + Green + Blue
#define RGB_ORANGE  8 // Red + Green + More Red
#define RGB_GREY    9 // Red + Green + Blue but dimmer

// Blink times in ms
#define RGB_SLOW    1000
#define RGB_FAST    750
#define RGB_PULSE   500

void init_neopixels (void);
void update_neopixels (uint8_t ring_color, uint8_t rail_color);

#endif

/**/
