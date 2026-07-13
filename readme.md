# RGB Status Indicator Lights
## A Plugin for grblHAL

This plugin drives Neopixel LEDs on grblHAL boards to indicate the CNC runtime state via color. Uses the grblHAL RGB HAL; the transfer method is driver-dependent (DMA on STM32F4x, PIO on RP2040/2350, RMT on ESP32, etc.).

`STATUS_LIGHT_ENABLE == 2` builds the legacy two-string implementation in `rgb.c`.

`STATUS_LIGHT_ENABLE == 3` builds the advanced single-strip implementation in `rgbv2.c`, intended for boards where LEDs `0..11` are the onboard E-stop ring and LEDs `12..end` are an offboard strip.

---

## Auto State Colors

By default, the lights change color automatically based on machine state:

| State | Color |
|---|---|
| Idle | White |
| Cycle / Running | Green |
| Jogging | Green |
| Hold | Yellow |
| Safety Door | Yellow |
| Homing | Blue |
| Check Mode | Blue |
| Alarm | Red |
| E-stop | Red |
| Tool Change | Magenta (Purple) |
| Sleep | Grey |

On program completion, the lights flash white briefly (checkered flag effect).

In `STATUS_LIGHT_ENABLE == 3`, the first 12 onboard LEDs animate for active states while the offboard LEDs stay static. Idle, Cycle / Running, and Sleep remain static to avoid unnecessary task churn and jitter.

### Onboard Ring Animations (`STATUS_LIGHT_ENABLE == 3`)

| State | Onboard Ring Animation |
|---|---|
| Idle | Solid white |
| Cycle / Running | Solid green |
| Jogging | Green comet with a short bright head and dim trailing tail |
| Hold | Full-ring yellow pulse between dim and bright |
| Safety Door | Alternating yellow half-ring blink |
| Homing | Mirrored blue sweep from opposite sides |
| Check Mode | Rotating cyan checkerboard |
| Alarm | Red chase with a fading tail |
| E-stop | Full-ring red blink |
| Tool Change | Rotating magenta dotted pattern |
| Sleep | Solid grey |

For `STATUS_LIGHT_ENABLE == 3`, LEDs `0..11` are the onboard ring and LEDs `12..end` remain a static offboard segment using the normal state color.

---

## M356 - Manual Override

Use `M356` to override the automatic color behavior per LED zone.

### Syntax

```text
M356 P<zone> Q<mode> S<brightness>
```

| Parameter | Values | Description |
|---|---|---|
| `P` | `STATUS_LIGHT_ENABLE == 2`: 0 = Rail, 1 = Ring | Legacy two-string zone mapping |
| `P` | `STATUS_LIGHT_ENABLE == 3`: 0 = Onboard, 1 = Offboard | Advanced single-strip zone mapping |
| `Q` | 0 = Auto, 1 = White, 2 = Off, 3 = Green | Override mode |
| `S` | 0-255 | Overall LED strip brightness |

### Examples

| Command | Zone |
|---|---|
| `M356 P0 Q0` | Zone 0 auto |
| `M356 P1 Q0` | Zone 1 auto |
| `M356 P0 Q1` | Zone 0 white |
| `M356 P1 Q1` | Zone 1 white |
| `M356 P0 Q2` | Zone 0 off |
| `M356 P1 Q2` | Zone 1 off |
| `M356 P0 Q3` | Zone 0 green |
| `M356 P1 Q3` | Zone 1 green |
| `M356 S64` | Set overall strip brightness to 64 |

`M356 S<value>` can be used on its own without changing the current override mode.

The override persists until changed or the controller is reset.

---

## M150 - Manual Color Control

For direct RGB color control (independent of state or overrides), the companion plugin from https://github.com/grblHAL/Plugins_misc/blob/main/rgb_led_m150.c can be used alongside this plugin. It adds `M150 R<red> G<green> B<blue>` support for setting arbitrary colors.

---

## License

CERN-OHL-S v2 - see license header in `rgb.c` and `rgbv2.c`.
