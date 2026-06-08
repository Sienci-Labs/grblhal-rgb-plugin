# RGB Status Indicator Lights
## A Plugin for grblHAL

This plugin drives Neopixel LEDs on the SLB Black (and compatible boards) to indicate the CNC's runtime state via color. Uses the grblHAL RGB HAL — the transfer method is driver-dependant (DMA on STM32F4x, PIO on RP2040/2350, RMT on ESP32, etc.).

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

---

## M356 — Manual Override

Use `M356` to override the automatic color behavior for the rail (strip 0) or ring (strip 1) independently.

### Syntax

```
M356 P<strip> Q<mode>
```

| Parameter | Values | Description |
|---|---|---|
| `P` | 0 = Rail, 1 = Ring | Selects which LED strip to control |
| `Q` | 0 = Auto, 1 = White, 2 = Off, 3 = Green | Override mode |

### Examples

| Command | Rail | Ring |
|---|---|---|
| `M356 P0 Q0` | Auto | _(unchanged)_ |
| `M356 P1 Q0` | _(unchanged)_ | Auto |
| `M356 P0 Q1` | White | _(unchanged)_ |
| `M356 P1 Q1` | _(unchanged)_ | White |
| `M356 P0 Q2` | Off | _(unchanged)_ |
| `M356 P1 Q2` | _(unchanged)_ | Off |
| `M356 P0 Q3` | Green | _(unchanged)_ |
| `M356 P1 Q3` | _(unchanged)_ | Green |

To set both strips at once, send two commands. For example, to force both to white:

```
M356 P0 Q1
M356 P1 Q1
```

The override persists until changed or the controller is reset.

---

## M150 — Manual Color Control

For direct RGB color control (independent of state or overrides), the companion plugin from https://github.com/grblHAL/Plugins_misc/blob/main/rgb_led_m150.c can be used alongside this plugin. It adds `M150 R<red> G<green> B<blue>` support for setting arbitrary colors.

---

## License

CERN-OHL-S v2 — see license header in `rgb.c`.
