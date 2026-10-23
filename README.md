# BLDC Full-Bridge (3 Half-Bridges) Driver

Open-loop 6-step (120 deg trapezoidal) drive for a **3-wire BLDC motor** using **3 half-bridges (6 N-channel MOSFETs)**.
6 PWM outputs with phase offset + a timer + a UART.

## How it works

- Six gate inputs on your existing discrete MOSFET driver (DRV): `AH, AL, BH, BL, CH, CL`. The firmware controls every high-side and low-side signal and generates the dead-time itself.
- Each step: one phase is the **PWM phase** (high-side PWM, complementary low-side), one phase is the **return phase** (low-side ON), and one phase **floats** (both off).
- Open loop: the rotor is dragged by the commutation frequency. Too fast or too heavy a load = it loses sync (stalls or buzzes).

### Commutation table (forward)

| Step | PWM phase (high) | Return phase (low ON) | Floating |
|------|------------------|-----------------------|----------|
| 0    | A                | B                     | C        |
| 1    | A                | C                     | B        |
| 2    | B                | C                     | A        |
| 3    | B                | A                     | C        |
| 4    | C                | A                     | B        |
| 5    | C                | B                     | A        |

Reverse direction walks the table backwards. One electrical cycle = 6 steps.

### PWM and dead-time with LEDC

All 6 channels share one LEDC timer (same counter), and the low-side windows are offset with the channel `hpoint`:

```
counter:  0         D         D+dt                    P-dt   P
high:     |#########|.........................................|   ON from 0 to D
low:      |.........|.........|#####################|.........|   ON from D+dt to P-dt
                       ^dt                              ^dt
```

- `P` = timer period in ticks, `D` = duty in ticks, `dt` = dead-time in ticks.
- Timer resolution is chosen automatically: `bits = floor(log2(80 MHz / PWM_FREQ_HZ))` (limited to 8..14). At 20 kHz that is 11 bits, 1 tick is about 24 ns, so dead-time resolution is about 24 ns.
- When duty changes, the low-side and high-side updates are ordered so the two windows never overlap, even for one PWM period.
- Because the low-side also switches, the high-side bootstrap capacitor keeps charging. `MAX_DUTY_PERCENT` (default 95) leaves time for that. Before starting, firmware turns all low-sides ON for `BOOT_CHARGE_MS` to pre-charge the bootstraps.
- Do not use Arduino `ledcWrite/ledcAttach` on these pins. The sketch drives LEDC directly.

## Bill of materials (all through-hole)

| Ref   | Value / Model                      | Note                                                    |
|-------|------------------------------------|---------------------------------------------------------|
| U1    | ESP32-DevKitC (ESP32-WROOM-32)     | Any ESP32 dev board with pin headers                    |
| DRV   | Existing discrete MOSFET driver    | 6 inputs, 6x N-ch MOSFET, e.g. IRLZ44N (TO-220)         |
| C1    | 470 uF / 25 V electrolytic         | Radial, bulk cap close to DRV                           |
| C2    | 100 nF / 50 V ceramic              | Radial/disc, parallel with C1                           |
| R1-R6 | 10 kOhm, 1/4 W                     | Input pull-downs. Skip if DRV already has them          |
| M1    | BLDC motor, 3 wires (A, B, C)      | No center tap needed                                    |
| PSU   | 12 V DC, >= 3 A                    | Design target: 12 V, max 2 A per phase                  |

No flyback diodes: the MOSFET body diodes handle freewheeling in a full bridge.

## Circuit

```
        +12V ●--+----------+---------------------+
                |          |                     |
              C1 (+)     C2 100nF                |
            470uF/25V      |                     |
                |          |                     |
        GND  ●--+----------+-------+             |
                                   |             |
                          +--------+-------------+--------+
                          | GND                  VM       |
 U1 ESP32-DevKitC         |                               |         M1 BLDC (3-wire)
 +-------------+          |  DISCRETE MOSFET DRIVER (DRV) |
 | GPIO25      |--(R1)----| AH                     OUT_A  |-------- wire A
 | GPIO26      |--(R2)----| AL                            |
 | GPIO27      |--(R3)----| BH                     OUT_B  |-------- wire B
 | GPIO32      |--(R4)----| BL                            |
 | GPIO33      |--(R5)----| CH                     OUT_C  |-------- wire C
 | GPIO23      |--(R6)----| CL                            |
 |         GND |--- GND   +-------------------------------+
 +-------------+

 (R1..R6) = 10 kOhm from that signal line to GND (input pull-down, keeps all MOSFETs off during ESP32 boot)
```

Notes:
- **Common GND is mandatory** between PSU, DRV and ESP32. It is the only "feedback-like" connection needed.
- Driver and gate-supply details (bootstrap, gate resistors, etc.) follow your existing DRV design.
- All 6 inputs are assumed **active-high** (HIGH = MOSFET ON). For an inverting driver, add inverters or change the firmware.
- ESP32 is powered separately (USB or its own 5 V). Do not power it from the motor rail unless regulated.

## Files

- `README.md` this file
- `firmware.ino` Arduino sketch (Arduino-ESP32 core 2.x or 3.x, LEDC via ESP-IDF API, no libraries)

## Adjustable variables

Edit at the top of `firmware.ino` (defaults), or change live over Serial (115200 baud, newline-terminated).

| Variable              | Default | Serial  | Meaning                                              |
|-----------------------|---------|---------|------------------------------------------------------|
| `PWM_FREQ_HZ`         | 20000   | `p<Hz>` | PWM carrier, 1000..40000                             |
| `DUTY_PERCENT`        | 40      | `d<%>`  | Target duty of the PWM phase, 0..100                 |
| `MAX_DUTY_PERCENT`    | 95      | `m<%>`  | Duty ceiling (bootstrap charge time), 10..98         |
| `DEAD_TIME_NS`        | 500     | `t<ns>` | Dead-time between high and low of the same phase     |
| `ELEC_FREQ_HZ`        | 20      | `f<Hz>` | Target electrical frequency (full 6-step cycles/s)   |
| `RAMP_TIME_MS`        | 2000    | `r<ms>` | Soft-start time from start values to target          |
| `START_FREQ_HZ`       | 3       | -       | Frequency at the beginning of the ramp               |
| `START_DUTY_PERCENT`  | 20      | -       | Duty at the beginning of the ramp                    |
| `BOOT_CHARGE_MS`      | 20      | -       | Low-side ON time to pre-charge bootstrap caps        |
| `DIRECTION`           | 0       | `x0/x1` | 0 = forward, 1 = reverse                             |
| `ENABLE_ON_BOOT`      | false   | `e0/e1` | Motor stays off after reset until `e1`               |
| -                     | -       | `?`     | Print current status (incl. real dead-time in ns)    |

Mechanical speed (RPM) = `ELEC_FREQ_HZ x 60 / pole_pairs`.
Example: `d30` Enter, `f10` Enter, `e1` Enter.

Changing `p` or `t` while running briefly turns all outputs off, reconfigures the timer, then resumes.

## Porting to another MCU

Replace only `timerConfig()`, `setCh()` and the timing calls. You need 6 PWM channels on one counter with per-channel start offset (or a hardware complementary PWM with dead-time). The commutation table and ramp logic are plain C.

## First run checklist

1. Use a **current-limited bench supply** (start at 6 V, 0.5 A limit). Motor unloaded.
2. **Before connecting the motor**, scope `AH` vs `AL` at DRV: confirm both are never high together and the gap matches `DEAD_TIME_NS`. Better, scope the actual gate-to-source voltages: dead-time must exceed the driver's turn-off delay plus margin.
3. Flash, open Serial at 115200, send `d25`, `f5`, `e1`. Raise `f` slowly; raise `d` if it skips steps.
4. If it vibrates instead of turning, lower `f` or raise `d`. Wrong direction: `x1`.
5. Check MOSFET and motor temperature before running longer.

## Limitations

- Open loop: no stall detection, no current limit, no overcurrent shutdown. Use a current-limited supply.
- At high `PWM_FREQ_HZ` with high `DUTY_PERCENT`, the complementary low-side window gets very short (it can be dropped automatically when it would be shorter than 1 tick, then freewheeling goes through the body diode).
