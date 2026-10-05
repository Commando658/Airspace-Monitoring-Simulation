# Airspace Runway Monitor

This is the Arduino code for my model aircraft monitoring system. It uses a
servo-mounted ultrasonic sensor to scan the airspace, then uses three IR gates
to measure how quickly the model plane travels along the runway.

The main program is in `airspace_runway_monitor.ino`.

## How it works

When the system starts, the servo sweeps the ultrasonic sensor from side to
side. The OLED shows a simple radar screen and the LEDs/buzzer report what the
sensor sees.

When the model plane passes the first IR gate, the system switches to rollout
mode. Gates 1 and 2 measure the first speed, and gates 2 and 3 measure the
second speed. Once gate 3 is reached, the program calculates acceleration and
shows a summary screen, which also carries a running count of completed runs
held in EEPROM.

The system then resets and starts scanning again. If the plane never reaches
gate 3, the system gives up after 8 seconds and resets automatically.

## Hardware map

| Pin | Function |
|---|---|
| D2 | IR gate 1 / touchdown, external interrupt |
| D3 | IR gate 2 / speed gate, external interrupt |
| D4 | IR gate 3 / parking bay |
| D5 | Active buzzer |
| D6 | HC-SR04 trigger |
| D7 | HC-SR04 echo |
| D8 | Servo signal |
| D9 | Airspace yellow LED |
| D10 | Airspace green LED |
| D11 | Airspace red LED |
| D12 | Runway yellow LED |
| A0 | Runway green LED |
| A1 | Runway red LED |
| A4/A5 | OLED SDA/SCL |

The OLED uses I2C address `0x3C`.

Power the SG90 servo from a separate regulated 5 V supply and connect that
supply's ground to Arduino GND. Do not power the servo from an Arduino I/O pin.

## Values you will probably need to adjust

At the top of the `.ino` file:

- `GATE_1_TO_2_CM`: distance between gates 1 and 2.
- `GATE_2_TO_3_CM`: distance between gates 2 and 3.
- `AIR_GREEN_MIN_CM` and `AIR_GREEN_MAX_CM`: airspace zones.
- `RUNWAY_GREEN_MIN_MS` and `RUNWAY_GREEN_MAX_MS`: runway speed bands. The
  defaults (0.15-0.40 m/s) are slower than walking pace, so measure a real
  rollout before trusting the green/red split.
- `ROLLOUT_TIMEOUT_MS`: maximum time allowed to reach gate 3.
- `RADAR_STEP_DEG`: servo sweep step. At 3 deg a full pass takes ~4.2 s.
- `RADAR_DISPLAY_MAX_CM`: range mapped to the outermost radar arc.

## How the speed calculation works

```text
velocity 1 = gate 1-to-2 distance / gate 1-to-2 time
velocity 2 = gate 2-to-3 distance / gate 2-to-3 time
acceleration = (velocity 2 - velocity 1) / time between segment midpoints
```

Gates 1 and 2 use hardware interrupts because accurate timing matters. Their
interrupt routines do the minimum possible: they record a `micros()` timestamp
and set a flag. The main loop does the calculations afterwards.

The maths never reads those volatile timestamps directly. When an edge is
accepted the loop copies it into `gate1AcceptedUs` / `gate2AcceptedUs` and works
from the copy, because the ISR keeps overwriting the volatile value on every
later beam break -- a wing, a tail or a vibration after gate 1 would otherwise
shift the measurement baseline.

## Important functions

- `setup()`: configures pins, OLED, servo, EEPROM, and interrupts.
- `loop()`: runs the state machine continuously without using `delay()`.
- `gate1ISR()` and `gate2ISR()`: timestamp gates 1 and 2.
- `intervalWithin()`: validates a gate interval as an elapsed duration so that
  `micros()` wrapping cannot discard a run.
- `readRangeCm()`: reads and validates the ultrasonic distance.
- `segmentVelocity()` / `bandForSpeed()`: shared distance-over-time and
  speed-to-LED-band helpers.
- `calculateMotion()`: calculates velocity and acceleration.
- `drawRadar()`: draws the scanning screen.
- `drawTelemetry()`: draws the rollout screen.
- `drawSummary()`: draws the parked summary screen.

## Timing constraints worth knowing before you change anything

**`micros()` wraps every ~71.6 minutes.** Unsigned subtraction survives the
wrap; comparing two raw timestamps does not. Every interval goes through
`intervalWithin()` for that reason. Do not reintroduce `if (gate2Us > gate1Us)`
style checks -- they silently throw away any pass that straddles the wrap.

**Gate 3 is polled, not interrupt driven.** D2 and D3 are the only external
interrupt pins on an ATmega328P and both are taken, so the main loop must never
block while a rollout is in progress. Pushing the 1 KB framebuffer over I2C at
400 kHz takes roughly 23 ms, and any of that latency would be charged straight
to segment 2. The telemetry screen is therefore redrawn only when its contents
actually change, and gate 3 is polled on every loop iteration.

If you want gate 3 to be as accurate as gates 1 and 2, move it to a pin-change
interrupt (`PCINT2_vect` on PORTD) rather than tightening the poll.

**One EEPROM write per completed run** now that runs are counted. At roughly
100,000 write cycles per cell that is ample, but it is no longer write-free.

## Known limitations

- The header calls D5 an active buzzer while the code drives it with `tone()`,
  which suits a passive one. An active buzzer will sound quieter and raspier
  than intended; switch to `digitalWrite()` if yours has its own oscillator.
- `tone()` runs a 4400 Hz interrupt that inflates `pulseIn()` slightly, so
  ranges read about 1% long while the intrusion alarm is sounding.
- A target closer than 2 cm cannot be ranged by an HC-SR04. It is reported as
  `RANGE_TOO_CLOSE` and escalated to an intrusion rather than shown as clear.

## Libraries needed

Install these libraries with Arduino CLI:

```bash
arduino-cli lib install Servo
arduino-cli lib install "Adafruit SSD1306"
arduino-cli lib install "Adafruit GFX Library"
```

`Wire` and `EEPROM` are included with the Arduino AVR core.

## Checking the code without a board

The Arduino CLI sketch folder must contain a matching `.ino` file:

```text
airspace_runway_monitor/
  airspace_runway_monitor.ino
  README.md
```

Compile for an Uno:

```bash
arduino-cli compile --fqbn arduino:avr:uno .
```

Compile for a classic ATmega328P Nano:

```bash
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328old .
```

## Uploading to the Arduino

Connect the board, find its port, and upload:

```bash
arduino-cli board list
arduino-cli upload -p /dev/cu.usbserial-XXXX \
  --fqbn arduino:avr:nano:cpu=atmega328old .
```

Run both from inside the sketch folder. Replace `/dev/cu.usbserial-XXXX` with
the port shown by `arduino-cli board list`.

### On Apple Silicon

The AVR toolchain shipped with the Arduino AVR core is x86-only, so
`arduino-cli compile` fails with `bad CPU type in executable` unless Rosetta is
installed:

```bash
softwareupdate --install-rosetta --agree-to-license
```

## Testing it visually

Use Arduino CLI compilation first to check that the code and libraries are
installed correctly. If you want to see the project working without the real
hardware, import the sketch into Wokwi and add an Arduino Uno, SSD1306 OLED,
HC-SR04, servo, LEDs, buzzer, and switches for the three IR gates.

Simulation is useful for checking display drawing and state transitions, but
real hardware testing is still required for sensor polarity, ultrasonic noise,
servo power, and physical gate timing.
