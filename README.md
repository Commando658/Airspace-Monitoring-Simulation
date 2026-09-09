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
shows a summary screen.

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
- `RUNWAY_GREEN_MIN_MS` and `RUNWAY_GREEN_MAX_MS`: runway speed bands.
- `ROLLOUT_TIMEOUT_MS`: maximum time allowed to reach gate 3.

## How the speed calculation works

```text
velocity 1 = gate 1-to-2 distance / gate 1-to-2 time
velocity 2 = gate 2-to-3 distance / gate 2-to-3 time
acceleration = (velocity 2 - velocity 1) / time between segment midpoints
```

Gates 1 and 2 use hardware interrupts because accurate timing matters. Their
interrupt routines do the minimum possible: they record a `micros()` timestamp
and set a flag. The main loop does the calculations afterwards.

## Important functions

- `setup()`: configures pins, OLED, servo, EEPROM, and interrupts.
- `loop()`: runs the state machine continuously without using `delay()`.
- `gate1ISR()` and `gate2ISR()`: timestamp gates 1 and 2.
- `readRangeCm()`: reads and validates the ultrasonic distance.
- `calculateMotion()`: calculates velocity and acceleration.
- `drawRadar()`: draws the scanning screen.
- `drawTelemetry()`: draws the rollout screen.
- `drawSummary()`: draws the parked summary screen.

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
arduino-cli compile \
  --fqbn arduino:avr:uno \
  "/Users/chimerekamsi/Desktop/c++ stuff/airspace_runway_monitor"
```

Compile for a classic ATmega328P Nano:

```bash
arduino-cli compile \
  --fqbn arduino:avr:nano:cpu=atmega328old \
  "/Users/chimerekamsi/Desktop/c++ stuff/airspace_runway_monitor"
```

## Uploading to the Arduino

Connect the board, find its port, and upload:

```bash
arduino-cli board list
arduino-cli upload \
  -p /dev/cu.usbserial-XXXX \
  --fqbn arduino:avr:nano:cpu=atmega328old \
  "/Users/chimerekamsi/Desktop/c++ stuff/airspace_runway_monitor"
```

Replace `/dev/cu.usbserial-XXXX` with the port shown by `arduino-cli board list`.

## Testing it visually

Use Arduino CLI compilation first to check that the code and libraries are
installed correctly. If you want to see the project working without the real
hardware, import the sketch into Wokwi and add an Arduino Uno, SSD1306 OLED,
HC-SR04, servo, LEDs, buzzer, and switches for the three IR gates.

Simulation is useful for checking display drawing and state transitions, but
real hardware testing is still required for sensor polarity, ultrasonic noise,
servo power, and physical gate timing.
