# Airspace & Runway Monitor

Arduino firmware for a model aircraft monitoring rig. A servo swings an
ultrasonic sensor back and forth to watch the airspace, and three more
ultrasonic sensors spaced along the runway time the aircraft as it rolls out, so
the system can work out how fast it was going and whether it was speeding up or
slowing down.

Everything shows on a small OLED, with coloured LEDs and a buzzer for things you
need to notice without looking at the screen.

The gates used to be IR beam-break sensors. They are ultrasonic now, which
changes how the timing works — there is a section further down explaining what
that costs you, because it is worth knowing before you trust the numbers.

---

## What you need

- Arduino Uno or Nano (ATmega328P)
- 4 × HC-SR04 ultrasonic sensors — three for the runway gates, one for the
  airspace radar
- 1 × SG90 servo (swings the radar sensor)
- 1 × SSD1306 128×64 OLED, I²C
- 6 × LEDs — yellow, green and red for airspace; yellow, green and red for the
  runway — each with its own resistor (220 Ω is fine)
- 1 × buzzer
- A separate 5 V supply for the servo

**The servo needs its own power.** Do not run it from an Arduino pin. It pulls
far more current than a pin can give, and the voltage dips will reset the board
or scramble the display mid-run. Give it a regulated 5 V supply and join that
supply's ground to the Arduino's ground.

---

## Wiring

These are the defaults in the sketch. If your board is wired differently you do
not have to rewire anything — just change the pin numbers in the sketch, which
is covered in the next section.

| Pin | Goes to | Pin | Goes to |
|---|---|---|---|
| D2 | Gate 1 TRIG | D9 | Airspace yellow LED |
| D3 | Gate 1 ECHO | D10 | Airspace green LED |
| D4 | Gate 2 TRIG | D11 | Airspace red LED |
| D13 | Gate 2 ECHO | D12 | Runway yellow LED |
| A2 | Gate 3 TRIG | A0 | Runway green LED |
| A3 | Gate 3 ECHO | A1 | Runway red LED |
| D6 | Radar TRIG | D5 | Buzzer |
| D7 | Radar ECHO | A4 / A5 | OLED SDA / SCL |
| D8 | Servo signal | | |

The OLED sits at I²C address `0x3C`. Most of these little panels do; if yours
doesn't, change `OLED_ADDRESS`.

Gate 1 is where the aircraft touches down, gate 2 sits a little further along,
and gate 3 is at the parking bay at the end.

**Every sensor needs its own trigger pin.** It is tempting to wire all the
triggers together to save pins, but then they all fire at once and each one
hears the others' chirps instead of its own echo. The readings come back as
nonsense. Keep them separate.

With serial telemetry switched on, D0 and D1 are the USB connection, which means
every single pin on the chip is doing something. If you need one back, turn off
`ENABLE_TELEMETRY`, or `HAVE_AIRSPACE_RADAR` if you are not fitting the servo
and its sensor.

---

## Getting it running

### 1. Install the libraries

```bash
arduino-cli lib install Servo
arduino-cli lib install "Adafruit SSD1306"
arduino-cli lib install "Adafruit GFX Library"
```

`Wire` and `EEPROM` come with the Arduino AVR core, so there is nothing to do
for those.

### 2. Find out which pins your sensors are really on

There is a separate sketch in `tools/wiring_check/` that works this out for you.
Upload it on its own, open the Serial Monitor at 115200, and put something solid
about 10–20 cm in front of each sensor so there is a clear echo to find.

It tries every sensible pair of pins in turn — sending a pulse out of one and
listening on the other — and prints the pairs that answered:

```
TRIG  ECHO  DISTANCE
--------------------
D2    D3    14.2 cm
D4    D13   13.8 cm
A2    A3    15.1 cm
D6    D7    22.4 cm
--------------------
Sensors found: 4
```

It takes a couple of minutes, because it has to wait for each echo to die away
before testing the next pair. It takes several readings of every pair and only
believes one where the readings agree with each other, so a floating pin picking
up noise doesn't get mistaken for a sensor.

If it finds nothing, check 5 V and ground on the sensors first, and make sure
something is actually in front of them.

### 3. Put your numbers in the sketch

Open `airspace_runway_monitor.ino` and find the block marked
`CONFIGURATION`. Everything you need to change is in there and nothing outside
it should need touching.

Set the pins to whatever the wiring check found:

```c
const uint8_t PIN_GATE1_TRIG = 2;
const uint8_t PIN_GATE1_ECHO = 3;
const uint8_t PIN_GATE2_TRIG = 4;
const uint8_t PIN_GATE2_ECHO = 13;
const uint8_t PIN_GATE3_TRIG = A2;
const uint8_t PIN_GATE3_ECHO = A3;
```

Then **measure the actual gaps between your gates with a ruler** and put the
real numbers in:

```c
const float GATE_1_TO_2_CM = 10.0f;
const float GATE_2_TO_3_CM = 15.0f;
```

This matters more than anything else in the file. Every speed is a distance
divided by a time, so if these two numbers are wrong then every reading the
system gives you is wrong by exactly the same proportion — and it will look
perfectly plausible while being wrong. Nothing will warn you.

If you are not fitting the servo and the radar sensor, set
`HAVE_AIRSPACE_RADAR` to `0`. The servo, the radar screen and the three airspace
LEDs all drop out together and the rest carries on working.

### 4. Upload it

```bash
arduino-cli board list
arduino-cli upload -p /dev/cu.usbserial-XXXX \
  --fqbn arduino:avr:nano:cpu=atmega328old .
```

Run that from inside the sketch folder, and swap in whichever port
`board list` showed you. For an Uno the board name is `arduino:avr:uno`.

To check it compiles without a board plugged in:

```bash
arduino-cli compile --fqbn arduino:avr:uno .
```

**On an Apple Silicon Mac** this fails with `bad CPU type in executable`,
because the AVR compiler that ships with the Arduino core is Intel-only. Install
Rosetta once and it works:

```bash
softwareupdate --install-rosetta --agree-to-license
```

### 5. Roll a plane down it and check the numbers

With telemetry on you get a line per run over serial at 115200:

```
run,seg1_us,seg2_us,v1_ms,v2_ms,accel_ms2
1,248300,301900,0.403,0.497,0.340
```

Do a few passes. Time one with a stopwatch and see if it roughly agrees. Then
set the speed bands from what you actually measured — the defaults are
placeholders and almost certainly wrong for your aircraft:

```c
const float RUNWAY_GREEN_MIN_MS = 0.15f;
const float RUNWAY_GREEN_MAX_MS = 0.40f;
```

0.15 to 0.40 m/s is slower than walking pace. A real model aircraft rolling out
will likely be several times that.

---

## How it works

The system is always in one of three modes, and moves between them on its own.

**Scanning.** The servo sweeps the radar sensor from one side to the other while
the OLED draws a little radar screen with a sweeping line and a blip wherever
something is. Meanwhile gate 1 is being watched for an arriving aircraft.

**Rolling out.** The moment gate 1 sees something, a timer starts and the system
switches to watching gate 2. When gate 2 is reached it has a distance and a
time, so it can work out the first speed, and the runway LEDs start reporting
whether that speed is sensible. Then it watches gate 3.

**Parked.** Gate 3 is the end of the run. Now it has both speeds and can work
out the acceleration, so it puts a summary on screen for four seconds, saves any
new records, prints the telemetry line, and goes back to scanning.

If the aircraft never reaches gate 3 — it stops short, or goes off the side —
the system gives up after eight seconds and goes back to scanning by itself.
Nothing needs resetting.

---

## Every feature, and how it works

### Airspace scanning

The servo steps the sensor across in 3° increments, pausing at each step long
enough to take a reading. A full sweep takes about 4.2 seconds.

What it finds puts the airspace into one of three zones, each with its own LED
and its own buzzer pattern so you can tell them apart without looking:

| Distance | Zone | LED | Buzzer |
|---|---|---|---|
| Under 10 cm | Intrusion | Red | Continuous tone |
| 10 – 25 cm | Optimal | Green | One short beep on entry |
| Over 25 cm | Approach | Yellow | Slow pulse |
| Nothing found | Clear | All off | Silent |

The beep only fires when the zone *changes*, not continuously, so it tells you
something happened rather than nagging.

Anything closer than 2 cm is a special case. An HC-SR04 physically cannot
measure that close, so it reports "too close to tell" rather than a distance —
and the system treats that as an intrusion, not as empty airspace. It would be a
bad trade to have the nearest possible object be the one thing that reads as
all-clear.

### How a gate decides the aircraft has arrived

Each gate is pinged in turn and converts its echo to a distance. If that
distance drops below `GATE_TRIGGER_CM` (20 cm by default), the gate counts the
aircraft as having arrived.

The catch is that an aeroplane is not a single point. The fuselage sits in front
of the sensor for a while, and a naive check would count it over and over as
each part goes past. So once a gate has counted something, it stops counting
until the distance climbs back past `GATE_RELEASE_CM` (30 cm) — in other words,
until the aircraft has actually gone.

The gap between those two numbers is deliberate. If a gate armed and fired at
the same distance, a reading wobbling by a millimetre either side of the line
would chatter on and off. Having to cross back over a *different, further* line
makes that impossible.

There is one more wrinkle. If the aircraft is parked in front of gate 3 when a
run ends, that gate must not count it again the instant the next run starts. So
at the end of every run each gate is read once, and any gate that still has
something in front of it stays switched off until it is clear.

### Only one sensor listens at a time

This is the most important thing about the gate design.

Ultrasonic sensors sitting near each other hear each other's chirps. Fire them
together and each one measures whatever it happened to pick up, which is not
necessarily its own echo. So they have to take turns.

But taking turns across all three would mean each gate only gets looked at every
third cycle, roughly 3.5 ms. During a rollout only one gate can possibly be next
— if the aircraft has passed gate 1, the only thing that matters is gate 2 — so
the system polls *only that one*. It gets looked at every 1.2 ms instead, which
is about three times more accurate, and nothing else is transmitting to
interfere with it.

| What's happening | Which sensor is listening |
|---|---|
| Scanning | Gate 1, taking turns with the radar |
| Rolling out, waiting for gate 2 | Gate 2, alone |
| Rolling out, waiting for gate 3 | Gate 3, alone |
| Parked, showing the summary | None |

### Speed and acceleration

Two speeds, each one a distance over a time:

```
speed 1 = gate 1-to-2 distance / time between gates 1 and 2
speed 2 = gate 2-to-3 distance / time between gates 2 and 3
```

Acceleration is where it gets slightly more interesting. The obvious thing is to
subtract the two speeds and divide by the time between the gates, but that is
wrong, and it's a mistake worth explaining.

Each speed is an *average* over its stretch of runway. An average doesn't belong
at either end of the stretch — it belongs in the middle. So the time between
those two speeds is the time between the two *midpoints*, not between the gates:

```
acceleration = (speed 2 - speed 1) / (half of segment 1 + half of segment 2)
```

Get that wrong and your acceleration is off by a factor that depends on how
unevenly your gates are spaced, which makes it look almost right and therefore
quite hard to spot.

### Runway LEDs

Once there is a speed, the runway LEDs report it:

| Speed | Meaning | LED |
|---|---|---|
| Below `RUNWAY_GREEN_MIN_MS` | Taxiing | Yellow |
| Within the band | Safe rollout | Green |
| Above `RUNWAY_GREEN_MAX_MS` | Overshoot | Red, flashing |

When the aircraft parks, the LED shows the band it actually achieved. If it
overshot, the light stays red next to a summary screen showing the overshoot —
earlier versions lit green at the end regardless, which contradicted the numbers
printed right beside it.

### The three screens

**Radar**, while scanning: sweep angle, current distance, a sweeping line and a
blip. The three arcs are range rings, and the blip is scaled so the outermost
arc is `RADAR_DISPLAY_MAX_CM` (60 cm). That scaling matters — an earlier version
scaled against the sensor's full 2 m range, which squeezed everything
interesting into about three pixels near the centre.

**Rollout**, during a run: both speeds, acceleration, and a one-line verdict.

This screen deliberately does *not* refresh on a timer. Pushing the full display
over I²C takes around 23 ms, and gate 3 is being polled — so a refresh landing
at the wrong moment would be charged straight to the measurement. It only
redraws when something has actually changed.

**Summary**, when parked: the run number, both speeds, the acceleration, and
your best-ever speed and acceleration.

### Records that survive a power cut

The best speed, the best acceleration and the total number of completed runs are
kept in EEPROM, so they are still there next time you switch on.

The stored data carries a marker so the firmware can tell whether what it read
is really its own. On a fresh board, or after the record format changes, that
marker won't match, and it quietly starts the records from zero rather than
showing you garbage.

Each completed run writes to EEPROM, which is good for about 100,000 writes.
That is plenty, but it is worth knowing it is not free.

### Serial telemetry

One CSV line per run at 115200, as shown earlier. This is how you check the
system against reality and set your speed bands from measurements rather than
guesses.

It costs about 175 bytes of RAM for the serial buffers. The chip only has 2 KB
and the display is already using half of it, so once everything is tuned you can
set `ENABLE_TELEMETRY` to `0` and get that back.

### It keeps working when things break

If the OLED is missing, unplugged, or at a different address, the firmware
notices at startup and carries on without it. The LEDs and the buzzer are the
outputs that actually matter for safety, and they stay live. Losing a screen
should not take the whole monitor down with it.

### Clock rollover

`micros()` counts microseconds and runs out of room after about 71.6 minutes,
wrapping back to zero. If a run happens to straddle that moment, naive code
sees the second gate as having happened *before* the first, decides the reading
is impossible, and throws the whole run away.

Subtracting two timestamps gives the right answer even across the wrap. It is
*comparing* them that breaks. So every interval in the firmware is checked as a
duration — "how long was that?" — and never by asking which timestamp is
larger. A run across the rollover gets measured like any other.

---

## What ultrasonic gates cost you

Worth being straight about this.

IR beam-break gates gave an electrical edge the instant the beam broke, which
could be timestamped to the microsecond. An ultrasonic sensor gives you no such
thing — you have to ask it, wait for the answer, and the aircraft is only seen
as often as you can ask.

That works out to roughly 1.2 ms of uncertainty on each gate crossing. What that
means in practice depends entirely on how fast your aircraft is moving:

| Time between gates | Error it introduces |
|---|---|
| 250 ms (slow taxi) | about 0.5% — fine |
| 100 ms | about 1.2% — fine |
| 50 ms | about 2.4% — noticeable |
| 20 ms (fast) | about 6% — and acceleration worse still |

Acceleration suffers more than speed does, because it is the difference between
two speeds that each already carry their own error.

So: **if your runs come out above about 2 m/s, treat the acceleration figure
with suspicion.** The honest options at that point are to space the gates
further apart, which gives the sensors more time to work with, or to measure the
echo with a pin-change interrupt instead of waiting for it, or to go back to IR
beam-break for the gates specifically, which is simply the better sensor for
timing something fast.

The interrupt approach is deliberately not implemented here. It needs
register-level code that could not be compiled or tested on the machine this was
written on, and shipping untested interrupt code to someone else's board is
worse than a documented limitation.

---

## Testing without any hardware

The firmware can be run on a computer, against stub Arduino headers and a
simulated clock. No board, no Arduino toolchain, nothing but a C++ compiler:

```bash
./test/run.sh
```

72 checks across two suites, covering the timing arithmetic, the gate detection
and hysteresis, the state machine, the airspace zones, the timeout, the EEPROM
records and the LED and buzzer logic. It also deliberately fakes a `micros()`
rollover, which on real hardware would mean sitting around for 71.6 minutes
waiting for one.

It proves nothing at all about sensor wiring, acoustic noise, servo power or
whether your gates are where you think they are. Bench testing is still
required. See [test/README.md](test/README.md) for how the simulation works and
how to add a test.

---

## If something isn't working

**Every run times out after 8 seconds.** Gate 1 is not seeing the aircraft. Run
the wiring check and confirm the pins. Then check `GATE_TRIGGER_CM` is
comfortably larger than the real gap between your sensor and the aircraft as it
passes.

**Speeds look wrong but consistent.** `GATE_1_TO_2_CM` and `GATE_2_TO_3_CM`
don't match the real spacing. Measure again.

**Readings jump around.** Sensors are probably hearing each other. Space them
further apart, angle them away from one another, or put something absorbent
between them. Also confirm the triggers are on separate pins and not wired
together.

**Display blank but LEDs work.** That is the intended behaviour with no display
detected. Check the I²C wiring and the `0x3C` address.

**Board resets mid-run, or the display scrambles.** The servo is being powered
from the Arduino. Give it its own supply.

**A gate fires several times for one aircraft.** Widen the gap between
`GATE_TRIGGER_CM` and `GATE_RELEASE_CM`.
