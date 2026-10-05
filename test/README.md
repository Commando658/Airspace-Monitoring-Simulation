# Host-side tests

These run the real `airspace_runway_monitor.ino` on your computer, against stub
Arduino headers and a simulated clock. No board, no Arduino toolchain and no AVR
cross-compiler are needed -- just a C++17 compiler.

```bash
./test/run.sh
```

## What this does and does not prove

It covers the parts that are genuinely hard to check on a bench: timing
arithmetic, state transitions, and LED/buzzer decisions. A `micros()` wrap
happens once every 71.6 minutes of uptime, and reproducing one deliberately on
hardware means waiting for it.

It proves nothing about sensor polarity, ultrasonic noise, servo power, I2C
wiring or physical gate alignment. **Real hardware testing is still required.**

## Layout

| Path | Purpose |
|---|---|
| `stubs/` | Stand-ins for the Arduino core, Wire, Servo, EEPROM and the Adafruit display |
| `harness.h` | Simulated clock, pins and display, plus helpers for driving a pass |
| `test_regressions.cpp` | One test per defect that was once present, written so reintroducing it fails |
| `test_behaviour.cpp` | State machine, airspace zones, timeout, display and EEPROM behaviour |
| `run.sh` | Builds and runs both suites |

## How the simulation works

`sim.nowUs` is the master clock and nothing advances it but the test. `step()`
moves it forward and runs one `loop()` iteration, so a whole pass can be played
out deterministically at microsecond resolution.

`micros()` and `millis()` return `uint32_t`, exactly as on AVR. That matters:
the timestamps in the firmware are `uint32_t` too, so parking the clock just
below `0x100000000` makes a pass genuinely straddle the wrap instead of hiding
it behind a 64-bit host `long`.

`attachInterrupt()` is a no-op, so `block()` invokes `gate1ISR()` / `gate2ISR()`
directly. Gate 3 is polled by the firmware and needs only the pin state.

The display stub records printed text instead of rendering, so a test can assert
on what the screen says.

## Adding a test

Put it in `test_regressions.cpp` if it pins down a specific bug, otherwise in
`test_behaviour.cpp`. Use `check()` for booleans and `near()` for floats.

Then confirm the test can actually fail -- reintroduce the bug, run the suite,
and check the assertion goes red. A regression test that passes either way is
worse than none, because it reads like coverage. The current suite was built
this way: all 13 deliberate reintroductions of known defects are caught.
