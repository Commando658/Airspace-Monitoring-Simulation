// Shared scaffolding for the host-side tests.
//
// The stubs in stubs/ stand in for the Arduino core and the three libraries, so
// airspace_runway_monitor.ino compiles as ordinary C++ and the state machine can
// be driven against a simulated clock, simulated pins and a captured display.
//
// This is not a substitute for testing on real hardware -- it says nothing about
// sensor polarity, ultrasonic noise, servo power or actual gate alignment. What
// it does cover is timing arithmetic, state transitions and LED/buzzer logic,
// none of which are convenient to check on a bench.
#pragma once
#include "stubs/Arduino.h"
#include "stubs/Wire.h"
#include "stubs/EEPROM.h"
Sim sim;
TwoWire Wire;
EEPROMClass EEPROM;
#include "../airspace_runway_monitor.ino"

namespace t {
const uint8_t G1 = 2, G2 = 3, G3 = 4;
const uint8_t AIR_YELLOW = 9, AIR_GREEN = 10, AIR_RED = 11;
const uint8_t RUN_YELLOW = 12, RUN_GREEN = A0, RUN_RED = A1;

int passed = 0, failed = 0;

void check(const char *name, bool ok, const std::string &detail = "") {
  printf("  %-4s %-54s %s\n", ok ? "PASS" : "FAIL", name, detail.c_str());
  ok ? passed++ : failed++;
}
void near(const char *name, float got, float want, float tol) {
  char d[80];
  snprintf(d, sizeof d, "got %.4f want %.4f", got, want);
  check(name, fabs(got - want) <= tol, d);
}
int report() {
  printf("\n--- %d passed, %d failed ---\n", passed, failed);
  return failed ? 1 : 0;
}

// Advance the simulated clock and run one loop() iteration.
void step(unsigned long us) { sim.nowUs += us; loop(); }
void run(unsigned long us, unsigned long chunk = 1000) {
  for (unsigned long t = 0; t < us; t += chunk) step(chunk);
}
// Park the 32-bit micros() view just short of its wrap. sim.nowUs keeps counting
// underneath, so a pass started here straddles the boundary.
void seekToJustBeforeMicrosWrap(unsigned long marginUs = 2000) {
  sim.nowUs = 0x100000000UL - marginUs;
}
void setRangeCm(float cm) { sim.echoUs = (unsigned long)(cm * 58.31f); }
void clear(uint8_t pin) { sim.pinState[pin] = HIGH; }
// attachInterrupt() is a no-op in the stubs, so an edge on gate 1 or gate 2 has
// to invoke its handler explicitly. Gate 3 is polled, so it only needs the pin.
void block(uint8_t pin) {
  sim.pinState[pin] = LOW;
  if (pin == G1) gate1ISR();
  if (pin == G2) gate2ISR();
}

// Known-good starting point: all beams clear, scanning, nothing pending.
void clean() {
  clear(G1); clear(G2); clear(G3);
  resetToScanning();
  run(300000);
}
const char *airZone() {
  if (sim.pinState[AIR_RED] == HIGH) return "INTRUSION";
  if (sim.pinState[AIR_GREEN] == HIGH) return "OPTIMAL";
  if (sim.pinState[AIR_YELLOW] == HIGH) return "APPROACH";
  return "clear";
}
bool screenContains(const char *needle) {
  for (auto &l : sim.screen)
    if (l.find(needle) != std::string::npos) return true;
  return false;
}

// Drive a complete pass. bounceMs > 0 re-breaks gate 1 that long after the
// accepted edge, simulating a wing, tail or vibration.
// Break gate 1 and gate 2 only, leaving the run mid-measurement so the live
// telemetry can be inspected before calculateMotion() recomputes everything.
void passToGate2(unsigned long seg1Ms, unsigned long bounceMs = 0) {
  block(G1); step(1000); clear(G1);
  unsigned long consumed = 1000;
  if (bounceMs) {
    run(bounceMs * 1000); consumed += bounceMs * 1000;
    block(G1); step(0); clear(G1);
  }
  run(seg1Ms * 1000 - consumed);
  block(G2); step(0); clear(G2); step(1000);
}

void pass(unsigned long seg1Ms, unsigned long seg2Ms, unsigned long bounceMs = 0) {
  block(G1); step(1000); clear(G1);
  unsigned long consumed = 1000;
  if (bounceMs) {
    run(bounceMs * 1000); consumed += bounceMs * 1000;
    block(G1); step(0); clear(G1);
  }
  run(seg1Ms * 1000 - consumed);
  block(G2); step(0); clear(G2); step(1000);
  run(seg2Ms * 1000 - 1000);
  // step(0) so loop() runs at the exact instant the beam breaks; run() advances
  // in 1ms chunks and would otherwise charge a chunk of its own to segment 2.
  block(G3); step(0); run(10000);
  clear(G3);  // plane leaves the bay
}
}  // namespace t
