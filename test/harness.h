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
SerialStub Serial;
#include "../airspace_runway_monitor.ino"

namespace t {
const uint8_t G1 = 0, G2 = 1, G3 = 2;  // gate indices, not pins
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

// Put a simulated object a given distance in front of a gate's sensor, or take
// it away again. The gates are ultrasonic now, so "the plane is here" means a
// short range on that gate's echo pin, not a pin being pulled low.
void placeAtGate(uint8_t index, float cm) {
  const uint8_t pin = gates[index].echoPin;
  sim.echoPinSet[pin] = true;
  sim.echoUsPin[pin] = (unsigned long)(cm * 58.31f);
}
void clearGateSensor(uint8_t index) {
  const uint8_t pin = gates[index].echoPin;
  sim.echoPinSet[pin] = true;
  sim.echoUsPin[pin] = 0;  // no echo came back = nothing in front of it
}
void clearAllGates() {
  for (uint8_t i = 0; i < GATE_COUNT; i++) clearGateSensor(i);
}
// Roll the plane past one gate: it appears, is seen, and moves on.
void crossGate(uint8_t index, float cm = 8.0f) {
  placeAtGate(index, cm);
  step(0);
  clearGateSensor(index);
}

// Known-good starting point: nothing in front of any gate, scanning.
void clean() {
  clearAllGates();
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
// Advance until the given amount of time has really elapsed. Each loop()
// iteration polls a sensor, and a poll costs real time -- up to GATE_TIMEOUT_US
// when nothing is there -- so we cannot simply add microseconds and call it a
// segment. This is also exactly where the accuracy limit comes from: a gate is
// only seen as often as it can be polled.
void advanceBy(unsigned long us) {
  const unsigned long target = sim.nowUs + us;
  while (sim.nowUs < target) step(0);
}

// Roll the plane through gate 1 and gate 2 only, leaving the run part way
// through so the live telemetry can be inspected before gate 3 recomputes it.
void passToGate2(unsigned long seg1Ms, unsigned long bounceMs = 0) {
  crossGate(G1);
  const unsigned long touchdown = sim.nowUs;
  if (bounceMs) {
    advanceBy(bounceMs * 1000);
    crossGate(G1);  // wing or tail passes the same sensor again
  }
  while (sim.nowUs - touchdown < seg1Ms * 1000) step(0);
  crossGate(G2);
}

// A complete pass: touchdown, speed gate, parking bay.
void pass(unsigned long seg1Ms, unsigned long seg2Ms, unsigned long bounceMs = 0) {
  passToGate2(seg1Ms, bounceMs);
  const unsigned long atGate2 = sim.nowUs;
  while (sim.nowUs - atGate2 < seg2Ms * 1000) step(0);
  crossGate(G3);
  run(10000);
}

// Worst-case timing granularity: a gate is polled at best every GATE_TIMEOUT_US
// while it waits, so that is the most a crossing time can be out by.
float pollQuantisationMs() { return GATE_TIMEOUT_US / 1000.0f; }
}  // namespace t
