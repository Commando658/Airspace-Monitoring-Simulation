// Regression tests: each one pins down a defect that was present in the first
// version of the firmware. They are written so that reintroducing the bug makes
// the assertion fail, not merely look different.
#include "harness.h"
using namespace t;

int main() {
  printf("=== regression tests ===\n\n");
  setup();

  printf("[1] a second look at gate 1 must not move the measurement baseline\n");
  printf("    (the maths once read a timestamp that kept being overwritten by\n");
  printf("     every later sighting, even ones that were discarded)\n");
  clean(); setRangeCm(40); run(200000);
  pass(250, 300);
  near("clean pass: v1", velocity1Ms, 0.400f, 0.012f);
  clean(); setRangeCm(40); run(200000);
  pass(250, 300, 40);
  near("40ms re-sighting: final v1 unchanged", velocity1Ms, 0.400f, 0.012f);
  // calculateMotion() recomputes v1 from the snapshots at gate 3, so the bad
  // value is only visible mid-rollout. Check it there as well, otherwise the
  // interesting half of the bug can be reintroduced undetected.
  clean(); setRangeCm(40); run(200000);
  passToGate2(250, 40);
  near("40ms re-sighting: live v1 mid-rollout", velocity1Ms, 0.400f, 0.012f);
  check("still rolling out when checked", state == STATE_ROLLING_OUT);

  printf("\n[1b] a gate crossing must be timed to within one poll period\n");
  printf("     (%.1f ms here -- this is the real accuracy limit of ultrasonic\n", pollQuantisationMs());
  printf("      gates, and it lands entirely inside the segment being measured)\n");
  clean(); setRangeCm(40); run(200000);
  pass(250, 300);
  {
    const uint32_t seg1 = gate2AcceptedUs - gate1AcceptedUs;
    const float errMs = (seg1 / 1000.0f) - 250.0f;
    check("segment 1 within one poll of the true 250ms",
          fabs(errMs) <= pollQuantisationMs() + 0.5f,
          "out by " + std::to_string(errMs) + " ms");
    check("and never short -- a poll can only be late, never early",
          errMs >= -0.5f, "err = " + std::to_string(errMs) + " ms");
  }

  printf("\n[2] a target too close to range is still a target\n");
  clean(); setRangeCm(1.0f); run(300000);
  check("1cm reads RANGE_TOO_CLOSE", radarRangeCm <= RANGE_TOO_CLOSE);
  check("escalated to an intrusion", std::string(airZone()) == "INTRUSION", airZone());
  check("alarm sounding", sim.toneHz == 2200);
  check("screen does not claim NO TARGET", !screenContains("NO TARGET"));

  printf("\n[3] a reset must not leave the previous run's range on screen\n");
  clean(); setRangeCm(40); run(300000);
  check("range populated before reset", radarRangeCm > 0.0f);
  resetToScanning();
  check("range cleared by reset", radarRangeCm == RANGE_NO_ECHO);
  lastDisplayUpdateMs = 0; sim.nowUs += 200000; updateDisplay();
  check("no stale INTRUSION frame", !screenContains("AIRSPACE INTRUSION"));

  printf("\n[4] parking must report the band actually achieved\n");
  clean(); setRangeCm(40); run(200000);
  pass(250, 300);                                  // v2 ~0.50, above the 0.40 max
  check("overshoot parks on red, not green",
        sim.pinState[RUN_RED] == HIGH && sim.pinState[RUN_GREEN] == LOW);
  clean(); setRangeCm(40); run(200000);
  pass(400, 500);                                  // v2 ~0.30, inside the band
  check("safe rollout parks on green",
        sim.pinState[RUN_GREEN] == HIGH && sim.pinState[RUN_RED] == LOW);

  printf("\n[5] micros() wraps every ~71.6 min; a pass across it must survive\n");
  // End to end: micros() is 32-bit in the stubs, so parking the clock just
  // short of the boundary makes a real pass straddle it.
  // Wrap inside segment 1 (between gate 1 and gate 2).
  clean(); setRangeCm(40); run(200000);
  seekToJustBeforeMicrosWrap(2000);
  pass(250, 300);
  near("wrap in segment 1: v1", velocity1Ms, 0.400f, 0.005f);
  near("wrap in segment 1: v2", velocity2Ms, 0.500f, 0.010f);
  check("wrap in segment 1: parked", state == STATE_PARKED);
  // Wrap inside segment 2 (between gate 2 and gate 3). This is a separate code
  // path -- the gate 3 interval is validated on its own.
  clean(); setRangeCm(40); run(200000);
  seekToJustBeforeMicrosWrap(252000);
  pass(250, 300);
  near("wrap in segment 2: v1", velocity1Ms, 0.400f, 0.005f);
  near("wrap in segment 2: v2", velocity2Ms, 0.500f, 0.010f);
  check("wrap in segment 2: parked", state == STATE_PARKED);
  {
    typedef uint32_t u32;
    auto within = [](u32 s, u32 e, u32 lim) { u32 d = e - s; return d > 0 && d <= lim; };
    const u32 g1 = 0xFFFFFF00u, g2 = g1 + 250000u, g3 = g2 + 300000u;
    check("gate1->gate2 accepted across the wrap", within(g1, g2, 5000000u));
    check("gate2->gate3 accepted across the wrap", within(g2, g3, 5000000u));
    check("the old ordering test would have rejected it", g2 <= g1,
          "g2 <= g1 is true across the wrap");
    check("zero-length interval still rejected", !within(g1, g1, 5000000u));
    check("implausible 10s interval rejected", !within(0u, 10000000u, 5000000u));
  }

  printf("\n[6] repeat sightings at gate 1 must not disturb a run in progress\n");
  clean();
  crossGate(G1);
  check("first sighting starts a rollout", state == STATE_ROLLING_OUT);
  {
    const uint32_t accepted = gate1AcceptedUs;
    const uint32_t started = rolloutStartedMs;
    const unsigned long touchdown = sim.nowUs;
    advanceBy(10000);
    crossGate(G1);                                 // wing crosses gate 1
    advanceBy(10000);
    crossGate(G1);                                 // and again
    check("baseline timestamp untouched", gate1AcceptedUs == accepted);
    check("rollout not restarted", rolloutStartedMs == started);
    check("still the same run", state == STATE_ROLLING_OUT);
    while (sim.nowUs - touchdown < 250000) step(0);
    crossGate(G2);
    near("v1 still measured from the first sighting", velocity1Ms, 0.400f, 0.012f);
  }
  printf("      (gate 1 is only polled in SCANNING, so once a run is under way\n");
  printf("       its sensor is not even being looked at)\n");

  printf("\n[7] a plane still parked in front of gate 3 must not count itself\n");
  clean();
  placeAtGate(G3, 8.0f);                           // plane sitting in the bay
  resetToScanning();                               // rearmGates() reads every sensor
  check("gate 3 left disarmed while occupied", gates[G3].armed == false);
  clearGateSensor(G3);                             // plane lifted out
  resetToScanning();
  check("armed again once the bay is clear", gates[G3].armed == true);

  printf("\n[8] hysteresis: one aeroplane fires a gate exactly once\n");
  clean();
  {
    uint32_t at = 0;
    placeAtGate(G1, 8.0f);
    check("fuselage arrives -> gate fires", pollGate(G1, &at) == true);
    check("still in front of it -> no second count", pollGate(G1, &at) == false);
    placeAtGate(G1, 25.0f);                        // between trigger and release
    pollGate(G1, &at);
    check("inside the hysteresis gap, still disarmed", gates[G1].armed == false);
    placeAtGate(G1, 35.0f);                        // past GATE_RELEASE_CM
    pollGate(G1, &at);
    check("past the release line -> re-armed", gates[G1].armed == true);
    placeAtGate(G1, 8.0f);
    check("next aeroplane fires it again", pollGate(G1, &at) == true);
  }
  printf("      (release is %.0fcm, trigger is %.0fcm -- the gap is what stops\n",
         GATE_RELEASE_CM, GATE_TRIGGER_CM);
  printf("       a reading jittering across one line from chattering)\n");

  printf("\n[9] a crossing is timed mid-flight, not when the echo lands\n");
  printf("    (the sound has to travel out and back; the plane was at the gate\n");
  printf("     halfway through that, not at the end of it)\n");
  clean();
  {
    placeAtGate(G1, 18.0f);                        // ~1.05ms round trip
    const uint32_t before = micros();
    uint32_t at = 0;
    const bool fired = pollGate(G1, &at);
    const uint32_t after = micros();
    check("gate fired", fired);
    check("timestamp is inside the read, not at the end of it",
          (uint32_t)(at - before) > 0 && (uint32_t)(after - at) > 0,
          "read spanned " + std::to_string(after - before) + "us, stamped at +" +
          std::to_string(at - before) + "us");
    const uint32_t firstHalf = at - before, secondHalf = after - at;
    const uint32_t diff = firstHalf > secondHalf ? firstHalf - secondHalf
                                                 : secondHalf - firstHalf;
    check("and lands near the middle of it", diff < (after - before) / 2,
          std::to_string(firstHalf) + "us before / " + std::to_string(secondHalf) + "us after");
  }

  printf("\n[10] a gate sighting closer than the sensor can measure still counts\n");
  clean();
  {
    uint32_t at = 0;
    placeAtGate(G1, 1.0f);                         // under SONAR_MIN_CM
    check("1cm in front of a gate still fires it", pollGate(G1, &at) == true);
  }

  printf("\n[11] an implausibly long segment must be rejected, not measured\n");
  printf("     (GATE_MAX_SEGMENT_US is %lus; the rollout timeout is %lus, so\n",
         (unsigned long)(GATE_MAX_SEGMENT_US / 1000000UL),
         (unsigned long)(ROLLOUT_TIMEOUT_MS / 1000UL));
  printf("      there is a window where a stray late echo could otherwise land)\n");
  clean();
  crossGate(G1);
  {
    const unsigned long touchdown = sim.nowUs;
    while (sim.nowUs - touchdown < 200000) step(0);
    crossGate(G2);
    check("gate 2 accepted normally", gate2Captured == true);
    const unsigned long atGate2 = sim.nowUs;
    while (sim.nowUs - atGate2 < 6000000) step(0);  // 6s: past the limit, inside the timeout
    crossGate(G3);
    check("stale gate 3 rejected, run not parked", state != STATE_PARKED);
    check("no bogus second velocity recorded", haveVelocity2 == false);
  }

  return report();
}
