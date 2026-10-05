// Regression tests: each one pins down a defect that was present in the first
// version of the firmware. They are written so that reintroducing the bug makes
// the assertion fail, not merely look different.
#include "harness.h"
using namespace t;

int main() {
  printf("=== regression tests ===\n\n");
  setup();

  printf("[1] a repeat gate 1 break must not move the measurement baseline\n");
  printf("    (the maths once read the volatile ISR timestamp, which the ISR\n");
  printf("     keeps overwriting even when the event itself is discarded)\n");
  clean(); setRangeCm(40); run(200000);
  pass(250, 300);
  near("clean pass: v1", velocity1Ms, 0.400f, 0.005f);
  clean(); setRangeCm(40); run(200000);
  pass(250, 300, 40);
  near("40ms gate 1 bounce: final v1 unchanged", velocity1Ms, 0.400f, 0.005f);
  // calculateMotion() recomputes v1 from the snapshots at gate 3, so the bad
  // value is only visible mid-rollout. Check it there as well, otherwise the
  // interesting half of the bug can be reintroduced undetected.
  clean(); setRangeCm(40); run(200000);
  passToGate2(250, 40);
  near("40ms bounce: live v1 during rollout", velocity1Ms, 0.400f, 0.005f);
  check("still rolling out when checked", state == STATE_ROLLING_OUT);

  printf("\n[1b] gate 3 latency must stay inside the loop period\n");
  printf("     (a 2ms poll timer is invisible on a 300ms segment but is 10%%\n");
  printf("      of a 20ms one, and it lands entirely in segment 2)\n");
  clean(); setRangeCm(40); run(200000);
  pass(60, 20);                                    // 15cm / 20ms = 7.5 m/s
  near("v2 on a 20ms segment", velocity2Ms, 7.5f, 0.25f);
  check("segment 2 not inflated by poll latency",
        velocity2Ms > 7.0f,
        "v2 = " + std::to_string(velocity2Ms) + " m/s");

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

  printf("\n[6] repeat gate 1 breaks must not disturb a run in progress\n");
  clean();
  block(G1); step(1000); clear(G1);
  check("first break starts a rollout", state == STATE_ROLLING_OUT);
  {
    const uint32_t accepted = gate1AcceptedUs;
    const uint32_t started = rolloutStartedMs;
    run(10000);
    block(G1); step(0); clear(G1);                 // wing or tail crosses gate 1
    run(10000);
    block(G1); step(0); clear(G1);                 // and again
    check("baseline timestamp untouched", gate1AcceptedUs == accepted);
    check("rollout not restarted", rolloutStartedMs == started);
    check("still the same run", state == STATE_ROLLING_OUT);
    // Finish the pass and confirm the extra edges did not reach the maths.
    run(230000);
    block(G2); step(0); clear(G2); step(1000);
    near("v1 still measured from the first edge", velocity1Ms, 0.400f, 0.010f);
  }
  printf("      (the lockout in acceptGate1() is defence in depth: the state\n");
  printf("       machine already discards gate 1 edges outside SCANNING)\n");

  printf("\n[7] a plane left blocking gate 3 must not fake-trigger the next run\n");
  block(G3); run(100000);
  resetToScanning();
  check("edge detector seeded from the live pin", gate3PreviousActive == true);
  clear(G3);

  return report();
}
