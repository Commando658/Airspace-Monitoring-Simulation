// Behavioural tests: the parts of the state machine and display logic that are
// awkward to confirm on a bench.
#include "harness.h"
using namespace t;

int main() {
  printf("=== behaviour tests ===\n\n");
  setup();

  printf("[1] a complete pass produces consistent telemetry\n");
  clean(); setRangeCm(40); run(200000);
  pass(250, 300);
  near("v1 = 10cm / 250ms", velocity1Ms, 0.400f, 0.005f);
  near("v2 = 15cm / 300ms", velocity2Ms, 0.500f, 0.010f);
  // v1 at the midpoint of segment 1, v2 at the midpoint of segment 2:
  // dt = (seg1 + seg2) / 2 = 275ms, so a = 0.1 / 0.275.
  near("acceleration uses segment midpoints", accelerationMs2, 0.364f, 0.020f);
  check("ends in PARKED", state == STATE_PARKED);
  check("summary is drawn", screenContains("SUMMARY"));
  run(200000);
  check("summary shows both velocities",
        screenContains("V1:") && screenContains("V2:"));

  printf("\n[2] airspace zones map to the right LED and alarm\n");
  struct { float cm; const char *zone; } cases[] = {
    {  5.0f, "INTRUSION" },   // inside AIR_GREEN_MIN_CM
    { 18.0f, "OPTIMAL"   },   // within the green band
    { 40.0f, "APPROACH"  },   // beyond the green band
  };
  for (auto &c : cases) {
    clean(); setRangeCm(c.cm); run(300000);
    char n[64]; snprintf(n, sizeof n, "%.0fcm -> %s", c.cm, c.zone);
    check(n, std::string(airZone()) == c.zone, airZone());
  }
  clean(); sim.echoUs = 0; run(300000);            // no echo at all
  check("no echo -> all airspace LEDs off", std::string(airZone()) == "clear");
  check("no echo -> NO TARGET on screen", screenContains("NO TARGET"));

  printf("\n[3] a rollout that never reaches gate 3 recovers on its own\n");
  clean();
  block(G1); step(1000); clear(G1);
  check("entered ROLLING_OUT", state == STATE_ROLLING_OUT);
  run(7000000);
  check("still rolling out at 7s", state == STATE_ROLLING_OUT);
  run(1500000);
  check("timed out back to SCANNING by 8.5s", state == STATE_SCANNING);

  printf("\n[4] the telemetry screen is frozen while gate 3 is awaited\n");
  printf("    (a framebuffer push blocks ~23ms at 400kHz and gate 3 is polled,\n");
  printf("     so a periodic redraw would land inside segment 2)\n");
  clean(); setRangeCm(40); run(200000);
  block(G1); step(1000); clear(G1);
  run(100000);
  block(G2); step(0); clear(G2);
  run(100000);
  {
    const size_t drawn = sim.screen.size();
    run(2000000);                                  // 2s of waiting
    check("no repeated pushes during the wait", sim.screen.size() == drawn,
          "redraws = " + std::to_string(sim.screen.size() - drawn));
  }

  printf("\n[5] the radar sweeps the full arc and reverses\n");
  clean(); setRangeCm(40);
  {
    int lo = 999, hi = -1;
    for (int i = 0; i < 400; i++) {
      run(80000);
      if (radarAngle < lo) lo = radarAngle;
      if (radarAngle > hi) hi = radarAngle;
    }
    check("reaches 0 and 180", lo == 0 && hi == 180,
          "lo = " + std::to_string(lo) + ", hi = " + std::to_string(hi));
  }
  check("blips scale to the display range, not the sensor limit",
        RADAR_DISPLAY_MAX_CM < SONAR_MAX_CM,
        std::to_string((int)RADAR_DISPLAY_MAX_CM) + "cm at the outer arc");

  printf("\n[6] the monitor survives a missing OLED\n");
  displayReady = false;
  clean(); setRangeCm(5.0f);
  sim.screen.clear();
  run(300000);
  // The safety outputs are the point; the panel is optional.
  check("intrusion LED still driven", sim.pinState[AIR_RED] == HIGH);
  check("alarm still sounding", sim.toneHz == 2200);
  check("nothing written to the absent panel", sim.screen.empty(),
        "lines drawn = " + std::to_string(sim.screen.size()));
  displayReady = true;

  printf("\n[7] the buzzer only touches Timer2 when the output changes\n");
  clean(); setRangeCm(5.0f);
  {
    const int before = sim.rawToneCalls;
    run(1000000);                                  // 1s of continuous alarm
    const int calls = sim.rawToneCalls - before;
    check("one tone() call for a steady alarm, not thousands", calls <= 2,
          "tone() calls = " + std::to_string(calls));
  }

  printf("\n[8] EEPROM records persist and count completed runs\n");
  {
    clean(); setRangeCm(40); run(200000);
    const uint16_t before = records.completedRuns;
    pass(250, 300);
    check("counter advanced", records.completedRuns == before + 1,
          std::to_string(before) + " -> " + std::to_string(records.completedRuns));
    const PerformanceRecord saved = records;
    records.magic = 0;                             // force a reload path
    loadRecords();
    check("records survive a reload",
          records.magic == EEPROM_MAGIC &&
          records.completedRuns == saved.completedRuns &&
          records.fastestVelocity == saved.fastestVelocity);
  }

  return report();
}
