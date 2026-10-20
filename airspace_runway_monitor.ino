/*
  Airspace & Runway Monitoring System
  Target: Arduino Nano/UNO (ATmega328P)

  The three runway gates are ultrasonic (HC-SR04), not IR beam-break. See the
  README for wiring and for how to change the pin map to match your board --
  everything you need to touch is in the CONFIGURATION block below.

  Pin map (defaults -- change them if your wiring differs):
    D2  gate 1 trigger    D8  servo signal      A0  runway green LED
    D3  gate 1 echo       D9  airspace yellow   A1  runway red LED
    D4  gate 2 trigger    D10 airspace green    A2  gate 3 trigger
    D13 gate 2 echo       D11 airspace red      A3  gate 3 echo
    D5  active buzzer     D12 runway yellow     A4/A5 OLED SDA/SCL
    D6  airspace radar trigger
    D7  airspace radar echo

  With telemetry on, D0/D1 are the USB serial lines, so every pin on the chip
  is in use. If you need a pin back, turn off ENABLE_TELEMETRY or, if the
  servo-mounted radar is not fitted, HAVE_AIRSPACE_RADAR.

  Power the SG90 servo from a separate regulated 5 V supply and connect its
  ground to Arduino GND. Do not power the servo from an Arduino I/O pin.

  State transitions:
    SCANNING   -- valid gate 1 edge --> ROLLING_OUT
    ROLLING_OUT -- valid gate 3 edge --> PARKED
    ROLLING_OUT -- 8 s timeout ------> SCANNING
    PARKED     -- summary timer ------> SCANNING

  Timing notes:
    All timestamps are uint32_t, matching the width of micros()/millis() on AVR.
    The width is stated explicitly because the wrap behaviour below depends on
    it, and because it lets this file be tested faithfully on a 64-bit host.

    micros() wraps every ~71.6 minutes. Unsigned subtraction survives the wrap
    but ordering two raw timestamps does not, so every gate interval is checked
    as an elapsed duration (see intervalWithin).

    An ultrasonic sensor gives no edge to interrupt on. Each gate has to be
    triggered and then listened to, so the gates are polled and the timing is
    only as good as how often we can poll them. Two things protect that:

      - Only one sensor is ever listening. Several HC-SR04s close together hear
        each other's chirps, and during a rollout only one gate can possibly be
        next, so we poll that one alone. It avoids the interference and gives
        the gate that matters the full sample rate (~1.2 ms rather than ~3.5 ms
        if all three were polled in turn).

      - Nothing else in the loop is allowed to block. Pushing the 1 KB
        framebuffer over I2C takes ~23 ms, which would land straight inside a
        measurement, so the telemetry screen is redrawn only when it changes.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ===========================================================================
// CONFIGURATION -- this is the part you change for your board.
// ===========================================================================

// Set to 0 if the servo-mounted airspace radar is not fitted. The servo, the
// radar screen and the three airspace LEDs all drop out with it.
#define HAVE_AIRSPACE_RADAR 1

// Serial telemetry at 115200: one line per completed run. Useful while you are
// tuning; costs ~175 bytes of RAM, so turn it off once the numbers are right.
#define ENABLE_TELEMETRY 1

// Gate sensors. Each needs its own trigger and echo pin -- they cannot share a
// trigger, because firing them together makes them hear each other.
const uint8_t PIN_GATE1_TRIG = 2;
const uint8_t PIN_GATE1_ECHO = 3;
const uint8_t PIN_GATE2_TRIG = 4;
const uint8_t PIN_GATE2_ECHO = 13;
const uint8_t PIN_GATE3_TRIG = A2;
const uint8_t PIN_GATE3_ECHO = A3;

// How close the plane has to be before a gate counts it as arrived, and how far
// away it has to get before that gate will count anything again. The gap
// between the two stops a long fuselage from firing the same gate repeatedly as
// it slides past. Measure the gap between your sensor and the plane and set
// GATE_TRIGGER_CM comfortably above it.
const float GATE_TRIGGER_CM = 20.0f;
const float GATE_RELEASE_CM = 30.0f;
const float GATE_MAX_RANGE_CM = 45.0f;  // Beyond this a gate reports "nothing there".

const uint8_t PIN_BUZZER = 5;
const uint8_t PIN_SONAR_TRIGGER = 6;
const uint8_t PIN_SONAR_ECHO = 7;
const uint8_t PIN_SERVO = 8;
const uint8_t PIN_AIR_YELLOW = 9;
const uint8_t PIN_AIR_GREEN = 10;
const uint8_t PIN_AIR_RED = 11;
const uint8_t PIN_RUNWAY_YELLOW = 12;
const uint8_t PIN_RUNWAY_GREEN = A0;
const uint8_t PIN_RUNWAY_RED = A1;

// OLED geometry and I2C address.
const uint8_t OLED_WIDTH = 128;
const uint8_t OLED_HEIGHT = 64;
const uint8_t OLED_ADDRESS = 0x3C;

// Measure these along the runway with a ruler and put the real numbers here.
// Every velocity is distance over time, so if these are wrong every reading the
// system produces is wrong by the same proportion, silently.
const float GATE_1_TO_2_CM = 10.0f; // Distance from gate 1 to gate 2.
const float GATE_2_TO_3_CM = 15.0f; // Distance from gate 2 to gate 3.

// Airspace thresholds in centimetres.
const float AIR_GREEN_MIN_CM = 10.0f; // Lower edge of the optimal airspace band.
const float AIR_GREEN_MAX_CM = 25.0f; // Upper edge of the optimal airspace band.
const float SONAR_MIN_CM = 2.0f;      // Below this the HC-SR04 cannot resolve range.
const float SONAR_MAX_CM = 200.0f;    // Reject echoes farther than this.

// readRangeCm() sentinels. A target too close to measure is still a target, so
// it is reported separately from "nothing came back" and escalated to an
// intrusion rather than being shown as clear airspace.
const float RANGE_NO_ECHO = -1.0f;
const float RANGE_TOO_CLOSE = -2.0f;

// Runway velocity thresholds in metres per second. These defaults are slower
// than walking pace and are almost certainly wrong for a real model aircraft --
// run a few passes with telemetry on and set them from what you actually see.
const float RUNWAY_GREEN_MIN_MS = 0.15f; // Lower edge of safe rollout.
const float RUNWAY_GREEN_MAX_MS = 0.40f; // Upper edge of safe rollout.

// ===========================================================================
// END OF CONFIGURATION
// ===========================================================================

// Timing and filtering values.
const uint32_t SONAR_TIMEOUT_US = 12000UL; // Radar: listen out to ~2 m.
// Gates only care about things that are close, so they give up listening much
// sooner. This matters: it is how long a poll costs when nothing is there, and
// that cost is the gap between samples while we wait for the plane.
const uint32_t GATE_TIMEOUT_US = 2600UL;
const uint32_t ROLLOUT_TIMEOUT_MS = 8000UL; // Give up if gate 3 is not reached.
const uint32_t SUMMARY_HOLD_MS = 4000UL;   // Summary screen duration.
const uint32_t GATE_MAX_SEGMENT_US = 5000000UL; // Longest credible gate-to-gate interval.
const uint32_t RADAR_UPDATE_MS = 70UL;     // Servo/radar update period.
const uint32_t DISPLAY_UPDATE_MS = 100UL;  // Display refresh period.
const uint32_t FLASH_INTERVAL_MS = 180UL;  // Red runway LED flash period.
const uint32_t SLOW_BEEP_INTERVAL_MS = 1200UL; // Approach pulse period.
const uint32_t SLOW_BEEP_ON_MS = 120UL;     // Approach pulse duration.
const uint32_t RAPID_BEEP_INTERVAL_MS = 180UL; // Overshoot alarm period.
const uint32_t RAPID_BEEP_ON_MS = 90UL;     // Overshoot pulse duration.
const uint32_t SINGLE_BEEP_MS = 130UL;      // One zone-entry beep.
const uint32_t DOUBLE_GAP_MS = 90UL;        // Gap between double beeps.

// Radar plot geometry.
const uint8_t RADAR_CENTER_X = 64;
const uint8_t RADAR_BASELINE_Y = 63;
const uint8_t RADAR_RADIUS = 45;
const uint8_t RADAR_ARC_1 = 15;
const uint8_t RADAR_ARC_2 = 30;
const uint8_t RADAR_ARC_3 = 45;
const uint8_t RADAR_STEP_DEG = 3;               // Sweep step; 1 deg took 12.6 s per pass.
const float RADAR_DISPLAY_MAX_CM = 60.0f;       // Range mapped to the outermost arc.

const uint16_t EEPROM_MAGIC = 0xA53D; // Bumped: record layout gained completedRuns.
const int EEPROM_RECORD_ADDRESS = 0;

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
Servo radarServo;

enum SystemState : uint8_t {
  STATE_SCANNING,
  STATE_ROLLING_OUT,
  STATE_PARKED
};

enum AirZone : uint8_t {
  AIR_NONE,
  AIR_APPROACH,
  AIR_OPTIMAL,
  AIR_INTRUSION
};

enum RunwayBand : uint8_t {
  RUNWAY_TAXIING,
  RUNWAY_SAFE,
  RUNWAY_OVERSHOOT
};

enum BuzzerPattern : uint8_t {
  BUZZER_OFF,
  BUZZER_SLOW,
  BUZZER_CONTINUOUS,
  BUZZER_SINGLE,
  BUZZER_DOUBLE,
  BUZZER_RAPID
};

struct PerformanceRecord {
  uint16_t magic;
  uint16_t completedRuns;
  float fastestVelocity;
  float highestAcceleration;
};

// One of these per runway gate. "armed" is what stops a gate firing twice for
// the same aeroplane: once a gate has counted something it stays disarmed until
// the range climbs back past GATE_RELEASE_CM, i.e. until the plane has gone.
struct UltrasonicGate {
  uint8_t trigPin;
  uint8_t echoPin;
  bool armed;
  float lastRangeCm;
};

const uint8_t GATE_COUNT = 3;
UltrasonicGate gates[GATE_COUNT] = {
  { PIN_GATE1_TRIG, PIN_GATE1_ECHO, true, -1.0f },
  { PIN_GATE2_TRIG, PIN_GATE2_ECHO, true, -1.0f },
  { PIN_GATE3_TRIG, PIN_GATE3_ECHO, true, -1.0f },
};

SystemState state = STATE_SCANNING;
AirZone previousAirZone = AIR_NONE;
RunwayBand previousRunwayBand = RUNWAY_TAXIING;
BuzzerPattern buzzerPattern = BUZZER_OFF;
PerformanceRecord records;

uint32_t rolloutStartedMs = 0;
uint32_t summaryStartedMs = 0;
uint32_t lastRadarUpdateMs = 0;
uint32_t lastDisplayUpdateMs = 0;
uint32_t buzzerStartedMs = 0;
uint32_t flashStartedMs = 0;

// The accepted crossing times. Everything downstream works from these, and they
// are only written once per gate per run, so a later reading at the same gate
// cannot move a measurement that has already been taken.
uint32_t gate1AcceptedUs = 0;
uint32_t gate2AcceptedUs = 0;
bool gate1Captured = false;
bool gate2Captured = false;

uint32_t lastSegment2Us = 0;  // Kept for the telemetry line.
bool displayReady = false;
bool telemetryDirty = false;
bool haveVelocity1 = false;
bool haveVelocity2 = false;
bool haveAcceleration = false;
float velocity1Ms = 0.0f;
float velocity2Ms = 0.0f;
float accelerationMs2 = 0.0f;
float radarRangeCm = RANGE_NO_ECHO;
uint8_t radarAngle = 0;
int8_t radarDirection = 1;

// Forward declarations. The Arduino preprocessor would synthesise these, but
// declaring them keeps the file buildable as ordinary C++ too.
void updateBuzzer();
void clearMotionData();
void calculateMotion(uint32_t gate3Us);
bool pollGate(uint8_t index, uint32_t *detectionUs);
void reportRun();

// micros() wraps every ~71.6 minutes. (end - start) is correct across the wrap
// for unsigned arithmetic; (end > start) is not. Every interval is validated here
// as a duration so a pass straddling the wrap is measured rather than discarded.
bool intervalWithin(uint32_t startUs, uint32_t endUs, uint32_t limitUs) {
  const uint32_t elapsedUs = endUs - startUs;
  return elapsedUs > 0UL && elapsedUs <= limitUs;
}

float segmentVelocity(float distanceCm, uint32_t elapsedUs) {
  return (distanceCm / 100.0f) / (elapsedUs / 1000000.0f);
}

RunwayBand bandForSpeed(float speedMs) {
  if (speedMs >= RUNWAY_GREEN_MAX_MS) {
    return RUNWAY_OVERSHOOT;
  }
  if (speedMs >= RUNWAY_GREEN_MIN_MS) {
    return RUNWAY_SAFE;
  }
  return RUNWAY_TAXIING;
}

// Ping one sensor and convert the echo to centimetres. Every ultrasonic read in
// the sketch goes through here, gates and radar alike. echoUsOut, if given, gets
// the raw round-trip time, which the gates use to work out exactly when the
// sound reached the plane.
float readSensorCm(uint8_t trigPin, uint8_t echoPin, uint32_t timeoutUs,
                   float maxCm, uint32_t *echoUsOut) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  const uint32_t durationUs = pulseIn(echoPin, HIGH, timeoutUs);
  if (echoUsOut != NULL) {
    *echoUsOut = durationUs;
  }
  if (durationUs == 0) {
    return RANGE_NO_ECHO;  // Nothing came back before we gave up listening.
  }

  const float distanceCm = durationUs * 0.0343f / 2.0f;
  if (distanceCm < SONAR_MIN_CM) {
    return RANGE_TOO_CLOSE;  // Something is there, just nearer than the sensor can measure.
  }
  if (distanceCm > maxCm) {
    return RANGE_NO_ECHO;
  }
  return distanceCm;
}

float readRangeCm() {
  return readSensorCm(PIN_SONAR_TRIGGER, PIN_SONAR_ECHO,
                      SONAR_TIMEOUT_US, SONAR_MAX_CM, NULL);
}

// Read one gate and say whether the plane has just arrived at it.
//
// detectionUs is set to the moment the sound actually reached the plane: the
// instant we fired, plus half the round trip. Using the same convention at
// every gate is what matters here -- a constant offset cancels out when we
// divide distance by elapsed time.
bool pollGate(uint8_t index, uint32_t *detectionUs) {
  UltrasonicGate &gate = gates[index];
  const uint32_t firedAtUs = micros();
  uint32_t echoUs = 0;
  const float rangeCm = readSensorCm(gate.trigPin, gate.echoPin,
                                     GATE_TIMEOUT_US, GATE_MAX_RANGE_CM, &echoUs);
  gate.lastRangeCm = rangeCm;

  // Closer than the sensor can measure still counts as something being there.
  const bool somethingThere = (rangeCm <= RANGE_TOO_CLOSE) ||
                              (rangeCm >= 0.0f && rangeCm <= GATE_TRIGGER_CM);

  if (!somethingThere) {
    if (rangeCm == RANGE_NO_ECHO || rangeCm >= GATE_RELEASE_CM) {
      gate.armed = true;  // Clear again, so this gate can count the next arrival.
    }
    return false;
  }
  if (!gate.armed) {
    return false;  // Same aeroplane, still in front of the sensor.
  }
  gate.armed = false;
  if (detectionUs != NULL) {
    *detectionUs = firedAtUs + (echoUs / 2UL);
  }
  return true;
}

// Called when a run ends. Each gate is read once so that a plane still sitting
// in front of a sensor leaves that gate disarmed, rather than counting itself
// the instant the next run begins.
void rearmGates() {
  for (uint8_t index = 0; index < GATE_COUNT; index++) {
    gates[index].armed = true;
    pollGate(index, NULL);
  }
}

void setAirspaceLeds(AirZone zone) {
  digitalWrite(PIN_AIR_YELLOW, zone == AIR_APPROACH ? HIGH : LOW);
  digitalWrite(PIN_AIR_GREEN, zone == AIR_OPTIMAL ? HIGH : LOW);
  digitalWrite(PIN_AIR_RED, zone == AIR_INTRUSION ? HIGH : LOW);
}

void setRunwayLeds(RunwayBand band, bool flashOn) {
  digitalWrite(PIN_RUNWAY_YELLOW, band == RUNWAY_TAXIING ? HIGH : LOW);
  digitalWrite(PIN_RUNWAY_GREEN, band == RUNWAY_SAFE ? HIGH : LOW);
  digitalWrite(PIN_RUNWAY_RED, band == RUNWAY_OVERSHOOT && flashOn ? HIGH : LOW);
}

void startBuzzer(BuzzerPattern pattern) {
  buzzerPattern = pattern;
  buzzerStartedMs = millis();
  updateBuzzer();  // Apply the new pattern immediately rather than on the next loop.
}

void updateBuzzer() {
  const uint32_t elapsedMs = millis() - buzzerStartedMs;
  bool on = false;

  switch (buzzerPattern) {
    case BUZZER_SLOW:
      on = (elapsedMs % SLOW_BEEP_INTERVAL_MS) < SLOW_BEEP_ON_MS;
      break;
    case BUZZER_CONTINUOUS:
      on = true;
      break;
    case BUZZER_SINGLE:
      on = elapsedMs < SINGLE_BEEP_MS;
      if (elapsedMs >= SINGLE_BEEP_MS) {
        buzzerPattern = BUZZER_OFF;
      }
      break;
    case BUZZER_DOUBLE:
      on = elapsedMs < SINGLE_BEEP_MS ||
           (elapsedMs >= SINGLE_BEEP_MS + DOUBLE_GAP_MS &&
            elapsedMs < (2UL * SINGLE_BEEP_MS) + DOUBLE_GAP_MS);
      if (elapsedMs >= (2UL * SINGLE_BEEP_MS) + DOUBLE_GAP_MS) {
        buzzerPattern = BUZZER_OFF;
      }
      break;
    case BUZZER_RAPID:
      on = (elapsedMs % RAPID_BEEP_INTERVAL_MS) < RAPID_BEEP_ON_MS;
      break;
    case BUZZER_OFF:
      break;
  }

  // Only touch the timer when the output actually changes; tone() reinitialises
  // Timer2 on every call and the loop runs thousands of times a second.
  static bool sounding = false;
  if (on && !sounding) {
    tone(PIN_BUZZER, 2200);
    sounding = true;
  } else if (!on && sounding) {
    noTone(PIN_BUZZER);
    sounding = false;
  }
}

void clearMotionData() {
  haveVelocity1 = false;
  haveVelocity2 = false;
  haveAcceleration = false;
  velocity1Ms = 0.0f;
  velocity2Ms = 0.0f;
  accelerationMs2 = 0.0f;
  gate1Captured = false;
  gate2Captured = false;
  gate1AcceptedUs = 0;
  gate2AcceptedUs = 0;
}

void loadRecords() {
  EEPROM.get(EEPROM_RECORD_ADDRESS, records);
  if (records.magic != EEPROM_MAGIC ||
      isnan(records.fastestVelocity) ||
      isnan(records.highestAcceleration)) {
    records.magic = EEPROM_MAGIC;
    records.completedRuns = 0;
    records.fastestVelocity = 0.0f;
    records.highestAcceleration = 0.0f;
    EEPROM.put(EEPROM_RECORD_ADDRESS, records);
  }
}

void updateRecords() {
  bool changed = true;  // A completed run always bumps the counter.
  if (records.completedRuns < 0xFFFFU) {
    records.completedRuns++;
  }
  if (haveVelocity2 && velocity2Ms > records.fastestVelocity) {
    records.fastestVelocity = velocity2Ms;
    changed = true;
  }
  if (haveAcceleration && accelerationMs2 > records.highestAcceleration) {
    records.highestAcceleration = accelerationMs2;
    changed = true;
  }
  if (changed) {
    EEPROM.put(EEPROM_RECORD_ADDRESS, records);
  }
}

// One line per completed run, so the numbers can be checked against a stopwatch
// and the speed bands set from real passes instead of guesswork.
#if ENABLE_TELEMETRY
void reportRun() {
  Serial.print(records.completedRuns);
  Serial.print(',');
  Serial.print(gate2AcceptedUs - gate1AcceptedUs);
  Serial.print(',');
  Serial.print(lastSegment2Us);
  Serial.print(',');
  Serial.print(velocity1Ms, 3);
  Serial.print(',');
  Serial.print(velocity2Ms, 3);
  Serial.print(',');
  Serial.println(accelerationMs2, 3);
}
#else
void reportRun() {}
#endif

void resetToScanning() {
  state = STATE_SCANNING;
  previousAirZone = AIR_NONE;
  radarRangeCm = RANGE_NO_ECHO;  // Otherwise the first frame redraws the last run's range.
  previousRunwayBand = RUNWAY_TAXIING;
  setAirspaceLeds(AIR_NONE);
  setRunwayLeds(RUNWAY_TAXIING, false);
  startBuzzer(BUZZER_OFF);
  clearMotionData();
  rearmGates();
}

// Touchdown: the plane has reached the first gate and the run begins.
void acceptGate1(uint32_t timestampUs) {
  clearMotionData();  // Wipes the snapshots, so record this one afterwards.
  gate1AcceptedUs = timestampUs;
  gate1Captured = true;
  rolloutStartedMs = millis();
  state = STATE_ROLLING_OUT;
  telemetryDirty = true;
}

// Second gate: this gives us the first speed, which is enough to drive the
// runway LEDs while we wait for the plane to reach the end.
void acceptGate2(uint32_t timestampUs) {
  if (!gate1Captured ||
      !intervalWithin(gate1AcceptedUs, timestampUs, GATE_MAX_SEGMENT_US)) {
    return;
  }
  gate2AcceptedUs = timestampUs;
  gate2Captured = true;
  velocity1Ms = segmentVelocity(GATE_1_TO_2_CM, timestampUs - gate1AcceptedUs);
  haveVelocity1 = true;
  telemetryDirty = true;
}

// Parking bay: the run is over, so work out the numbers and show the summary.
void acceptGate3(uint32_t timestampUs) {
  if (!gate2Captured ||
      !intervalWithin(gate2AcceptedUs, timestampUs, GATE_MAX_SEGMENT_US)) {
    return;
  }
  calculateMotion(timestampUs);
  updateRecords();
  reportRun();
  summaryStartedMs = millis();
  state = STATE_PARKED;
  startBuzzer(BUZZER_OFF);
  // Show the band actually achieved; a green light after an overshoot would
  // contradict the numbers printed right next to it.
  setRunwayLeds(haveVelocity2 ? bandForSpeed(velocity2Ms) : RUNWAY_TAXIING, true);
}

// Only ever one sensor listening at a time. Whichever gate the plane could
// reach next is the only one worth polling, and polling it alone means it gets
// every spare cycle instead of a third of them.
void updateGates() {
  uint32_t detectionUs = 0;

  if (state == STATE_SCANNING) {
    if (pollGate(0, &detectionUs)) {
      acceptGate1(detectionUs);
    }
    return;
  }
  if (state != STATE_ROLLING_OUT) {
    return;  // Nothing to watch for while the summary is up.
  }
  if (!gate2Captured) {
    if (pollGate(1, &detectionUs)) {
      acceptGate2(detectionUs);
    }
    return;
  }
  if (pollGate(2, &detectionUs)) {
    acceptGate3(detectionUs);
  }
}

void calculateMotion(uint32_t gate3Us) {
  const uint32_t segment1Us = gate2AcceptedUs - gate1AcceptedUs;
  const uint32_t segment2Us = gate3Us - gate2AcceptedUs;
  lastSegment2Us = segment2Us;
  if (segment1Us == 0UL || segment2Us == 0UL) {
    return;
  }

  velocity1Ms = segmentVelocity(GATE_1_TO_2_CM, segment1Us);
  velocity2Ms = segmentVelocity(GATE_2_TO_3_CM, segment2Us);
  haveVelocity1 = true;
  haveVelocity2 = true;

  // Each segment's average velocity belongs at its midpoint, not at either gate.
  const uint32_t midpointDeltaUs = (segment1Us / 2UL) + (segment2Us / 2UL);
  if (midpointDeltaUs > 0) {
    accelerationMs2 = (velocity2Ms - velocity1Ms) /
                      (midpointDeltaUs / 1000000.0f);
    haveAcceleration = true;
  }
}

void updateAirspace() {
  AirZone zone = AIR_NONE;
  if (radarRangeCm <= RANGE_TOO_CLOSE) {
    zone = AIR_INTRUSION;  // Unmeasurably close still means occupied airspace.
  } else if (radarRangeCm >= 0.0f) {
    if (radarRangeCm < AIR_GREEN_MIN_CM) {
      zone = AIR_INTRUSION;
    } else if (radarRangeCm <= AIR_GREEN_MAX_CM) {
      zone = AIR_OPTIMAL;
    } else {
      zone = AIR_APPROACH;
    }
  }
  setAirspaceLeds(zone);
  if (zone != previousAirZone) {
    previousAirZone = zone;
    if (zone == AIR_APPROACH) {
      startBuzzer(BUZZER_SLOW);
    } else if (zone == AIR_OPTIMAL) {
      startBuzzer(BUZZER_SINGLE);
    } else if (zone == AIR_INTRUSION) {
      startBuzzer(BUZZER_CONTINUOUS);
    } else {
      startBuzzer(BUZZER_OFF);
    }
  }
}

void updateRunwayFeedback() {
  if (!haveVelocity1) {
    return;
  }
  const float speed = haveVelocity2 ? velocity2Ms : velocity1Ms;
  const RunwayBand band = bandForSpeed(speed);
  if (band != previousRunwayBand) {
    previousRunwayBand = band;
    if (band == RUNWAY_SAFE) {
      startBuzzer(BUZZER_DOUBLE);
    } else if (band == RUNWAY_OVERSHOOT) {
      startBuzzer(BUZZER_RAPID);
    } else {
      startBuzzer(BUZZER_OFF);
    }
  }
  const bool flashOn = (millis() - flashStartedMs) < FLASH_INTERVAL_MS;
  if (millis() - flashStartedMs >= FLASH_INTERVAL_MS * 2UL) {
    flashStartedMs = millis();
  }
  setRunwayLeds(band, flashOn);
}

void drawRadar() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(F("A:"));
  display.print(radarAngle);
  display.print(F(" R:"));
  if (radarRangeCm <= RANGE_TOO_CLOSE) {
    display.print(F("<2cm"));
  } else if (radarRangeCm < 0.0f) {
    display.print(F("--"));
  } else {
    display.print(radarRangeCm, 0);
    display.print(F("cm"));
  }
  display.setCursor(0, 8);
  if (radarRangeCm == RANGE_NO_ECHO) {
    display.print(F("NO TARGET"));
  } else if (previousAirZone == AIR_APPROACH) {
    display.print(F("APPROACH DETECTED"));
  } else if (previousAirZone == AIR_OPTIMAL) {
    display.print(F("VECTOR OPTIMAL"));
  } else {
    display.print(F("AIRSPACE INTRUSION"));
  }

  display.drawLine(RADAR_CENTER_X - RADAR_RADIUS, RADAR_BASELINE_Y,
                   RADAR_CENTER_X + RADAR_RADIUS, RADAR_BASELINE_Y, SSD1306_WHITE);
  const uint8_t radarArcs[] = {RADAR_ARC_1, RADAR_ARC_2, RADAR_ARC_3};
  for (uint8_t arcIndex = 0; arcIndex < 3; ++arcIndex) {
    const uint8_t radius = radarArcs[arcIndex];
    for (uint8_t angle = 0; angle <= 180; angle += 5) {
      const float radians = angle * PI / 180.0f;
      const int16_t x = RADAR_CENTER_X + cos(radians) * radius;
      const int16_t y = RADAR_BASELINE_Y - sin(radians) * radius;
      display.drawPixel(x, y, SSD1306_WHITE);
    }
  }
  const float sweepRadians = radarAngle * PI / 180.0f;
  display.drawLine(RADAR_CENTER_X, RADAR_BASELINE_Y,
                   RADAR_CENTER_X + cos(sweepRadians) * RADAR_RADIUS,
                   RADAR_BASELINE_Y - sin(sweepRadians) * RADAR_RADIUS,
                   SSD1306_WHITE);
  if (radarRangeCm >= SONAR_MIN_CM && radarRangeCm <= SONAR_MAX_CM) {
    // Scaled to RADAR_DISPLAY_MAX_CM, not SONAR_MAX_CM: against the 200 cm limit
    // the whole 10-25 cm working band collapsed into 3 pixels.
    const uint8_t blipRadius = min((uint8_t)RADAR_RADIUS,
                                   (uint8_t)(radarRangeCm * RADAR_RADIUS / RADAR_DISPLAY_MAX_CM));
    const int16_t x = RADAR_CENTER_X + cos(sweepRadians) * blipRadius;
    const int16_t y = RADAR_BASELINE_Y - sin(sweepRadians) * blipRadius;
    display.fillCircle(x, y, 2, SSD1306_WHITE);
  }
  display.display();
}

void formatFloat(float value, char *buffer, uint8_t width = 6) {
  dtostrf(value, width, 2, buffer);
}

void drawTelemetry() {
  char velocityText[10];
  char accelerationText[10];
  const float currentSpeed = haveVelocity2 ? velocity2Ms : velocity1Ms;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(F("ROLLOUT TELEMETRY"));
  display.setCursor(0, 16);
  display.print(F("V1: "));
  if (haveVelocity1) {
    formatFloat(velocity1Ms, velocityText);
    display.print(velocityText);
    display.print(F(" m/s"));
  } else {
    display.print(F("--"));
  }
  display.setCursor(0, 28);
  display.print(F("V2: "));
  if (haveVelocity2) {
    formatFloat(velocity2Ms, velocityText);
    display.print(velocityText);
    display.print(F(" m/s"));
  } else {
    display.print(F("--"));
  }
  display.setCursor(0, 40);
  display.print(F("ACC: "));
  if (haveAcceleration) {
    formatFloat(accelerationMs2, accelerationText);
    display.print(accelerationText);
    display.print(F(" m/s2"));
  } else {
    display.print(F("--"));
  }
  display.setCursor(0, 54);
  if (haveVelocity1 && currentSpeed >= RUNWAY_GREEN_MAX_MS) {
    display.print(F("WARNING: OVERSHOOT"));
  } else if (haveVelocity1 && currentSpeed >= RUNWAY_GREEN_MIN_MS) {
    display.print(F("SAFE ROLLOUT"));
  } else {
    display.print(F("TAXIING"));
  }
  display.display();
}

void drawSummary() {
  char valueText[10];
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(F("SUMMARY  RUN "));
  display.print(records.completedRuns);
  display.setCursor(0, 12);
  display.print(F("V1: "));
  if (haveVelocity1) {
    formatFloat(velocity1Ms, valueText);
    display.print(valueText);
    display.print(F(" m/s"));
  } else {
    display.print(F("--"));
  }
  display.setCursor(0, 23);
  display.print(F("V2: "));
  if (haveVelocity2) {
    formatFloat(velocity2Ms, valueText);
    display.print(valueText);
    display.print(F(" m/s"));
  } else {
    display.print(F("--"));
  }
  display.setCursor(0, 34);
  display.print(F("ACC: "));
  if (haveAcceleration) {
    formatFloat(accelerationMs2, valueText);
    display.print(valueText);
    display.print(F(" m/s2"));
  } else {
    display.print(F("--"));
  }
  display.setCursor(0, 46);
  display.print(F("BEST V: "));
  formatFloat(records.fastestVelocity, valueText);
  display.print(valueText);
  display.print(F(" m/s"));
  display.setCursor(0, 57);
  display.print(F("MAX A: "));
  formatFloat(records.highestAcceleration, valueText);
  display.print(valueText);
  display.print(F(" m/s2"));
  display.display();
}

void updateRadar() {
  if (millis() - lastRadarUpdateMs < RADAR_UPDATE_MS) {
    return;
  }
  lastRadarUpdateMs = millis();
  radarServo.write(radarAngle);
  radarRangeCm = readRangeCm();
  updateAirspace();
  // Step and reverse in one go; the previous form burned a tick at each end.
  int16_t nextAngle = (int16_t)radarAngle + (int16_t)radarDirection * (int16_t)RADAR_STEP_DEG;
  if (nextAngle >= 180) {
    nextAngle = 180;
    radarDirection = -1;
  } else if (nextAngle <= 0) {
    nextAngle = 0;
    radarDirection = 1;
  }
  radarAngle = (uint8_t)nextAngle;
}

void updateDisplay() {
  if (!displayReady || millis() - lastDisplayUpdateMs < DISPLAY_UPDATE_MS) {
    return;
  }
  if (state == STATE_ROLLING_OUT) {
    // A framebuffer push blocks for ~23 ms at 400 kHz. Gate 3 is polled, so that
    // would be charged straight to segment 2 -- redraw only on a real change.
    if (!telemetryDirty) {
      return;
    }
    telemetryDirty = false;
  }
  lastDisplayUpdateMs = millis();
  if (state == STATE_SCANNING) {
    drawRadar();
  } else if (state == STATE_ROLLING_OUT) {
    drawTelemetry();
  } else {
    drawSummary();
  }
}

void setup() {
#if ENABLE_TELEMETRY
  Serial.begin(115200);
#endif
  for (uint8_t index = 0; index < GATE_COUNT; index++) {
    pinMode(gates[index].trigPin, OUTPUT);
    digitalWrite(gates[index].trigPin, LOW);
    pinMode(gates[index].echoPin, INPUT);
  }
  pinMode(PIN_BUZZER, OUTPUT);
#if HAVE_AIRSPACE_RADAR
  pinMode(PIN_SONAR_TRIGGER, OUTPUT);
  pinMode(PIN_SONAR_ECHO, INPUT);
#endif
  pinMode(PIN_AIR_YELLOW, OUTPUT);
  pinMode(PIN_AIR_GREEN, OUTPUT);
  pinMode(PIN_AIR_RED, OUTPUT);
  pinMode(PIN_RUNWAY_YELLOW, OUTPUT);
  pinMode(PIN_RUNWAY_GREEN, OUTPUT);
  pinMode(PIN_RUNWAY_RED, OUTPUT);

  Wire.begin();
  Wire.setClock(400000UL);
  // A missing panel must not take the monitor down with it: the LEDs and buzzer
  // are the safety-critical outputs and stay live either way.
  displayReady = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
  if (displayReady) {
    display.clearDisplay();
    display.display();
  }
#if HAVE_AIRSPACE_RADAR
  radarServo.attach(PIN_SERVO);
  radarServo.write(0);
#endif
  loadRecords();
  resetToScanning();
#if ENABLE_TELEMETRY
  Serial.println(F("# airspace runway monitor ready"));
  Serial.println(F("# run,seg1_us,seg2_us,v1_ms,v2_ms,accel_ms2"));
#endif
}

void loop() {
  // Watch whichever gate is next. This is the measurement, so it runs first and
  // nothing below it is allowed to take long.
  updateGates();

  if (state == STATE_SCANNING) {
#if HAVE_AIRSPACE_RADAR
    updateRadar();
#endif
  } else if (state == STATE_ROLLING_OUT) {
    updateRunwayFeedback();
    if (millis() - rolloutStartedMs >= ROLLOUT_TIMEOUT_MS) {
      resetToScanning();  // Plane never made it to the end; start watching again.
    }
  } else if (millis() - summaryStartedMs >= SUMMARY_HOLD_MS) {
    resetToScanning();
  }

  updateBuzzer();
  updateDisplay();
}
