/*
  Airspace & Runway Monitoring System
  Target: Arduino Nano/UNO (ATmega328P)

  Pin map:
    D2  IR gate 1 / touchdown (INT0)   D9  airspace yellow LED
    D3  IR gate 2 / speed gate (INT1)  D10 airspace green LED
    D4  IR gate 3 / parking bay        D11 airspace red LED
    D5  active buzzer                  D12 runway yellow LED
    D6  HC-SR04 trigger                A0  runway green LED
    D7  HC-SR04 echo                   A1  runway red LED
    D8  servo signal                   A4/A5 OLED SDA/SCL

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

    Gates 1 and 2 are interrupt driven. Gate 3 is polled, so the main loop must
    never block while a rollout is in progress -- pushing the 1 KB framebuffer
    over I2C takes ~23 ms, which would land directly in the measurement. The
    telemetry screen is therefore redrawn only when its contents change.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Hardware pins.
const uint8_t PIN_GATE_1 = 2;
const uint8_t PIN_GATE_2 = 3;
const uint8_t PIN_GATE_3 = 4;
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

// Change these after measuring the track.
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

// Runway velocity thresholds in metres per second.
const float RUNWAY_GREEN_MIN_MS = 0.15f; // Lower edge of safe rollout.
const float RUNWAY_GREEN_MAX_MS = 0.40f; // Upper edge of safe rollout.

// Timing and filtering values.
const uint32_t SONAR_TIMEOUT_US = 12000UL; // pulseIn timeout for a missing echo.
const uint32_t GATE_LOCKOUT_US = 80000UL;  // Ignore repeat beam breaks.
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

// ISR-owned values are volatile. The main loop copies multi-byte values atomically.
volatile uint32_t gate1TimestampUs = 0;
volatile uint32_t gate2TimestampUs = 0;
volatile bool gate1Pending = false;
volatile bool gate2Pending = false;

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

// Stable copies of the accepted gate edges. All velocity maths reads these, never
// the volatile ISR variables -- a second beam break (wing, tail, vibration)
// overwrites the volatile value even when the event itself is discarded.
uint32_t gate1AcceptedUs = 0;
uint32_t gate2AcceptedUs = 0;
bool gate1Captured = false;
bool gate2Captured = false;

// Gate 1 repeat-break lockout. Kept outside clearMotionData() so that resetting a
// run does not also disarm the debounce. A separate flag is used because 0 is a
// legal micros() value just after the wrap.
uint32_t gate1LockoutUs = 0;
bool gate1LockoutArmed = false;

bool displayReady = false;
bool telemetryDirty = false;
bool gate3PreviousActive = false;
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
void clearMotionData(bool clearGate2Event);

void gate1ISR() {
  gate1TimestampUs = micros();
  gate1Pending = true;
}

void gate2ISR() {
  gate2TimestampUs = micros();
  gate2Pending = true;
}

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

float readRangeCm() {
  digitalWrite(PIN_SONAR_TRIGGER, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_SONAR_TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_SONAR_TRIGGER, LOW);

  const uint32_t durationUs = pulseIn(PIN_SONAR_ECHO, HIGH, SONAR_TIMEOUT_US);
  if (durationUs == 0) {
    return RANGE_NO_ECHO;
  }

  const float distanceCm = durationUs * 0.0343f / 2.0f;
  if (distanceCm < SONAR_MIN_CM) {
    return RANGE_TOO_CLOSE;  // Something is there, just nearer than the sensor can measure.
  }
  if (distanceCm > SONAR_MAX_CM) {
    return RANGE_NO_ECHO;
  }
  return distanceCm;
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

void clearMotionData(bool clearGate2Event) {
  noInterrupts();
  gate1Pending = false;
  if (clearGate2Event) {
    gate2Pending = false;
  }
  interrupts();
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
  // Seed the gate 3 edge detector from the live pin so a beam that is already
  // blocked (plane still sitting in the bay) cannot register as a fresh trigger.
  gate3PreviousActive = digitalRead(PIN_GATE_3) == LOW;
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

void resetToScanning() {
  state = STATE_SCANNING;
  previousAirZone = AIR_NONE;
  radarRangeCm = RANGE_NO_ECHO;  // Otherwise the first frame redraws the last run's range.
  previousRunwayBand = RUNWAY_TAXIING;
  setAirspaceLeds(AIR_NONE);
  setRunwayLeds(RUNWAY_TAXIING, false);
  startBuzzer(BUZZER_OFF);
  clearMotionData(true);
}

void acceptGate1(uint32_t timestampUs) {
  if (gate1LockoutArmed &&
      (uint32_t)(timestampUs - gate1LockoutUs) < GATE_LOCKOUT_US) {
    return;
  }
  clearMotionData(false);  // Clears the snapshots, so record the edge afterwards.
  gate1AcceptedUs = timestampUs;
  gate1Captured = true;
  gate1LockoutUs = timestampUs;
  gate1LockoutArmed = true;
  rolloutStartedMs = millis();
  state = STATE_ROLLING_OUT;
  telemetryDirty = true;
}

void processGateEvents() {
  bool gate1Event = false;
  uint32_t gate1Us = 0;

  noInterrupts();
  if (gate1Pending) {
    gate1Pending = false;
    gate1Us = gate1TimestampUs;
    gate1Event = true;
  }
  interrupts();

  if (state != STATE_SCANNING) {
    return;
  }
  if (gate1Event) {
    acceptGate1(gate1Us);
  }
}

void calculateMotion(uint32_t gate3Us) {
  const uint32_t segment1Us = gate2AcceptedUs - gate1AcceptedUs;
  const uint32_t segment2Us = gate3Us - gate2AcceptedUs;
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

void processGate2Timestamp() {
  uint32_t gate2Us = 0;
  bool pending = false;
  noInterrupts();
  if (gate2Pending) {
    gate2Pending = false;
    gate2Us = gate2TimestampUs;
    pending = true;
  }
  interrupts();

  if (!pending || state != STATE_ROLLING_OUT || !gate1Captured) {
    return;
  }
  if (gate2Captured &&
      (uint32_t)(gate2Us - gate2AcceptedUs) < GATE_LOCKOUT_US) {
    return;  // Repeat break of the same beam.
  }
  if (!intervalWithin(gate1AcceptedUs, gate2Us, GATE_MAX_SEGMENT_US)) {
    return;
  }

  gate2AcceptedUs = gate2Us;
  gate2Captured = true;
  velocity1Ms = segmentVelocity(GATE_1_TO_2_CM, gate2Us - gate1AcceptedUs);
  haveVelocity1 = true;
  telemetryDirty = true;
}

// Polled every loop iteration rather than on a timer: the quantisation here lands
// directly in segment 2, so the only acceptable latency is the loop period.
void pollGate3() {
  if (state != STATE_ROLLING_OUT) {
    return;
  }
  const bool active = digitalRead(PIN_GATE_3) == LOW;
  const uint32_t gate3Us = micros();
  const bool triggered = active && !gate3PreviousActive;
  gate3PreviousActive = active;
  if (!triggered || !gate2Captured) {
    return;
  }
  if (!intervalWithin(gate2AcceptedUs, gate3Us, GATE_MAX_SEGMENT_US)) {
    return;
  }

  calculateMotion(gate3Us);
  updateRecords();
  summaryStartedMs = millis();
  state = STATE_PARKED;
  startBuzzer(BUZZER_OFF);
  // Report the band actually achieved; lighting green after an overshoot would
  // contradict the numbers on the summary screen.
  setRunwayLeds(haveVelocity2 ? bandForSpeed(velocity2Ms) : RUNWAY_TAXIING, true);
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
  pinMode(PIN_GATE_1, INPUT_PULLUP);
  pinMode(PIN_GATE_2, INPUT_PULLUP);
  pinMode(PIN_GATE_3, INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_SONAR_TRIGGER, OUTPUT);
  pinMode(PIN_SONAR_ECHO, INPUT);
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
  radarServo.attach(PIN_SERVO);
  radarServo.write(0);
  loadRecords();
  attachInterrupt(digitalPinToInterrupt(PIN_GATE_1), gate1ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_GATE_2), gate2ISR, FALLING);
  resetToScanning();
}

void loop() {
  processGateEvents();
  processGate2Timestamp();

  if (state == STATE_SCANNING) {
    updateRadar();
  } else if (state == STATE_ROLLING_OUT) {
    updateRunwayFeedback();
    pollGate3();
    if (millis() - rolloutStartedMs >= ROLLOUT_TIMEOUT_MS) {
      resetToScanning();
    }
  } else if (millis() - summaryStartedMs >= SUMMARY_HOLD_MS) {
    resetToScanning();
  }

  updateBuzzer();
  updateDisplay();
}
