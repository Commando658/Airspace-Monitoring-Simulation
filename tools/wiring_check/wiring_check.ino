/*
  Wiring check for the Airspace & Runway Monitor.

  Upload this on its own, open the Serial Monitor at 115200, and it will tell
  you which pins your ultrasonic sensors are actually plugged into. You do not
  need to know anything about the wiring beforehand.

  It works by trying every sensible pair of pins in turn: it sends a pulse out
  of one and listens on the other. A real HC-SR04 answers with a steady
  distance. A pin with nothing on it either stays silent or returns noise, so
  the sketch takes several readings of each pair and only believes a pair that
  agrees with itself.

  Put something solid about 10-20 cm in front of the sensors before you run it,
  so there is a clear echo to find.

  When it finishes, copy the whole output and send it back. The numbers go
  straight into the CONFIGURATION block of airspace_runway_monitor.ino.
*/

#include <Arduino.h>

// Pins worth testing. A4 and A5 are left out because the display uses them,
// and D0/D1 are left out because they are the USB serial connection.
const uint8_t CANDIDATE_PINS[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
                                  A0, A1, A2, A3};
const uint8_t CANDIDATE_COUNT = sizeof(CANDIDATE_PINS) / sizeof(CANDIDATE_PINS[0]);

const uint8_t SAMPLES_PER_PAIR = 5;     // Readings taken before believing a pair.
const uint8_t AGREEING_NEEDED = 3;      // How many must agree.
const float AGREEMENT_CM = 3.0f;        // How close "agree" means.
const unsigned long LISTEN_TIMEOUT_US = 25000UL;  // ~4 m of listening.
const unsigned long SETTLE_MS = 60UL;   // Let the last echo die away.

float pingOnce(uint8_t trigPin, uint8_t echoPin) {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(4);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  const unsigned long echoUs = pulseIn(echoPin, HIGH, LISTEN_TIMEOUT_US);
  if (echoUs == 0) {
    return -1.0f;
  }
  return echoUs * 0.0343f / 2.0f;
}

// Takes several readings and returns the distance only if enough of them agree.
// This is what separates a real sensor from a floating pin picking up noise.
float measurePair(uint8_t trigPin, uint8_t echoPin) {
  float readings[SAMPLES_PER_PAIR];
  uint8_t valid = 0;

  for (uint8_t i = 0; i < SAMPLES_PER_PAIR; i++) {
    const float cm = pingOnce(trigPin, echoPin);
    if (cm > 0.0f) {
      readings[valid++] = cm;
    }
    delay(SETTLE_MS);
  }
  if (valid < AGREEING_NEEDED) {
    return -1.0f;
  }

  // Find a reading that enough of the others sit close to.
  for (uint8_t i = 0; i < valid; i++) {
    uint8_t agreeing = 0;
    float total = 0.0f;
    for (uint8_t j = 0; j < valid; j++) {
      if (fabs(readings[j] - readings[i]) <= AGREEMENT_CM) {
        agreeing++;
        total += readings[j];
      }
    }
    if (agreeing >= AGREEING_NEEDED) {
      return total / agreeing;
    }
  }
  return -1.0f;
}

const char *pinName(uint8_t pin) {
  static char buffer[5];
  if (pin >= A0) {
    buffer[0] = 'A';
    buffer[1] = (char)('0' + (pin - A0));
    buffer[2] = '\0';
  } else {
    buffer[0] = 'D';
    if (pin >= 10) {
      buffer[1] = (char)('0' + (pin / 10));
      buffer[2] = (char)('0' + (pin % 10));
      buffer[3] = '\0';
    } else {
      buffer[1] = (char)('0' + pin);
      buffer[2] = '\0';
    }
  }
  return buffer;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;  // Wait for the USB port on boards that need it.
  }

  Serial.println();
  Serial.println(F("Ultrasonic wiring check"));
  Serial.println(F("======================="));
  Serial.println(F("Put something solid 10-20cm in front of each sensor."));
  Serial.print(F("Testing "));
  Serial.print((int)CANDIDATE_COUNT * ((int)CANDIDATE_COUNT - 1));
  Serial.println(F(" pin pairs. This takes a couple of minutes."));
  Serial.println();
  Serial.println(F("TRIG  ECHO  DISTANCE"));
  Serial.println(F("--------------------"));

  uint8_t found = 0;
  for (uint8_t t = 0; t < CANDIDATE_COUNT; t++) {
    for (uint8_t e = 0; e < CANDIDATE_COUNT; e++) {
      if (t == e) {
        continue;
      }
      const float cm = measurePair(CANDIDATE_PINS[t], CANDIDATE_PINS[e]);
      if (cm > 0.0f) {
        found++;
        Serial.print(pinName(CANDIDATE_PINS[t]));
        Serial.print(F("    "));
        Serial.print(pinName(CANDIDATE_PINS[e]));
        Serial.print(F("    "));
        Serial.print(cm, 1);
        Serial.println(F(" cm"));
      }
    }
  }

  Serial.println(F("--------------------"));
  Serial.print(F("Sensors found: "));
  Serial.println(found);
  Serial.println();
  if (found == 0) {
    Serial.println(F("Nothing answered. Check 5V and GND on the sensors, and"));
    Serial.println(F("that something is actually in front of them."));
  } else {
    Serial.println(F("Copy everything above into the chat, and these pins go"));
    Serial.println(F("into the CONFIGURATION block of the main sketch."));
    Serial.println(F("If a sensor shows up on more than one pair, the lowest"));
    Serial.println(F("pin numbers are usually the real wiring."));
  }
}

void loop() {
  // Nothing to do; the check runs once at startup.
}
