#pragma once
#include "Arduino.h"
struct Servo { void attach(uint8_t){} void write(int a){ sim.servoAngle=a; } };
