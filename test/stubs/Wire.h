#pragma once
#include "Arduino.h"
struct TwoWire { void begin(){} void setClock(unsigned long){} };
extern TwoWire Wire;
