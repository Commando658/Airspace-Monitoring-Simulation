#pragma once
#include "Arduino.h"
struct EEPROMClass {
  uint8_t cell[1024];
  EEPROMClass(){ memset(cell,0xFF,sizeof cell); }
  template<class T> T& get(int a,T&v){ memcpy(&v,cell+a,sizeof(T)); return v; }
  template<class T> const T& put(int a,const T&v){ memcpy(cell+a,&v,sizeof(T)); return v; }
};
extern EEPROMClass EEPROM;
