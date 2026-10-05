// Minimal Arduino stubs so the sketch can be compiled and driven on the host.
#pragma once
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define FALLING 2
#define PI 3.1415926535897932384626433832795

static const uint8_t A0=14,A1=15,A2=16,A3=17,A4=18,A5=19;

#define F(x) (x)
template <class T> T mmin(T a, T b){ return a<b?a:b; }
#define min(a,b) mmin(a,b)

// ---- simulated world -------------------------------------------------------
struct Sim {
  unsigned long nowUs = 0;
  uint8_t pinMode_[24] = {0};
  int pinState[24];
  unsigned long echoUs = 0;      // what pulseIn should report
  int toneHz = -1;               // -1 = silent
  int toneCalls = 0;
  int rawToneCalls = 0;
  int noToneCalls = 0;
  int servoAngle = -1;
  std::vector<std::string> screen;
  Sim(){ for(int i=0;i<24;i++) pinState[i]=HIGH; }
};
extern Sim sim;

inline void pinMode(uint8_t p,uint8_t m){ sim.pinMode_[p]=m; }
inline void digitalWrite(uint8_t p,uint8_t v){ sim.pinState[p]=v; }
inline int  digitalRead(uint8_t p){ return sim.pinState[p]; }
// 32-bit, exactly as on AVR, so wrap behaviour is reproduced rather than hidden
// behind a 64-bit host long. sim.nowUs is the wider master clock behind them.
inline uint32_t micros(){ return (uint32_t)sim.nowUs; }
inline uint32_t millis(){ return (uint32_t)(sim.nowUs/1000UL); }
inline void delayMicroseconds(unsigned int us){ sim.nowUs+=us; }
inline uint32_t pulseIn(uint8_t,uint8_t,uint32_t timeout){
  if(sim.echoUs==0||sim.echoUs>timeout){ sim.nowUs+=timeout; return 0; }
  sim.nowUs+=sim.echoUs; return sim.echoUs;
}
inline void tone(uint8_t,unsigned int hz){ sim.rawToneCalls++; if(sim.toneHz!=(int)hz) sim.toneCalls++; sim.toneHz=hz; }
inline void noTone(uint8_t){ if(sim.toneHz!=-1) sim.noToneCalls++; sim.toneHz=-1; }
inline uint8_t digitalPinToInterrupt(uint8_t p){ return p; }
inline void attachInterrupt(uint8_t,void(*)(),int){}
inline void noInterrupts(){}
inline void interrupts(){}
inline char* dtostrf(double v,signed char w,unsigned char p,char*s){
  char fmt[16]; snprintf(fmt,sizeof fmt,"%%%d.%df",(int)w,(int)p);
  snprintf(s,32,fmt,v); return s;
}
