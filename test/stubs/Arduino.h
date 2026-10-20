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
  unsigned long echoUs = 0;          // radar echo (legacy single-sensor path)
  unsigned long echoUsPin[24] = {0};  // per-echo-pin round trip; 0 = nothing there
  bool echoPinSet[24] = {false};      // true once a test owns that pin
  int toneHz = -1;               // -1 = silent
  int toneCalls = 0;
  int rawToneCalls = 0;
  int noToneCalls = 0;
  int servoAngle = -1;
  std::vector<std::string> screen;
  std::vector<std::string> serial;
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
// Each echo pin answers for its own sensor. A pin a test has not claimed falls
// back to sim.echoUs, which is how the single radar sensor is driven -- but a
// claimed pin reporting 0 means "nothing there", NOT "ask the radar".
inline uint32_t pulseIn(uint8_t echoPin,uint8_t,uint32_t timeout){
  unsigned long d = sim.echoPinSet[echoPin] ? sim.echoUsPin[echoPin] : sim.echoUs;
  if(d==0||d>timeout){ sim.nowUs+=timeout; return 0; }
  sim.nowUs+=d; return (uint32_t)d;
}
inline void tone(uint8_t,unsigned int hz){ sim.rawToneCalls++; if(sim.toneHz!=(int)hz) sim.toneCalls++; sim.toneHz=hz; }
inline void noTone(uint8_t){ if(sim.toneHz!=-1) sim.noToneCalls++; sim.toneHz=-1; }
inline uint8_t digitalPinToInterrupt(uint8_t p){ return p; }
inline void attachInterrupt(uint8_t,void(*)(),int){}
inline void noInterrupts(){}
inline void interrupts(){}
// Serial stub. Telemetry lines are captured so a test can assert on them
// instead of them vanishing into a port that does not exist here.
struct SerialStub {
  std::string line;
  void begin(unsigned long){}
  void print(const char*s){ line+=s; }
  void print(char c){ line+=c; }
  void print(int v){ char b[16]; snprintf(b,sizeof b,"%d",v); line+=b; }
  void print(unsigned v){ char b[16]; snprintf(b,sizeof b,"%u",v); line+=b; }
  void print(unsigned long v){ char b[24]; snprintf(b,sizeof b,"%lu",v); line+=b; }
  void print(float v,int d){ char b[32],f[8]; snprintf(f,sizeof f,"%%.%df",d); snprintf(b,sizeof b,f,v); line+=b; }
  void println(const char*s){ line+=s; flushLine(); }
  void println(float v,int d){ print(v,d); flushLine(); }
  void flushLine(){ sim.serial.push_back(line); line.clear(); }
};
extern SerialStub Serial;

inline char* dtostrf(double v,signed char w,unsigned char p,char*s){
  char fmt[16]; snprintf(fmt,sizeof fmt,"%%%d.%df",(int)w,(int)p);
  snprintf(s,32,fmt,v); return s;
}
