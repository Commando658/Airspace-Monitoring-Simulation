#pragma once
#include "Arduino.h"
#define SSD1306_WHITE 1
#define SSD1306_SWITCHCAPVCC 2
struct Adafruit_SSD1306 {
  std::string line; bool ok=true;
  Adafruit_SSD1306(int,int,TwoWire*,int){}
  bool begin(int,int){ return ok; }
  void clearDisplay(){ sim.screen.clear(); line.clear(); }
  void setTextSize(int){} void setTextColor(int){}
  void setCursor(int,int){ if(!line.empty()){sim.screen.push_back(line);line.clear();} }
  void print(const char*s){ line+=s; }
  void print(int v){ char b[16]; snprintf(b,sizeof b,"%d",v); line+=b; }
  void print(unsigned v){ char b[16]; snprintf(b,sizeof b,"%u",v); line+=b; }
  void print(unsigned long v){ char b[24]; snprintf(b,sizeof b,"%lu",v); line+=b; }
  void print(float v,int d){ char b[32],f[8]; snprintf(f,sizeof f,"%%.%df",d); snprintf(b,sizeof b,f,v); line+=b; }
  void drawLine(int,int,int,int,int){} void drawPixel(int,int,int){}
  void fillCircle(int,int,int,int){} void drawCircle(int,int,int,int){}
  void display(){ if(!line.empty()){sim.screen.push_back(line);line.clear();} }
};
