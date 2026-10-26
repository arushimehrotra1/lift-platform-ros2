// Minimal Arduino/ESP32 stub: lets the REAL firmware run on a PC against fw_sim.cpp's plant.
#pragma once
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <string>
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define constrain(x,a,b) ((x)<(a)?(a):((x)>(b)?(b):(x)))
extern uint32_t g_us;                       // simulated clock
inline uint32_t micros(){ return g_us; }
inline uint32_t millis(){ return g_us/1000; }
inline void pinMode(int,int){}
inline void analogReadResolution(int){}
inline bool ledcAttach(int,int,int){ return true; }
int  analogRead(int pin);                   // provided by fw_sim.cpp
int  digitalRead(int pin);
void digitalWrite(int pin,int v);
void ledcWrite(int pin,uint32_t duty);
struct SerialStub {
  std::string rx, tx;
  void begin(int){}
  int available(){ return (int)rx.size(); }
  int read(){ if(rx.empty()) return -1; int c=(unsigned char)rx[0]; rx.erase(0,1); return c; }
  void println(const char*s){ tx+=s; tx+='\n'; }
};
extern SerialStub Serial;
