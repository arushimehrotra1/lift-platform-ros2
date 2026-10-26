#pragma once
#include <Arduino.h>
#include <vector>
struct Preferences {
  static std::vector<uint8_t>& store(){ static std::vector<uint8_t> v; return v; }
  bool begin(const char*,bool=false){ return true; } void end(){}
  size_t getBytes(const char*,void*d,size_t n){ auto&v=store(); if(v.size()!=n) return 0; memcpy(d,v.data(),n); return n; }
  size_t putBytes(const char*,const void*s,size_t n){ auto&v=store(); v.assign((const uint8_t*)s,(const uint8_t*)s+n); return n; }
};
