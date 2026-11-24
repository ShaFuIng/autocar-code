#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <deque>
#include <string>
#include <algorithm>
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define CHANGE 3
#define BMDUINO_ARCH_HT32 1
extern uint32_t fakeMs;
extern uint32_t fakeIrqMask, fakeSerialFlushes;
extern int pinLevels[64], pwmLevels[64];
extern void (*callbacks[64])();
inline uint32_t millis(){return fakeMs;}
inline uint32_t micros(){return fakeMs*1000;}
inline void delay(uint32_t n){fakeMs+=n;}
inline void pinMode(int,int){}
inline int digitalRead(int pin){return pinLevels[pin];}
inline void digitalWrite(int pin,int value){pinLevels[pin]=value;}
inline void analogWrite(int pin,int value){pwmLevels[pin]=value;}
inline uint32_t __get_PRIMASK(){return fakeIrqMask;}
inline void __disable_irq(){fakeIrqMask=1;}
inline void __set_PRIMASK(uint32_t mask){fakeIrqMask=mask;}
// Model the HT32-specific side effect instead of an empty Arduino stub.
inline void noInterrupts(){fakeIrqMask=1;++fakeSerialFlushes;fakeMs+=100;}
inline void interrupts(){fakeIrqMask=0;}
inline int digitalPinToInterrupt(int pin){return pin;}
inline void attachInterrupt(int pin,void(*fn)(),int){callbacks[pin]=fn;}
template<class T> T constrain(T x,T low,T high){return std::max(low,std::min(x,high));}
class Stream {
 public:
  std::deque<uint8_t> rx;
  std::string tx;
  int capacity=128;
  int available(){return (int)rx.size();}
  int read(){if(rx.empty())return -1;int c=rx.front();rx.pop_front();return c;}
  int availableForWrite(){return capacity;}
  size_t write(uint8_t c){tx.push_back(c);return 1;}
  void begin(int){}
  explicit operator bool()const{return true;}
  void feed(const std::string &s){for(unsigned char c:s)rx.push_back(c);}
};
extern Stream Serial,Serial1,Serial3;
