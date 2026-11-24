#include "Encoder.h"
EncoderSnapshot encA = {}, encB = {};
uint32_t encoderSampleMs=0;
struct Counter {
  volatile uint32_t forward, reverse, edges, invalid, lastMs;
  volatile uint8_t previous;
};
static Counter a = {}, b = {};
static uint32_t sampledAt;
static uint8_t levels(uint8_t x, uint8_t y) { return (digitalRead(x) << 1) | digitalRead(y); }
static void edge(Counter &c, uint8_t x, uint8_t y, int8_t sign) {
  static const int8_t table[16] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  uint8_t next = levels(x,y), old = c.previous;
  int8_t step = table[(old << 2) | next] * sign;
  if ((old ^ next) == 3) ++c.invalid;
  if (next != old) { ++c.edges; c.lastMs = millis(); }
  if (step > 0) ++c.forward;
  if (step < 0) ++c.reverse;
  c.previous = next;
}
static void onA() { edge(a, ENC_A_A, ENC_A_B, ENC_A_SIGN); }
static void onB() { edge(b, ENC_B_A, ENC_B_B, ENC_B_SIGN); }
void encoderBegin() {
  if (!ENCODERS_ENABLED) return;
  pinMode(ENC_A_A, INPUT_PULLUP); pinMode(ENC_A_B, INPUT_PULLUP);
  pinMode(ENC_B_A, INPUT_PULLUP); pinMode(ENC_B_B, INPUT_PULLUP);
  a.previous = levels(ENC_A_A, ENC_A_B); b.previous = levels(ENC_B_A, ENC_B_B);
  attachInterrupt(digitalPinToInterrupt(ENC_A_A), onA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A_B), onA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_A), onB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_B), onB, CHANGE);
}
static void snapshot(Counter &c, EncoderSnapshot &s, uint32_t now) {
#if defined(BMDUINO_ARCH_HT32) || defined(ARDUINO_ARCH_HT32)
  // HT32 core's noInterrupts() disables IRQs AND flushes every serial port.
  // Draining queued UART TX with IRQs off can lose incoming MOVE frames.
  // Only mask IRQs for this short volatile copy, preserving the prior mask.
  uint32_t irqMask = __get_PRIMASK();
  __disable_irq();
#else
  noInterrupts();
#endif
  uint32_t f=c.forward, r=c.reverse, e=c.edges, bad=c.invalid, last=c.lastMs;
  uint8_t state=c.previous;
#if defined(BMDUINO_ARCH_HT32) || defined(ARDUINO_ARCH_HT32)
  __set_PRIMASK(irqMask);
#else
  interrupts();
#endif
  s.delta = (int32_t)(f-s.forward) - (int32_t)(r-s.reverse);
  s.forward=f; s.reverse=r; s.edges=e; s.invalid=bad;
  s.seen = e != 0; s.lastEdgeAge = now-last; s.levels=state;
}
void encoderTick() {
  uint32_t now=millis();
  if (now-sampledAt < TELEMETRY_MS) return;
  sampledAt=now;
  encoderSampleMs=now;
  snapshot(a,encA,now); snapshot(b,encB,now);
}
