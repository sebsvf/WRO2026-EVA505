#include "encoder.h"
#include "../config.h"

static volatile long _ticks = 0;
static long _last_ticks_for_rpm = 0;

static void IRAM_ATTR _onEncoderA() {
  // Quadrature decoding: direction is determined by comparing the
  // edge on A against the current level of B.
  bool b_level = digitalRead(ENCODER_B_PIN);
  if (b_level) {
    _ticks++;
  } else {
    _ticks--;
  }
}

void encoder_init() {
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), _onEncoderA, RISING);
}

long encoder_get_ticks() {
  noInterrupts();
  long snapshot = _ticks;
  interrupts();
  return snapshot;
}

void encoder_reset() {
  noInterrupts();
  _ticks = 0;
  _last_ticks_for_rpm = 0;
  interrupts();
}

float encoder_get_rpm(unsigned long dt_ms) {
  long current = encoder_get_ticks();
  long delta = current - _last_ticks_for_rpm;
  _last_ticks_for_rpm = current;

  if (dt_ms == 0) return 0.0f;

  float revs = (float)delta / (float)ENCODER_TICKS_PER_REV;
  float minutes = (dt_ms / 1000.0f) / 60.0f;
  return revs / minutes;
}
