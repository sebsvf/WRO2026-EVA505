#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

// Interrupt-driven quadrature-style pulse counter for the Pololu DC
// gearmotor's integrated encoder. Only channel A is used for tick
// counting (single-channel counting); channel B is read at the same
// time to determine direction, per the standard quadrature decoding
// trick of comparing A's edge against B's level.

void encoder_init();
long encoder_get_ticks();
void encoder_reset();
float encoder_get_rpm(unsigned long dt_ms);

#endif  // ENCODER_H
