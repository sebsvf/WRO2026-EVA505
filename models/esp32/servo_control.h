#include "motor_control.h"
#include "../config.h"
#include "../pwm_compat.h"
#include <math.h>
static float target = 0, applied = 0;
static uint32_t last_update = 0, zero_since = 0;
static int last_direction = 0;
static bool initialized = false;

static void write_power(float power) {
  const int duty = lroundf(fabsf(power) * ((1 << MOTOR_PWM_RESOLUTION_BITS) - 1));
  // DRV8871 IN1/IN2 drive/coast truth table. Clear the inactive input first.
  if (power * MOTOR_DIRECTION >= 0) {
    pwm_write(MOTOR_IN2_CHANNEL, 0);
    pwm_write(MOTOR_IN1_CHANNEL, duty);
  } else {
    pwm_write(MOTOR_IN1_CHANNEL, 0);
    pwm_write(MOTOR_IN2_CHANNEL, duty);
  }
}
bool motor_init() {
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, LOW);
  bool first = pwm_attach(MOTOR_IN1_PIN, MOTOR_IN1_CHANNEL, MOTOR_PWM_FREQ_HZ,
                          MOTOR_PWM_RESOLUTION_BITS);
  bool second = pwm_attach(MOTOR_IN2_PIN, MOTOR_IN2_CHANNEL, MOTOR_PWM_FREQ_HZ,
                           MOTOR_PWM_RESOLUTION_BITS);
  initialized = first && second;
  motor_stop();
  return initialized;
}
void motor_stop() {
  target = applied = 0;
  zero_since = last_update = millis();
  write_power(0);
}
void motor_set_target_power(float power) {
  if (!isfinite(power) || !initialized) { motor_stop(); return; }
  if (power == 0) { motor_stop(); return; }
  target = constrain(power, -MOTOR_MAX_POWER, MOTOR_MAX_POWER);
}
void motor_update(uint32_t now) {
  float dt = min(static_cast<uint32_t>(now - last_update), uint32_t(50)) * 0.001f;
  last_update = now;
  if (!initialized || target == 0) { applied = 0; write_power(0); return; }
  int direction = target > 0 ? 1 : -1;
  if (last_direction && direction != last_direction) {
    if (applied != 0) {
      applied = 0;
      zero_since = now;
      write_power(0);
      return;
    }
    if (uint32_t(now - zero_since) < MOTOR_REVERSE_DWELL_MS) return;
  }
  last_direction = direction;
  float max_step = MOTOR_RAMP_PER_S * dt;
  applied += constrain(target - applied, -max_step, max_step);
  write_power(applied);
}
float motor_get_power() { return applied; }
