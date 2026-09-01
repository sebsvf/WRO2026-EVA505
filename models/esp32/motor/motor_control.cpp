#include "motor_control.h"
#include "../config.h"
#include "../encoder/encoder.h"

static float _target_speed_mm_s = 0.0f;
static float _current_speed_mm_s = 0.0f;

// PID internal state
static float _integral = 0.0f;
static float _prev_error = 0.0f;
static unsigned long _last_update_ms = 0;

void motor_init() {
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  ledcSetup(MOTOR_PWM_CHANNEL, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION_BITS);
  ledcAttachPin(MOTOR_PWM_PIN, MOTOR_PWM_CHANNEL);
  motor_stop();
}

void motor_set_raw_pwm(int pwm) {
  int max_duty = (1 << MOTOR_PWM_RESOLUTION_BITS) - 1;
  pwm = constrain(pwm, -max_duty, max_duty);

  digitalWrite(MOTOR_DIR_PIN, pwm >= 0 ? HIGH : LOW);
  ledcWrite(MOTOR_PWM_CHANNEL, abs(pwm));
}

void motor_stop() {
  _target_speed_mm_s = 0.0f;
  _integral = 0.0f;
  motor_set_raw_pwm(0);
}

void motor_set_target_speed_mm_s(float target_mm_s) {
  _target_speed_mm_s = constrain(target_mm_s, -MAX_SPEED_MM_S, MAX_SPEED_MM_S);
}

float motor_get_current_speed_mm_s() {
  return _current_speed_mm_s;
}

// Called at VELOCITY_PID_FREQ_HZ (100 Hz) from main.ino's timer loop.
void motor_update_pid() {
  unsigned long now = millis();
  unsigned long dt_ms = (_last_update_ms == 0) ? 0 : (now - _last_update_ms);
  _last_update_ms = now;

  float rpm = encoder_get_rpm(dt_ms > 0 ? dt_ms : 1);
  // v = rpm * 2*pi*r / 60  (Engineering Diary Section 1.4)
  _current_speed_mm_s = rpm * 2.0f * PI * WHEEL_RADIUS_MM / 60.0f;

  float error = _target_speed_mm_s - _current_speed_mm_s;

  float dt_s = dt_ms / 1000.0f;
  if (dt_s > 0) {
    _integral += error * dt_s;
    // Basic anti-windup clamp -- tune alongside VELOCITY_PID_KI.
    _integral = constrain(_integral, -500.0f, 500.0f);
  }
  float derivative = (dt_s > 0) ? (error - _prev_error) / dt_s : 0.0f;
  _prev_error = error;

  float output = VELOCITY_PID_KP * error
               + VELOCITY_PID_KI * _integral
               + VELOCITY_PID_KD * derivative;

  int max_duty = (1 << MOTOR_PWM_RESOLUTION_BITS) - 1;
  int pwm = (int)constrain(output, (float)-max_duty, (float)max_duty);
  motor_set_raw_pwm(pwm);
}
