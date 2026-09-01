#include "servo_control.h"
#include "../config.h"
#include <ESP32Servo.h>

static Servo _servo;
static float _current_angle_deg = SERVO_CENTER_DEG;

void servo_init() {
  _servo.setPeriodHertz(50);          // standard analog servo frequency
  _servo.attach(SERVO_PIN, 500, 2400); // pulse-width range, MG996R typical
  servo_set_angle_deg(SERVO_CENTER_DEG);
}

void servo_set_angle_deg(float angle_deg) {
  if (angle_deg < SERVO_MIN_DEG) angle_deg = SERVO_MIN_DEG;
  if (angle_deg > SERVO_MAX_DEG) angle_deg = SERVO_MAX_DEG;

  _current_angle_deg = angle_deg;
  _servo.write((int)angle_deg);
}

float servo_get_current_angle_deg() {
  return _current_angle_deg;
}
