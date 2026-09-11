#pragma once
#include <Arduino.h>
bool motor_init();
void motor_stop();  // Immediate zero drive; coasts, not a measured physical stop.
void motor_set_target_power(float power);  // Signed duty fraction [-1, 1].
void motor_update(uint32_t now);
float motor_get_power();                   // Applied duty, NOT measured velocity.
