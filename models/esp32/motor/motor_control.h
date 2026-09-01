#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

// Low-level PWM/DIR driver plus the closed-loop velocity PID described
// in Section 3.3 / 4.8 of the engineering journal. The PID itself is
// implemented here (not just the raw PWM writer) because it must run
// deterministically at VELOCITY_PID_FREQ_HZ, which only the ESP32 can
// guarantee.

void motor_init();
void motor_set_raw_pwm(int pwm);       // -1023..1023, signed (direction via sign)
void motor_stop();

// Closed-loop interface used by main.ino:
void motor_set_target_speed_mm_s(float target_mm_s);
void motor_update_pid();               // call at VELOCITY_PID_FREQ_HZ
float motor_get_current_speed_mm_s();

#endif  // MOTOR_CONTROL_H
