#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

// Direct steering actuation. The Raspberry Pi already computes the
// full PD + feedforward steering law (see
// raspberry_pi/control/steering_controller.py); the ESP32 applies the
// received angle as-is, clamped to the mechanical range, since the
// MG996R's internal servo position loop is assumed accurate enough
// that no additional correction is needed here (Sec 4.7).

void servo_init();
void servo_set_angle_deg(float angle_deg);
float servo_get_current_angle_deg();

#endif  // SERVO_CONTROL_H
