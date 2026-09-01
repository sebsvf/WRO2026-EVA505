
#ifndef CONFIG_H
#define CONFIG_H

// ---- UART link to Raspberry Pi 5 --------------------------------------
#define UART_BAUDRATE           115200
#define UART_RX_PIN             16      // TODO: confirm against final wiring
#define UART_TX_PIN             17      // TODO: confirm against final wiring
#define UART_WATCHDOG_TIMEOUT_MS 400    // no valid command within this window -> fail-safe stop

// ---- Motor driver ------------------------------------------------------
// Assumes a generic dual-input PWM/DIR motor driver (e.g. a single-channel
// H-bridge breakout). TODO: confirm the actual driver model/wiring.
#define MOTOR_PWM_PIN            25      // TODO: confirm
#define MOTOR_DIR_PIN            26      // TODO: confirm
#define MOTOR_PWM_FREQ_HZ        20000
#define MOTOR_PWM_RESOLUTION_BITS 10     // 0-1023 duty range
#define MOTOR_PWM_CHANNEL        0

// ---- Steering servo (MG996R) -------------------------------------------
#define SERVO_PIN                27      // TODO: confirm
#define SERVO_CENTER_DEG         90
#define SERVO_MIN_DEG            60      // mechanical limit, see Engineering Diary Table 2
#define SERVO_MAX_DEG            120     // mechanical limit, see Engineering Diary Table 2

// ---- Encoder (Pololu DC gearmotor, integrated encoder) ------------------
#define ENCODER_A_PIN            32      // TODO: confirm
#define ENCODER_B_PIN            33      // TODO: confirm
#define ENCODER_TICKS_PER_REV    20      // TODO: confirm against motor datasheet

// ---- Velocity PID (Section 3.3 / 4.8 of the engineering journal) -------
#define VELOCITY_PID_FREQ_HZ     100
#define VELOCITY_PID_KP          2.0f    // TODO: tune experimentally
#define VELOCITY_PID_KI          0.5f    // TODO: tune experimentally
#define VELOCITY_PID_KD          0.05f   // TODO: tune experimentally
#define WHEEL_RADIUS_MM          32.5f   // Engineering Diary Section 1.3

// ---- Speed profiles ------------------------------------------------------
#define MAX_SPEED_MM_S           1200    // initial target, see Sec 4.11 -- validate experimentally
#define PARK_MAX_SPEED_MM_S      150

#endif  // CONFIG_H
