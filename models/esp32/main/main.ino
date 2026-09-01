/*
 * main.ino
 * ---------
 * ESP32 firmware entry point. Scope is intentionally narrow (per
 * documentation/software_architecture.md Sec 4.1/4.3): parse commands
 * from the Raspberry Pi over UART, run the deterministic velocity PID
 * and servo actuation, report encoder telemetry back, and enforce the
 * communication watchdog fail-safe. No perception or FSM logic lives
 * here -- that is entirely the Raspberry Pi's responsibility.
 */

#include "../config.h"
#include "../motor/motor_control.h"
#include "../servo/servo_control.h"
#include "../encoder/encoder.h"

static unsigned long _last_command_ms = 0;
static unsigned long _last_pid_update_ms = 0;
static unsigned long _last_telemetry_ms = 0;
static bool _parking_mode = false;

static String _rx_buffer = "";

void setup() {
  Serial.begin(UART_BAUDRATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  motor_init();
  servo_init();
  encoder_init();

  _last_command_ms = millis();
  Serial.println("STATUS:OK");
}

void loop() {
  _read_uart();
  _check_watchdog();

  unsigned long now = millis();

  // Velocity PID at its own fixed rate, independent of UART traffic.
  unsigned long pid_period_ms = 1000UL / VELOCITY_PID_FREQ_HZ;
  if (now - _last_pid_update_ms >= pid_period_ms) {
    motor_update_pid();
    _last_pid_update_ms = now;
  }

  // Telemetry at 20 Hz -- enough for the Pi's watchdog / logging
  // without flooding the link.
  if (now - _last_telemetry_ms >= 50) {
    Serial.print("ENC:");
    Serial.println(encoder_get_ticks());
    _last_telemetry_ms = now;
  }
}

static void _read_uart() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      _handle_line(_rx_buffer);
      _rx_buffer = "";
    } else if (c != '\r') {
      _rx_buffer += c;
    }
  }
}

static void _handle_line(const String &line) {
  _last_command_ms = millis();

  if (line.startsWith("SET_SPEED:")) {
    float mm_s = line.substring(String("SET_SPEED:").length()).toFloat();
    float limit = _parking_mode ? PARK_MAX_SPEED_MM_S : MAX_SPEED_MM_S;
    mm_s = constrain(mm_s, -limit, limit);
    motor_set_target_speed_mm_s(mm_s);

  } else if (line.startsWith("SET_STEERING:")) {
    float deg = line.substring(String("SET_STEERING:").length()).toFloat();
    servo_set_angle_deg(deg);

  } else if (line == "STOP") {
    motor_stop();

  } else if (line == "MODE:PARK") {
    _parking_mode = true;

  } else if (line == "MODE:DRIVE") {
    _parking_mode = false;

  } else if (line == "PING") {
    Serial.println("PONG");

  } else {
    // Unrecognized command -- report a fault rather than silently
    // ignoring it, so protocol drift is visible during testing.
    Serial.println("STATUS:FAULT");
  }
}

static void _check_watchdog() {
  if (millis() - _last_command_ms > UART_WATCHDOG_TIMEOUT_MS) {
    // Fail-safe: stop independently of the Raspberry Pi's state.
    // This is the core safety guarantee described in Sec 4.10 of the
    // engineering journal -- it must not depend on receiving a STOP
    // command, since the whole point is handling the Pi going silent.
    motor_stop();
  }
}
