#pragma once
// Portable code, also exercised on the host (no Arduino dependencies).
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>

namespace protocol {
inline uint16_t crc16(const char* data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(static_cast<unsigned char>(data[i])) << 8;
    for (int bit = 0; bit < 8; ++bit)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
  }
  return crc;
}
inline bool newer(uint32_t value, uint32_t previous) {
  const uint32_t delta = value - previous;
  return delta != 0 && delta < 0x80000000UL;
}
inline bool number(const char* text, float& value) {
  if (!text || !*text || *text == ' ' || *text == '\t') return false;
  char* end = nullptr;
  errno = 0;
  value = strtof(text, &end);
  return errno == 0 && end != text && *end == '\0' && isfinite(value);
}
inline bool integer(const char* text, uint32_t& value) {
  if (!text || !*text) return false;
  uint64_t result = 0;
  for (const char* p = text; *p; ++p) {
    if (*p < '0' || *p > '9') return false;
    result = result * 10 + (*p - '0');
    if (result > 0xFFFFFFFFULL) return false;
  }
  value = static_cast<uint32_t>(result);
  return true;
}
enum class Kind { Hello, Arm, Stop, Ping, Drive };
struct Command {
  uint32_t session = 0, sequence = 0;
  Kind kind = Kind::Stop;
  float power = 0, angle = 90;
};
inline bool decode(char* line, Command& command) {
  char* star = strchr(line, '*');
  if (!star || strlen(star + 1) != 4) return false;
  uint16_t expected = 0;
  for (int i = 1; i <= 4; ++i) {
    char c = star[i];
    int digit = c >= '0' && c <= '9' ? c - '0' :
                c >= 'A' && c <= 'F' ? c - 'A' + 10 : -1;
    if (digit < 0) return false;
    expected = (expected << 4) | digit;
  }
  if (crc16(line, star - line) != expected) return false;
  *star = '\0';
  char* parts[6];
  size_t count = 0;
  char* start = line;
  for (char* p = line;; ++p) {
    if (*p == ':' || *p == '\0') {
      bool last = *p == '\0';
      if (p == start || count == 6) return false;
      *p = '\0';
      parts[count++] = start;
      start = p + 1;
      if (last) break;
    }
  }
  if (count < 4 || strcmp(parts[0], "C1") ||
      !integer(parts[1], command.session) || command.session == 0 ||
      !integer(parts[2], command.sequence)) return false;
  if (count == 6 && !strcmp(parts[3], "DRIVE")) {
    command.kind = Kind::Drive;
    return number(parts[4], command.power) && number(parts[5], command.angle) &&
           fabsf(command.power) <= 1 && command.angle >= 0 && command.angle <= 180;
  }
  if (count != 4) return false;
  if (!strcmp(parts[3], "HELLO")) command.kind = Kind::Hello;
  else if (!strcmp(parts[3], "ARM")) command.kind = Kind::Arm;
  else if (!strcmp(parts[3], "STOP")) command.kind = Kind::Stop;
  else if (!strcmp(parts[3], "PING")) command.kind = Kind::Ping;
  else return false;
  return true;
}

// Oversized/corrupted line is discarded THROUGH newline, never executed as a suffix.
template <size_t N> class Framer {
 public:
  char line[N] = {};
  bool push(char c) {
    if (c == '\n') {
      bool ready = !discard_ && used_ > 0;
      line[used_] = '\0';
      used_ = 0;
      discard_ = false;
      return ready;
    }
    if (c == '\r') return false;
    if (c < 32 || c > 126 || used_ >= N - 1) discard_ = true;
    if (!discard_) line[used_++] = c;
    return false;
  }
 private:
  size_t used_ = 0;
  bool discard_ = false;
};
}
