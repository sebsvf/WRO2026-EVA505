#include <Arduino.h>
#include <Wire.h>

constexpr uint8_t AS5600_ADDR = 0x36;

bool leer(uint8_t registro, uint8_t *datos, uint8_t cantidad) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(registro);

  if (Wire.endTransmission(false) != 0) return false;

  size_t recibidos =
      Wire.requestFrom(AS5600_ADDR, cantidad, true);

  if (recibidos != cantidad) {
    while (Wire.available()) Wire.read();
    return false;
  }

  for (uint8_t i = 0; i < cantidad; i++) {
    datos[i] = Wire.read();
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  if (!Wire.begin(21, 22, 100000)) {
    Serial.println("ERROR: no se pudo iniciar I2C.");
    while (true) delay(1000);
  }

  Wire.setTimeOut(50);
  Serial.println("Prueba AS5600 iniciada.");
}

void loop() {
  uint8_t estado;

  if (!leer(0x0B, &estado, 1)) {
    Serial.println("ERROR I2C: revisar conexiones y alimentacion.");
    delay(1000);
    return;
  }

  const bool detectado = estado & 0x20;
  const bool debil = estado & 0x10;
  const bool fuerte = estado & 0x08;

  if (!detectado) {
    Serial.println("I2C OK | Sin iman detectado");
  } else if (debil || fuerte) {
    Serial.print("I2C OK | Revisar campo:");
    if (debil) Serial.print(" debil");
    if (fuerte) Serial.print(" fuerte");
    Serial.println();
  } else {
    uint8_t datos[2];

    if (!leer(0x0C, datos, 2)) {
      Serial.println("ERROR: no se pudo leer el angulo.");
      delay(250);
      return;
    }

    uint16_t raw =
        ((uint16_t(datos[0]) << 8) | datos[1]) & 0x0FFF;
    float angulo = raw * (360.0f / 4096.0f);

    Serial.print("I2C OK | Iman OK | RAW: ");
    Serial.print(raw);
    Serial.print(" | Grados: ");
    Serial.println(angulo, 2);
  }

  delay(250);
}
