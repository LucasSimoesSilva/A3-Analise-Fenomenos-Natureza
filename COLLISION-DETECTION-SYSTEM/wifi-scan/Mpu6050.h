#pragma once
#include <Arduino.h>
#include <Wire.h>

struct Vec3 {
  float x, y, z;
};

inline float norm(const Vec3 &v) { return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z); }

struct ImuSample {
  Vec3 accelG;    // aceleração em g (1 g = 9,81 m/s²)
  Vec3 gyroDps;   // velocidade angular em °/s
  float tempC;    // temperatura interna do chip
};

class Mpu6050 {
 public:
  explicit Mpu6050(TwoWire &wire = Wire, uint8_t addr = 0x68);

  // Inicializa o I2C e configura: ±8 g, ±500 °/s, filtro interno ~94 Hz.
  bool begin(int sda, int scl, uint32_t i2cHz = 400000);

  // Lê os 14 bytes de uma vez (accel + temp + gyro) — leitura "atômica".
  bool read(ImuSample &out);

  uint8_t whoAmI();

 private:
  bool writeReg(uint8_t reg, uint8_t val);
  bool readRegs(uint8_t reg, uint8_t *buf, size_t len);

  TwoWire &wire_;
  uint8_t addr_;
};
