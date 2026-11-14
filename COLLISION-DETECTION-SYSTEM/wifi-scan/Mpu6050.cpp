#include "Mpu6050.h"

namespace {
// Registradores (datasheet "MPU-6000/6050 Register Map", rev. 4.2)
constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
constexpr uint8_t REG_CONFIG       = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
constexpr uint8_t REG_WHO_AM_I     = 0x75;

// Sensibilidades conforme as escalas escolhidas em begin()
constexpr float ACCEL_LSB_PER_G  = 2048.0f;  // AFS_SEL=3 -> ±16 g
constexpr float GYRO_LSB_PER_DPS = 65.5f;    // FS_SEL=1  -> ±500 °/s
}

Mpu6050::Mpu6050(TwoWire &wire, uint8_t addr) : wire_(wire), addr_(addr) {}

bool Mpu6050::begin(int sda, int scl, uint32_t i2cHz) {
  wire_.begin(sda, scl, i2cHz);
  wire_.setTimeOut(10);
  delay(50);

  if (!writeReg(REG_PWR_MGMT_1, 0x80)) return false;  // reset do chip
  delay(100);

  return writeReg(REG_PWR_MGMT_1, 0x01)        // acorda; clock = PLL do giroscópio X (mais estável)
      && writeReg(REG_SMPLRT_DIV, 1)           // 1 kHz / (1+1) = 500 Hz
      && writeReg(REG_CONFIG, 0x01)            // DLPF ~184 Hz (atraso ~2 ms): não achata o pico da batida
      && writeReg(REG_GYRO_CONFIG, 0x08)       // ±500 °/s
      && writeReg(REG_ACCEL_CONFIG, 0x18);     // ±16 g (batidas de carrinho passam de 8 g)
}

uint8_t Mpu6050::whoAmI() {
  uint8_t id = 0;
  readRegs(REG_WHO_AM_I, &id, 1);
  return id;
}

bool Mpu6050::read(ImuSample &out) {
  uint8_t b[14];
  if (!readRegs(REG_ACCEL_XOUT_H, b, sizeof b)) return false;

  // Dados em big-endian, inteiros de 16 bits com sinal
  auto i16 = [&b](int i) { return (int16_t)((b[i] << 8) | b[i + 1]); };

  out.accelG.x = i16(0) / ACCEL_LSB_PER_G;
  out.accelG.y = i16(2) / ACCEL_LSB_PER_G;
  out.accelG.z = i16(4) / ACCEL_LSB_PER_G;
  out.tempC    = i16(6) / 340.0f + 36.53f;
  out.gyroDps.x = i16(8) / GYRO_LSB_PER_DPS;
  out.gyroDps.y = i16(10) / GYRO_LSB_PER_DPS;
  out.gyroDps.z = i16(12) / GYRO_LSB_PER_DPS;
  return true;
}

bool Mpu6050::writeReg(uint8_t reg, uint8_t val) {
  wire_.beginTransmission(addr_);
  wire_.write(reg);
  wire_.write(val);
  return wire_.endTransmission() == 0;
}

bool Mpu6050::readRegs(uint8_t reg, uint8_t *buf, size_t len) {
  wire_.beginTransmission(addr_);
  wire_.write(reg);
  if (wire_.endTransmission(false) != 0) return false;  // repeated start

  if (wire_.requestFrom((uint16_t)addr_, (size_t)len, true) != len) return false;
  for (size_t i = 0; i < len; i++) buf[i] = wire_.read();
  return true;
}
