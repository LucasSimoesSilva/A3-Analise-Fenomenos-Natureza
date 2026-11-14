#include "ForceSensor.h"
#include "config.h"
#include <EEPROM.h>

namespace {
const uint32_t EEPROM_MAGIC = 0xA3F0CE01;  // marca que existe calibração válida

struct CalData {
  uint32_t magic;
  long offset;
  float countsPerKg;
};
}

void ForceSensor::begin(uint8_t doutPin, uint8_t sckPin) {
  hx_.begin(doutPin, sckPin);  // ganho 128, canal A (padrão)
  countsPerKg_ = DEFAULT_COUNTS_PER_KG;
}

bool ForceSensor::read(float &newtons) {
  if (!hx_.is_ready()) return false;
  long raw = hx_.read();
  newtons = (raw - offset_) / countsPerKg_ * GRAVITY;
  return true;
}

bool ForceSensor::averageRaw(uint8_t samples, long &out) {
  long long sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    if (!hx_.wait_ready_timeout(500)) return false;
    sum += hx_.read();
  }
  out = (long)(sum / samples);
  return true;
}

bool ForceSensor::tare(uint8_t samples) {
  long avg;
  if (!averageRaw(samples, avg)) return false;
  offset_ = avg;
  return true;
}

bool ForceSensor::calibrate(float knownKg, uint8_t samples) {
  if (knownKg <= 0) return false;
  long avg;
  if (!averageRaw(samples, avg)) return false;
  float counts = (float)(avg - offset_);
  if (fabs(counts) < 100) return false;
  countsPerKg_ = counts / knownKg;
  return true;
}

bool ForceSensor::loadFromEeprom() {
  CalData d;
  EEPROM.get(0, d);
  if (d.magic != EEPROM_MAGIC || d.countsPerKg == 0 || isnan(d.countsPerKg)) return false;
  offset_ = d.offset;
  countsPerKg_ = d.countsPerKg;
  return true;
}

void ForceSensor::saveToEeprom() {
  CalData d = {EEPROM_MAGIC, offset_, countsPerKg_};
  EEPROM.put(0, d);
}
