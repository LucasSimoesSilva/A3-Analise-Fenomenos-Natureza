#pragma once
#include <Arduino.h>
#include <HX711.h>

class ForceSensor {
 public:
  void begin(uint8_t doutPin, uint8_t sckPin);

  bool read(float &newtons);

  bool tare(uint8_t samples);            // zera com a célula vazia
  bool calibrate(float knownKg, uint8_t samples);

  bool loadFromEeprom();
  void saveToEeprom();

  long offset() const { return offset_; }
  float countsPerKg() const { return countsPerKg_; }

 private:
  bool averageRaw(uint8_t samples, long &out);

  HX711 hx_;
  long offset_ = 0;
  float countsPerKg_ = 1.0f;
};

