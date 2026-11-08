#pragma once
#include <Arduino.h>
#include <HX711.h>

class ForceSensor {
 public:
  void begin(uint8_t doutPin, uint8_t sckPin);

  // Não bloqueia: só retorna true quando o HX711 tem leitura nova.
  bool read(float &newtons);

  bool tare(uint8_t samples);            // zera com a célula vazia
  bool calibrate(float knownKg, uint8_t samples);  // com peso conhecido em cima

  bool loadFromEeprom();  // true se havia calibração salva
  void saveToEeprom();

  long offset() const { return offset_; }
  float countsPerKg() const { return countsPerKg_; }

 private:
  bool averageRaw(uint8_t samples, long &out);

  HX711 hx_;
  long offset_ = 0;
  float countsPerKg_ = 1.0f;
};

