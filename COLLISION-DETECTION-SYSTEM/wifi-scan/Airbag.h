#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>

class Airbag {
 public:
  void begin(int pin, int restDeg, int deployDeg, int minUs, int maxUs);
  void deploy();    // abre o airbag (chamado no mesmo ciclo da detecção)
  void retract();   // volta à posição de repouso
  bool deployed() const { return deployed_; }

 private:
  Servo servo_;
  int restDeg_ = 0;
  int deployDeg_ = 180;
  bool deployed_ = false;
};
