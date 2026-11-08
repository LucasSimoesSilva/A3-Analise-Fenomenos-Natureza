#include "Airbag.h"

void Airbag::begin(int pin, int restDeg, int deployDeg, int minUs, int maxUs) {
  restDeg_ = restDeg;
  deployDeg_ = deployDeg;

  ESP32PWM::allocateTimer(0);
  servo_.setPeriodHertz(50);  // SG90 trabalha com PWM de 50 Hz (20 ms)
  servo_.attach(pin, minUs, maxUs);
  retract();
}

void Airbag::deploy() {
  // O tempo real de abertura é mecânico (~0,1 s por 60° no SG90).
  servo_.write(deployDeg_);
  deployed_ = true;
}

void Airbag::retract() {
  servo_.write(restDeg_);
  deployed_ = false;
}