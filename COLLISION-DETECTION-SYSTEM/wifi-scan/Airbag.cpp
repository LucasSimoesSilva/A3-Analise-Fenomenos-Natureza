#include "Airbag.h"

void Airbag::begin(int pin, int restDeg, int deployDeg, int minUs, int maxUs) {
  restDeg_ = restDeg;
  deployDeg_ = deployDeg;

  ESP32PWM::allocateTimer(0);
  servo_.setPeriodHertz(50);
  servo_.attach(pin, minUs, maxUs);
  retract();
}

void Airbag::deploy() {
  servo_.write(deployDeg_);
  deployed_ = true;
}

void Airbag::retract() {
  servo_.write(restDeg_);
  deployed_ = false;
}