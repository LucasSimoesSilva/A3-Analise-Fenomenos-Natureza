#pragma once
#include <Arduino.h>

struct PushResult {
  uint16_t id;          // número do empurrão (1, 2, 3...)
  float peakN;          // força máxima (N)
  float impulseNs;      // impulso (N·s)
  uint32_t durationMs;  // duração do empurrão
  uint32_t timeToPeakMs;  // quanto tempo levou do início até o pico
};

class PushAnalyzer {
 public:
  enum Event { NONE, STARTED, FINISHED };

  void begin(float startN, float endN, uint16_t endHoldMs, uint32_t maxMs);

  // Chamar a cada leitura nova. tUs = micros() do momento da leitura.
  Event update(float forceN, uint32_t tUs);

  bool pushing() const { return state_ == PUSHING; }
  const PushResult &last() const { return last_; }
  float recordPeakN() const { return recordPeakN_; }
  void resetRecord() { recordPeakN_ = 0; }

 private:
  void finish(uint32_t endUs, float impulse);

  enum State { IDLE, PUSHING };
  State state_ = IDLE;

  float startN_ = 5, endN_ = 2;
  uint32_t endHoldUs_ = 150000, maxUs_ = 10000000;

  bool hasPrev_ = false;
  float prevF_ = 0;
  uint32_t prevUs_ = 0;

  uint32_t startUs_ = 0;
  uint32_t peakUs_ = 0;
  float peak_ = 0;
  float impulse_ = 0;

  bool below_ = false;         // força está abaixo do limite de fim?
  uint32_t belowSinceUs_ = 0;
  float impulseAtBelow_ = 0;   // impulso no instante em que caiu abaixo

  uint16_t nextId_ = 1;
  float recordPeakN_ = 0;
  PushResult last_ = {0, 0, 0, 0, 0};
};
