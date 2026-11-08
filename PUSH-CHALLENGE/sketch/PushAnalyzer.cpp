#include "PushAnalyzer.h"

void PushAnalyzer::begin(float startN, float endN, uint16_t endHoldMs, uint32_t maxMs) {
  startN_ = startN;
  endN_ = endN;
  endHoldUs_ = (uint32_t)endHoldMs * 1000UL;
  maxUs_ = maxMs * 1000UL;
}

PushAnalyzer::Event PushAnalyzer::update(float f, uint32_t tUs) {
  if (!hasPrev_) {
    prevF_ = f;
    prevUs_ = tUs;
    hasPrev_ = true;
    return NONE;
  }

  float dt = (tUs - prevUs_) * 1e-6f;
  // Puxar (força negativa) não conta como empurrão
  float fa = prevF_ > 0 ? prevF_ : 0;
  float fb = f > 0 ? f : 0;
  float slice = 0.5f * (fa + fb) * dt;  // área do trapézio entre as 2 leituras
  Event ev = NONE;

  if (state_ == IDLE) {
    if (f >= startN_) {
      // Começou! Inclui o trecho desde a leitura anterior (a "subida")
      state_ = PUSHING;
      startUs_ = prevUs_;
      peak_ = f;
      peakUs_ = tUs;
      impulse_ = slice;
      below_ = false;
      ev = STARTED;
    }
  } else {  // PUSHING
    impulse_ += slice;
    if (f > peak_) {
      peak_ = f;
      peakUs_ = tUs;
    }

    if (f < endN_) {
      if (!below_) {
        below_ = true;
        belowSinceUs_ = tUs;
        impulseAtBelow_ = impulse_;
      } else if (tUs - belowSinceUs_ >= endHoldUs_) {
        finish(belowSinceUs_, impulseAtBelow_);
        ev = FINISHED;
      }
    } else {
      below_ = false;  // voltou a empurrar antes do tempo: continua o mesmo empurrão
    }

    if (state_ == PUSHING && tUs - startUs_ >= maxUs_) {
      finish(tUs, impulse_);
      ev = FINISHED;
    }
  }

  prevF_ = f;
  prevUs_ = tUs;
  return ev;
}

void PushAnalyzer::finish(uint32_t endUs, float impulse) {
  last_.id = nextId_++;
  last_.peakN = peak_;
  last_.impulseNs = impulse;
  last_.durationMs = (endUs - startUs_) / 1000UL;
  last_.timeToPeakMs = (peakUs_ - startUs_) / 1000UL;
  if (peak_ > recordPeakN_) recordPeakN_ = peak_;
  state_ = IDLE;
  below_ = false;
}
