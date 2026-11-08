#pragma once
#include <stddef.h>

template <size_t N>
class MovingAverage {
  static_assert(N > 0, "A janela da media movel deve ser > 0");

 public:
  float update(float x) {
    sum_ += x - buf_[idx_];  // entra o novo, sai o mais antigo
    buf_[idx_] = x;
    idx_ = (idx_ + 1) % N;
    if (count_ < N) count_++;

    // A cada volta completa recalcula a soma do zero, para o erro de
    // arredondamento do float não se acumular ao longo de horas.
    if (idx_ == 0) {
      sum_ = 0;
      for (size_t i = 0; i < N; i++) sum_ += buf_[i];
    }
    return sum_ / count_;
  }

  float value() const { return count_ ? sum_ / count_ : 0.0f; }

  void reset() {
    for (size_t i = 0; i < N; i++) buf_[i] = 0;
    sum_ = 0;
    idx_ = 0;
    count_ = 0;
  }

 private:
  float buf_[N] = {};
  float sum_ = 0;
  size_t idx_ = 0;
  size_t count_ = 0;
};
