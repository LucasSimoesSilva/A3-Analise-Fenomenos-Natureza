#include "CollisionDetector.h"
 
void CollisionDetector::begin(float thresholdG, uint8_t confirmSamples, uint32_t armDelayMs) {
  thresholdG_ = thresholdG;
  confirmSamples_ = confirmSamples > 0 ? confirmSamples : 1;
  armDelayMs_ = armDelayMs;
  rearm(millis());
}
 
bool CollisionDetector::update(const Vec3 &a, uint32_t nowMs) {
  // 1) tira a gravidade; sobra só a aceleração "de verdade" do movimento
  Vec3 dyn = {a.x - g0_.x, a.y - g0_.y, a.z - g0_.z};
  gForce_ = norm(dyn);
 
  // 2) projeta no eixo de movimento (+ = acelerando, - = freando)
  float comp = axis_ == 0 ? dyn.x : (axis_ == 1 ? dyn.y : dyn.z);
  forwardG_ = invert_ ? -comp : comp;
  float decel = -forwardG_;
 
  // 3) carência após rearmar: ignora tudo (pico incluso)
  if (state_ == AirbagState::ARMED && (uint32_t)(nowMs - armedAtMs_) < armDelayMs_) {
    overCount_ = 0;
    return false;
  }
 
  if (decel > peakG_) peakG_ = decel;
 
  if (state_ != AirbagState::ARMED) return false;  // já disparou: fica travado
 
  // 4) só desaceleração forte e SEGUIDA conta (ruído isolado é descartado)
  if (decel >= thresholdG_) {
    if (++overCount_ >= confirmSamples_) {
      state_ = AirbagState::DEPLOYED;
      collisionMs_ = nowMs;
      return true;
    }
  } else {
    overCount_ = 0;
  }
  return false;
}
 
void CollisionDetector::rearm(uint32_t nowMs) {
  state_ = AirbagState::ARMED;
  overCount_ = 0;
  peakG_ = 0;
  collisionMs_ = 0;
  armedAtMs_ = nowMs;
}