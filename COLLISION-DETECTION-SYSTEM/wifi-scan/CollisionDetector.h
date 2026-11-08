#pragma once
#include <Arduino.h>
#include "Mpu6050.h"
 
enum class AirbagState : uint8_t { ARMED, DEPLOYED };
 
// Detecta DESACELERAÇÃO BRUSCA no eixo de movimento do carrinho.
//  - arrancada / aceleração (valor positivo no eixo) nunca dispara;
//  - choque lateral ou vertical não dispara (só o eixo de movimento conta);
//  - a gravidade é descontada com a referência g0 medida na calibração.
class CollisionDetector {
 public:
  void begin(float thresholdG, uint8_t confirmSamples, uint32_t armDelayMs);
 
  void setGravityRef(const Vec3 &g0) { g0_ = g0; }
  void setThreshold(float g) { thresholdG_ = g; }
  void setForward(uint8_t axis, bool invert) {
    axis_ = axis > 2 ? 0 : axis;
    invert_ = invert;
  }
  float threshold() const { return thresholdG_; }
  uint8_t axis() const { return axis_; }
  bool invert() const { return invert_; }
 
  // Processa uma amostra filtrada. Retorna true apenas no instante da colisão
  // (borda de subida), para acionar o airbag uma única vez.
  bool update(const Vec3 &accelFiltered, uint32_t nowMs);
 
  // Volta ao estado ARMED, zera o pico e inicia a carência (ARM_DELAY_MS).
  void rearm(uint32_t nowMs);
 
  AirbagState state() const { return state_; }
  float forwardG() const { return forwardG_; }      // G dinâmico no eixo de movimento (+ acelera, - freia)
  float gForce() const { return gForce_; }          // módulo do G dinâmico (todos os eixos), só informativo
  float peakG() const { return peakG_; }            // maior DESACELERAÇÃO desde o último rearme
  uint32_t collisionMs() const { return collisionMs_; }
 
 private:
  Vec3 g0_ = {0, 0, 1};
  float thresholdG_ = 3.0f;
  uint8_t confirmSamples_ = 2;
  uint8_t overCount_ = 0;
  uint8_t axis_ = 0;
  bool invert_ = false;
  uint32_t armDelayMs_ = 0;
  uint32_t armedAtMs_ = 0;
  float forwardG_ = 0;
  float gForce_ = 0;
  float peakG_ = 0;
  uint32_t collisionMs_ = 0;
  AirbagState state_ = AirbagState::ARMED;
};