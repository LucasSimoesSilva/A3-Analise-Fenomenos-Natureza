#pragma once
#include <Arduino.h>
#include "Mpu6050.h"
 
struct TelemetrySnapshot {
  uint32_t tMs;          // millis() da amostra
  Vec3 accelG;           // aceleração filtrada (g), inclui gravidade
  Vec3 gyroDps;          // giroscópio (°/s)
  float forwardG;        // G dinâmico no eixo de movimento (+ acelera, - freia)
  float gForce;          // |a - g0| : módulo do G dinâmico (informativo)
  float peakG;           // maior desaceleração desde o último rearme
  float thresholdG;      // limite configurado
  float tempC;
  uint8_t axis;          // eixo de movimento: 0=X 1=Y 2=Z
  bool invert;           // sentido invertido?
  bool deployed;
  uint32_t collisionMs;  // millis() da colisão (0 = nenhuma)
};
 
// Comandos que chegam por WebSocket, HTTP ou Serial (mesmo parser para todos)
enum class CmdType : uint8_t { REARM, CALIBRATE, THRESHOLD, AXIS, INVERT };
struct Command {
  CmdType type;
  float value;  // THRESHOLD: g | AXIS: 0/1/2 | INVERT: 0/1
};
 
namespace Net {
 
bool begin();
void loop();  // manutenção periódica (limpa clientes desconectados)
 
// Chamados pelo loop principal
void publishTelemetry(const TelemetrySnapshot &s);  // 10 Hz, descartável se a rede engasgar
void publishCollision(const TelemetrySnapshot &s);  // evento: sempre enviado
 
// Curva do gráfico: pushSample() a cada leitura (500 Hz), flushSamples() a cada ~50 ms.
// tLastMs = millis() da ÚLTIMA amostra empurrada; as anteriores são deduzidas por periodUs.
void pushSample(float forwardG);
void flushSamples(uint32_t tLastMs, uint32_t periodUs);
 
// "rearm" | "calibrate" | "threshold=3.5" | "axis=x|y|z" | "invert=0|1"
bool parseCommand(String text, Command &out, String &err);
bool popCommand(Command &out);  // true enquanto houver comando pendente
 
}  // namespace Net