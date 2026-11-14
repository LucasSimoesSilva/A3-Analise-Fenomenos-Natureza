#pragma once
#include <Arduino.h>
#include "Mpu6050.h"

struct TelemetrySnapshot {
  uint32_t tMs;
  Vec3 accelG;
  Vec3 gyroDps;
  float forwardG;        // G dinâmico no eixo de movimento (+ acelera, - freia)
  float gForce;
  float peakG;
  float thresholdG;
  float tempC;
  uint8_t axis;          // 0=X 1=Y 2=Z
  bool invert;
  bool deployed;
  uint32_t collisionMs;
};

// Comandos vindos de WebSocket, HTTP ou Serial (mesmo parser para todos)
enum class CmdType : uint8_t { REARM, CALIBRATE, THRESHOLD, AXIS, INVERT };
struct Command {
  CmdType type;
  float value;
};

namespace Net {

bool begin();
void loop();

void publishTelemetry(const TelemetrySnapshot &s);
void publishCollision(const TelemetrySnapshot &s);
void publishCalibration(bool ok, float mag, float stdG);

void pushSample(float forwardG);
void flushSamples(uint32_t tLastMs, uint32_t periodUs);
void clearSamples();

// "rearm" | "calibrate" | "threshold=3.5" | "axis=x|y|z" | "invert=0|1"
bool parseCommand(String text, Command &out, String &err);
bool popCommand(Command &out);

}
