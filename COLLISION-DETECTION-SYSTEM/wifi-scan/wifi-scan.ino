#include <Preferences.h>

#include "config.h"
#include "MovingAverage.h"
#include "Mpu6050.h"
#include "CollisionDetector.h"
#include "Airbag.h"
#include "Net.h"

Mpu6050 imu;
CollisionDetector detector;
Airbag airbag;
Preferences prefs;  // guarda threshold e orientação na flash

MovingAverage<cfg::FILTER_WINDOW> filtX, filtY, filtZ;

ImuSample lastRaw = {};
Vec3 accelFilt = {0, 0, 1};
uint32_t nextSampleUs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t lastBatchMs = 0;
uint32_t lastSampleMs = 0;
uint32_t lastImuRetryMs = 0;
uint16_t imuFails = 0;

// ---------------------------------------------------------------
TelemetrySnapshot makeSnapshot() {
  TelemetrySnapshot s;
  s.tMs = millis();
  s.accelG = accelFilt;
  s.gyroDps = lastRaw.gyroDps;
  s.forwardG = detector.forwardG();
  s.gForce = detector.gForce();
  s.peakG = detector.peakG();
  s.thresholdG = detector.threshold();
  s.tempC = lastRaw.tempC;
  s.axis = detector.axis();
  s.invert = detector.invert();
  s.deployed = detector.state() == AirbagState::DEPLOYED;
  s.collisionMs = detector.collisionMs();
  return s;
}

// TARA: mede o vetor gravidade com o carrinho parado (~1 s, bloqueia o loop).
// Se o carrinho estava mexendo, rejeita e mantém a calibração anterior.
bool calibrate() {
  Serial.println("[CAL] mantenha o carrinho parado...");
  Vec3 sum = {0, 0, 0}, sumSq = {0, 0, 0};
  uint16_t ok = 0;
  ImuSample s;
  for (uint16_t i = 0; i < cfg::CALIBRATION_SAMPLES; i++) {
    if (imu.read(s)) {
      sum.x += s.accelG.x;  sumSq.x += s.accelG.x * s.accelG.x;
      sum.y += s.accelG.y;  sumSq.y += s.accelG.y * s.accelG.y;
      sum.z += s.accelG.z;  sumSq.z += s.accelG.z * s.accelG.z;
      ok++;
    }
    delayMicroseconds(cfg::SAMPLE_PERIOD_US);
  }
  if (ok < cfg::CALIBRATION_SAMPLES / 2) {
    Serial.println("[CAL] ERRO: leituras validas insuficientes (calibracao anterior mantida)");
    Net::publishCalibration(false, 0, 0);
    return false;
  }

  Vec3 g0 = {sum.x / ok, sum.y / ok, sum.z / ok};
  float vx = sumSq.x / ok - g0.x * g0.x;
  float vy = sumSq.y / ok - g0.y * g0.y;
  float vz = sumSq.z / ok - g0.z * g0.z;
  float stdG = sqrtf(max(0.0f, max(vx, max(vy, vz))));
  float mag = norm(g0);
  Serial.printf("[CAL] g0 = (%.3f, %.3f, %.3f)  |g0| = %.3f g  ruido = %.3f g\n", g0.x, g0.y, g0.z, mag, stdG);

  bool good = !(mag < 0.9f || mag > 1.1f || (cfg::CAL_REQUIRE_STILL && stdG > cfg::CAL_MAX_STD_G));
  Net::publishCalibration(good, mag, stdG);
  if (!good) {
    Serial.println("[CAL] REJEITADA: carrinho mexeu ou |g0| longe de 1 g. Calibracao anterior mantida.");
    return false;
  }

  detector.setGravityRef(g0);
  filtX.reset();
  filtY.reset();
  filtZ.reset();
  Net::clearSamples();
  Serial.println("[CAL] OK - G zerado.");
  return true;
}

void rearm() {
  detector.rearm(millis());
  airbag.retract();
  Serial.println("[AIRBAG] rearmado");
}

void setThreshold(float g) {
  g = constrain(g, cfg::MIN_THRESHOLD_G, cfg::MAX_THRESHOLD_G);
  detector.setThreshold(g);
  prefs.putFloat("thr", g);
  Serial.printf("[CFG] novo threshold = %.2f g (salvo)\n", g);
}

void setOrientation(uint8_t axis, bool invert) {
  detector.setForward(axis, invert);
  prefs.putUChar("axis", axis);
  prefs.putBool("inv", invert);
  Serial.printf("[CFG] eixo de movimento = %c%s (salvo)\n", "XYZ"[axis > 2 ? 0 : axis],
                invert ? " invertido" : "");
}

void execute(const Command &c) {
  switch (c.type) {
    case CmdType::REARM:     rearm(); break;
    case CmdType::CALIBRATE: calibrate(); break;
    case CmdType::THRESHOLD: setThreshold(c.value); break;
    case CmdType::AXIS:      setOrientation((uint8_t)c.value, detector.invert()); break;
    case CmdType::INVERT:    setOrientation(detector.axis(), c.value != 0); break;
  }
}

// ---------------------------------------------------------------
void sampleAndDetect() {
  if (!imu.read(lastRaw)) {
    if (imuFails < 0xFFFF) imuFails++;
    if (imuFails >= cfg::IMU_MAX_FAILS && millis() - lastImuRetryMs > 500) {
      lastImuRetryMs = millis();
      Serial.println("[IMU] sem resposta - tentando reiniciar o sensor...");
      imu.begin(cfg::PIN_SDA, cfg::PIN_SCL);
    }
    return;
  }
  imuFails = 0;

  accelFilt.x = filtX.update(lastRaw.accelG.x);
  accelFilt.y = filtY.update(lastRaw.accelG.y);
  accelFilt.z = filtZ.update(lastRaw.accelG.z);

  uint32_t nowMs = millis();
  bool hit = detector.update(accelFilt, nowMs);
  if (hit) airbag.deploy();

  lastSampleMs = nowMs;
  Net::pushSample(detector.forwardG());

  if (hit) {
    TelemetrySnapshot s = makeSnapshot();
    Net::publishCollision(s);
    Serial.printf("[AIRBAG] COLISAO! decel=%.2f g (limite %.2f) em t=%lu ms\n", -s.forwardG,
                  s.thresholdG, (unsigned long)s.collisionMs);
  }
}

void handleCommands() {
  // Comandos do dashboard (WebSocket/HTTP)
  Command c;
  while (Net::popCommand(c)) execute(c);

  // Comandos pelo Monitor Serial: rearm | calibrate | threshold=3.5 | axis=x | invert=1
  while (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (!line.length()) continue;
    String err;
    if (Net::parseCommand(line, c, err)) execute(c);
    else Serial.println("[CMD] " + err);
  }

  // Botão BOOT: toque curto = rearmar | segurar 1,5 s = tara (feita 0,8 s após soltar)
  static uint32_t pressedAt = 0, tareAt = 0;
  static bool longDone = false;
  if (digitalRead(cfg::PIN_REARM_BUTTON) == LOW) {
    if (pressedAt == 0) { pressedAt = millis(); longDone = false; }
    else if (!longDone && millis() - pressedAt > 1500) { longDone = true; Serial.println("[CAL] solte o botao..."); }
  } else if (pressedAt) {
    if (longDone) tareAt = millis() + 800;
    else if (millis() - pressedAt > 50) rearm();
    pressedAt = 0;
  }
  if (tareAt && millis() >= tareAt) { tareAt = 0; calibrate(); }
}

void sendTelemetry() {
  uint32_t now = millis();

  if (now - lastBatchMs >= cfg::BATCH_PERIOD_MS) {
    lastBatchMs = now;
    Net::flushSamples(lastSampleMs, cfg::SAMPLE_PERIOD_US);
  }

  if (now - lastTelemetryMs >= cfg::TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = now;
    TelemetrySnapshot s = makeSnapshot();
    Net::publishTelemetry(s);

    if (cfg::SERIAL_PLOT)
      Serial.printf("forward:%.3f threshold:%.2f peak:%.3f\n", s.forwardG, -s.thresholdG, s.peakG);
  }
}

// ---------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);
  delay(200);
  Serial.println("\n=== A3 | Airbag ESP32 ===");

  pinMode(cfg::PIN_REARM_BUTTON, INPUT_PULLUP);

  airbag.begin(cfg::PIN_SERVO, cfg::SERVO_REST_DEG, cfg::SERVO_DEPLOY_DEG,
               cfg::SERVO_MIN_US, cfg::SERVO_MAX_US);

  while (!imu.begin(cfg::PIN_SDA, cfg::PIN_SCL)) {
    Serial.println("[IMU] ERRO: MPU-6050 nao responde. Confira SDA/SCL/VCC/GND.");
    delay(1000);
  }
  uint8_t id = imu.whoAmI();
  Serial.printf("[IMU] WHO_AM_I = 0x%02X %s\n", id,
                id == 0x68 ? "(OK)" : "(clone/variante — deve funcionar, confira as leituras)");

  prefs.begin("airbag", false);
  float thr = prefs.getFloat("thr", cfg::DEFAULT_THRESHOLD_G);
  thr = constrain(thr, cfg::MIN_THRESHOLD_G, cfg::MAX_THRESHOLD_G);
  detector.begin(thr, cfg::CONFIRM_SAMPLES, cfg::ARM_DELAY_MS);
  detector.setForward(prefs.getUChar("axis", cfg::FORWARD_AXIS),
                      prefs.getBool("inv", cfg::FORWARD_INVERT));
  Serial.printf("[CFG] threshold = %.2f g | eixo = %c%s\n", thr, "XYZ"[detector.axis()],
                detector.invert() ? " invertido" : "");

  // Tara no boot, ANTES de ligar o Wi-Fi. Tenta por até CAL_BOOT_WAIT_MS.
  bool calOk = false;
  uint32_t calStart = millis();
  while (!calOk && millis() - calStart < cfg::CAL_BOOT_WAIT_MS) {
    calOk = calibrate();
    if (!calOk) delay(300);
  }
  if (!calOk)
    Serial.println("[CAL] AVISO: usando gravidade padrao (0,0,1) - o G NAO estara zerado. Envie 'calibrate' com o carrinho parado.");
  detector.rearm(millis());

  if (cfg::NET_ENABLED) {
    Net::begin();
  } else {
    Serial.println("[NET] DESATIVADA (modo diagnostico): so Serial.");
  }

  nextSampleUs = micros();
}

void loop() {
  uint32_t nowUs = micros();
  if ((int32_t)(nowUs - nextSampleUs) >= 0) {
    nextSampleUs += cfg::SAMPLE_PERIOD_US;
    if ((int32_t)(nowUs - nextSampleUs) > (int32_t)(10 * cfg::SAMPLE_PERIOD_US))
      nextSampleUs = nowUs + cfg::SAMPLE_PERIOD_US;
    sampleAndDetect();
  }

  handleCommands();
  sendTelemetry();
  Net::loop();
}
