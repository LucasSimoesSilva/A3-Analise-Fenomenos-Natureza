#include "Net.h"
#include "config.h"

#include <WiFi.h>
#include <esp_wifi.h>
#include <ESPmDNS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

namespace {

AsyncWebServer server(cfg::HTTP_PORT);
AsyncWebSocket ws("/ws");

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
TelemetrySnapshot snapshot = {};

constexpr size_t CMD_QUEUE = 8;
Command cmdQ[CMD_QUEUE];
size_t cmdHead = 0, cmdCount = 0;

float sampleBuf[cfg::BATCH_MAX];
size_t sampleN = 0;

constexpr size_t JSON_BUF = 448;
constexpr size_t BATCH_BUF = 64 + cfg::BATCH_MAX * 9 + 8;

const char TEST_PAGE[] = R"rawliteral(<!DOCTYPE html><html><body><h1>ESP32 Airbag Online</h1></body></html>)rawliteral";

TelemetrySnapshot readSnapshot() {
  portENTER_CRITICAL(&mux);
  TelemetrySnapshot s = snapshot;
  portEXIT_CRITICAL(&mux);
  return s;
}

size_t buildJson(const TelemetrySnapshot &s, const char *type, char *buf, size_t n) {
  int len = snprintf(buf, n,
      "{\"type\":\"%s\",\"t\":%lu,"
      "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
      "\"gx\":%.1f,\"gy\":%.1f,\"gz\":%.1f,"
      "\"af\":%.3f,\"gforce\":%.3f,\"peak\":%.3f,\"threshold\":%.2f,\"temp\":%.1f,"
      "\"axis\":\"%c\",\"invert\":%d,"
      "\"state\":\"%s\",\"collision_t\":%lu}",
      type, (unsigned long)s.tMs,
      s.accelG.x, s.accelG.y, s.accelG.z,
      s.gyroDps.x, s.gyroDps.y, s.gyroDps.z,
      s.forwardG, s.gForce, s.peakG, s.thresholdG, s.tempC,
      "xyz"[s.axis > 2 ? 0 : s.axis], s.invert ? 1 : 0,
      s.deployed ? "DEPLOYED" : "ARMED", (unsigned long)s.collisionMs);
  if (len < 0) return 0;
  return (size_t)len < n ? (size_t)len : n - 1;
}

bool pushCommand(const Command &c) {
  bool ok = false;
  portENTER_CRITICAL(&mux);
  if (cmdCount < CMD_QUEUE) {
    cmdQ[(cmdHead + cmdCount) % CMD_QUEUE] = c;
    cmdCount++;
    ok = true;
  }
  portEXIT_CRITICAL(&mux);
  return ok;
}

bool enqueueCommand(const String &text, String &err) {
  Command c;
  if (!Net::parseCommand(text, c, err)) return false;
  if (!pushCommand(c)) {
    err = "fila de comandos cheia";
    return false;
  }
  return true;
}

String errorJson(String msg) {
  msg.replace("\"", "'");
  msg.replace("\\", "/");
  return "{\"type\":\"error\",\"message\":\"" + msg + "\"}";
}

void onWsEvent(AsyncWebSocket *, AsyncWebSocketClient *client, AwsEventType type,
               void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT: {
      Serial.printf("[WS] cliente #%u conectado (%s)\n", (unsigned)client->id(),
                    client->remoteIP().toString().c_str());
      char buf[JSON_BUF];
      buildJson(readSnapshot(), "telemetry", buf, sizeof buf);
      client->text(buf);
      break;
    }
    case WS_EVT_DISCONNECT:
      Serial.printf("[WS] cliente #%u desconectado\n", (unsigned)client->id());
      break;
    case WS_EVT_DATA: {
      AwsFrameInfo *info = static_cast<AwsFrameInfo *>(arg);
      if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT &&
          len <= 48) {
        String cmd;
        cmd.reserve(len);
        for (size_t i = 0; i < len; i++) cmd += (char)data[i];
        String err;
        if (!enqueueCommand(cmd, err)) client->text(errorJson(err));
      }
      break;
    }
    default:
      break;
  }
}

void sendCommandResult(AsyncWebServerRequest *req, const String &cmd) {
  String err;
  if (enqueueCommand(cmd, err))
    req->send(202, "application/json", "{\"ok\":true}");
  else
    req->send(400, "application/json", errorJson(err));
}

// Unidade do ESP-IDF: 0,25 dBm
void applyTxPower() {
  esp_wifi_set_max_tx_power((int8_t)(cfg::WIFI_TX_POWER_DBM * 4));
  Serial.printf("[WiFi] potencia de TX limitada a %d dBm\n", cfg::WIFI_TX_POWER_DBM);
}

bool startWifi() {
  if (cfg::WIFI_AP_MODE) {
    WiFi.mode(WIFI_AP);
    WiFi.setSleep(false);
    if (!WiFi.softAP(cfg::AP_SSID, cfg::AP_PASS, cfg::AP_CHANNEL, 0, cfg::AP_MAX_CONN)) return false;
    applyTxPower();
    Serial.printf("[WiFi] AP \"%s\" ativo. Dashboard conectar em: ws://%s/ws\n", cfg::AP_SSID,
                  WiFi.softAPIP().toString().c_str());
    return true;
  }

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(cfg::STA_SSID, cfg::STA_PASS);
  applyTxPower();
  Serial.printf("[WiFi] conectando em \"%s\"", cfg::STA_SSID);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() != WL_CONNECTED) return false;

  if (MDNS.begin(cfg::MDNS_NAME)) MDNS.addService("http", "tcp", cfg::HTTP_PORT);
  Serial.printf("[WiFi] conectado. Dashboard: ws://%s/ws\n", WiFi.localIP().toString().c_str());
  return true;
}

}  // namespace

namespace Net {

bool parseCommand(String cmd, Command &out, String &err) {
  cmd.trim();
  cmd.toLowerCase();

  if (cmd == "rearm" || cmd == "reset") {
    out = Command{CmdType::REARM, 0};
    return true;
  }
  if (cmd == "calibrate") {
    out = Command{CmdType::CALIBRATE, 0};
    return true;
  }
  if (cmd.startsWith("threshold=")) {
    float v = cmd.substring(10).toFloat();
    if (v < cfg::MIN_THRESHOLD_G || v > cfg::MAX_THRESHOLD_G) {
      err = "threshold fora da faixa " + String(cfg::MIN_THRESHOLD_G, 1) + "-" +
            String(cfg::MAX_THRESHOLD_G, 1) + " g";
      return false;
    }
    out = Command{CmdType::THRESHOLD, v};
    return true;
  }
  if (cmd.startsWith("axis=")) {
    String a = cmd.substring(5);
    if (a == "x" || a == "y" || a == "z") {
      out = Command{CmdType::AXIS, (float)(a[0] - 'x')};
      return true;
    }
    err = "axis deve ser x, y ou z";
    return false;
  }
  if (cmd.startsWith("invert=")) {
    out = Command{CmdType::INVERT, cmd.substring(7).toInt() ? 1.0f : 0.0f};
    return true;
  }
  err = "comando desconhecido: " + cmd;
  return false;
}

bool popCommand(Command &out) {
  bool ok = false;
  portENTER_CRITICAL(&mux);
  if (cmdCount > 0) {
    out = cmdQ[cmdHead];
    cmdHead = (cmdHead + 1) % CMD_QUEUE;
    cmdCount--;
    ok = true;
  }
  portEXIT_CRITICAL(&mux);
  return ok;
}

bool begin() {
  if (!startWifi()) {
    Serial.println("[WiFi] FALHA ao iniciar a rede");
    return false;
  }

  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "Content-Type");

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send(200, "text/html", TEST_PAGE);
  });

  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *req) {
    char buf[JSON_BUF];
    buildJson(readSnapshot(), "status", buf, sizeof buf);
    req->send(200, "application/json", buf);
  });

  server.on("/api/rearm", HTTP_POST, [](AsyncWebServerRequest *req) {
    sendCommandResult(req, "rearm");
  });

  server.on("/api/calibrate", HTTP_POST, [](AsyncWebServerRequest *req) {
    sendCommandResult(req, "calibrate");
  });

  server.on("/api/threshold", HTTP_POST, [](AsyncWebServerRequest *req) {
    const AsyncWebParameter *p = nullptr;
    if (req->hasParam("value")) p = req->getParam("value");
    else if (req->hasParam("value", true)) p = req->getParam("value", true);
    if (!p) {
      req->send(400, "application/json", errorJson("parametro 'value' ausente"));
      return;
    }
    sendCommandResult(req, "threshold=" + p->value());
  });

  server.onNotFound([](AsyncWebServerRequest *req) {
    if (req->method() == HTTP_OPTIONS) req->send(204);
    else req->send(404, "application/json", errorJson("rota inexistente"));
  });

  server.begin();
  return true;
}

void loop() {
  static uint32_t lastCleanup = 0;
  if (millis() - lastCleanup > 1000) {
    lastCleanup = millis();
    ws.cleanupClients();
  }
}

void publishTelemetry(const TelemetrySnapshot &s) {
  portENTER_CRITICAL(&mux);
  snapshot = s;
  portEXIT_CRITICAL(&mux);

  if (ws.count() == 0 || !ws.availableForWriteAll()) return;
  char buf[JSON_BUF];
  buildJson(s, "telemetry", buf, sizeof buf);
  ws.textAll(buf);
}

void publishCollision(const TelemetrySnapshot &s) {
  portENTER_CRITICAL(&mux);
  snapshot = s;
  portEXIT_CRITICAL(&mux);

  char buf[JSON_BUF];
  buildJson(s, "collision", buf, sizeof buf);
  ws.textAll(buf);
}

void publishCalibration(bool ok, float mag, float stdG) {
  if (ws.count() == 0) return;
  char buf[96];
  snprintf(buf, sizeof buf, "{\"type\":\"cal\",\"ok\":%d,\"mag\":%.3f,\"std\":%.3f}",
           ok ? 1 : 0, mag, stdG);
  ws.textAll(buf);
}

void pushSample(float forwardG) {
  if (sampleN == cfg::BATCH_MAX) {
    memmove(sampleBuf, sampleBuf + 1, (cfg::BATCH_MAX - 1) * sizeof(float));
    sampleN--;
  }
  sampleBuf[sampleN++] = forwardG;
}

void flushSamples(uint32_t tLastMs, uint32_t periodUs) {
  if (sampleN == 0) return;
  if (ws.count() == 0) {
    sampleN = 0;
    return;
  }
  if (!ws.availableForWriteAll()) return;

  char buf[BATCH_BUF];
  int n = snprintf(buf, sizeof buf, "{\"type\":\"batch\",\"t\":%lu,\"dt_us\":%lu,\"a\":[",
                   (unsigned long)tLastMs, (unsigned long)periodUs);
  for (size_t i = 0; i < sampleN && n < (int)sizeof buf - 12; i++)
    n += snprintf(buf + n, sizeof buf - n, "%s%.3f", i ? "," : "", sampleBuf[i]);
  snprintf(buf + n, sizeof buf - n, "]}");
  ws.textAll(buf);
  sampleN = 0;
}

void clearSamples() { sampleN = 0; }

}  // namespace Net
