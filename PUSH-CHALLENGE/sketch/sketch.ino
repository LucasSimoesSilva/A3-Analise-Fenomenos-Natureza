#include "config.h"
#include "ForceSensor.h"
#include "PushAnalyzer.h"

ForceSensor sensor;
PushAnalyzer analyzer;

// ---------------------------------------------------------------
void printSample(float f) {
  if (SERIAL_PLOT) {
    Serial.println(f, 2);
    return;
  }
  Serial.print(F("{\"type\":\"sample\",\"t\":"));
  Serial.print(millis());
  Serial.print(F(",\"f\":"));
  Serial.print(f, 2);
  Serial.print(F(",\"push\":"));
  Serial.print(analyzer.pushing() ? 1 : 0);
  Serial.println('}');
}

void printResult() {
  if (SERIAL_PLOT) return;
  const PushResult &r = analyzer.last();
  Serial.print(F("{\"type\":\"result\",\"id\":"));   Serial.print(r.id);
  Serial.print(F(",\"peak\":"));                    Serial.print(r.peakN, 2);
  Serial.print(F(",\"peak_kg\":"));                 Serial.print(r.peakN / GRAVITY, 2);
  Serial.print(F(",\"impulse\":"));                 Serial.print(r.impulseNs, 3);
  Serial.print(F(",\"duration\":"));                Serial.print(r.durationMs);
  Serial.print(F(",\"time_to_peak\":"));            Serial.print(r.timeToPeakMs);
  Serial.print(F(",\"record\":"));                  Serial.print(analyzer.recordPeakN(), 2);
  Serial.println('}');
}

void printInfo(const __FlashStringHelper *msg) {
  if (SERIAL_PLOT) return;
  Serial.print(F("{\"type\":\"info\",\"message\":\""));
  Serial.print(msg);
  Serial.print(F("\",\"offset\":"));
  Serial.print(sensor.offset());
  Serial.print(F(",\"counts_per_kg\":"));
  Serial.print(sensor.countsPerKg(), 1);
  Serial.println('}');
}

void printError(const __FlashStringHelper *msg) {
  Serial.print(F("{\"type\":\"error\",\"message\":\""));
  Serial.print(msg);
  Serial.println(F("\"}"));
}

// ---------------------------------------------------------------
bool isMeasuring = false;

void processCommand(char *cmd) {
  for (char *p = cmd; *p; p++) *p = tolower(*p);

  // Comando de início da rodada enviado pelo navegador
  if (strcmp(cmd, "start") == 0) {
    isMeasuring = true;
    analyzer.resetRecord(); // Zera o pico guardado no analisador
    printInfo(F("medicao iniciada"));
  } 
  // Comando de encerramento da rodada enviado pelo navegador
  else if (strcmp(cmd, "stop") == 0) {
    isMeasuring = false;
    printInfo(F("medicao parada"));
  }
  else if (strcmp(cmd, "tare") == 0) {
    if (sensor.tare(TARE_SAMPLES)) {
      sensor.saveToEeprom();
      printInfo(F("tara feita"));
    } else {
      printError(F("HX711 nao respondeu"));
    }
  } else if (strncmp(cmd, "cal=", 4) == 0) {
    float kg = atof(cmd + 4);
    if (sensor.calibrate(kg, TARE_SAMPLES)) {
      sensor.saveToEeprom();
      printInfo(F("calibrado e salvo"));
    } else {
      printError(F("calibracao falhou: faca tare antes e coloque o peso"));
    }
  } else if (strcmp(cmd, "reset") == 0) {
    analyzer.resetRecord();
    printInfo(F("recorde zerado"));
  } else if (strcmp(cmd, "info") == 0) {
    printInfo(F("calibracao atual"));
  } else {
    printError(F("comando desconhecido"));
  }
}

// Lê comandos sem travar a leitura do sensor
void readSerialCommands() {
  static char buf[24];
  static uint8_t len = 0;
  static unsigned long lastCharMs = 0;

  while (Serial.available()) {
    char c = Serial.read();
    lastCharMs = millis();
    if (c == '\n' || c == '\r') {
      if (len > 0) {
        buf[len] = '\0';
        processCommand(buf);
        len = 0;
      }
    } else if (c != ' ' && len < sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }
  if (len > 0 && millis() - lastCharMs > 100) {  // aceita também sem Enter
    buf[len] = '\0';
    processCommand(buf);
    len = 0;
  }
}

// ---------------------------------------------------------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  sensor.begin(PIN_HX_DOUT, PIN_HX_SCK);
  analyzer.begin(PUSH_START_N, PUSH_END_N, PUSH_END_HOLD_MS, PUSH_MAX_MS);

  bool hadCal = sensor.loadFromEeprom();

  // Tara a cada ligada (a célula deve estar vazia). Mantém o fator calibrado.
  if (!sensor.tare(TARE_SAMPLES)) printError(F("HX711 nao respondeu: confira DT/SCK/VCC/GND"));

  if (hadCal) printInfo(F("pronto (calibracao carregada da EEPROM)"));
  else printInfo(F("pronto, MAS SEM CALIBRACAO: use cal=KG com um peso conhecido"));
}

void loop() {
  float f;
  if (sensor.read(f)) { 
    // O Arduino só processa o sensor e envia JSON se o START tiver sido acionado
    if (isMeasuring) {
      uint32_t t = micros();
      PushAnalyzer::Event ev = analyzer.update(f, t);

      if (ev == PushAnalyzer::STARTED && !SERIAL_PLOT)
        Serial.println(F("{\"type\":\"push_start\"}"));

      printSample(f);

      if (ev == PushAnalyzer::FINISHED) printResult();
    }
  }

  readSerialCommands();
}