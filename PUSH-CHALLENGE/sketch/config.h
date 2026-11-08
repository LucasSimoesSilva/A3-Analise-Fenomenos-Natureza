#pragma once
#include <Arduino.h>

// ---------------- Pinos (HX711) ----------------
const uint8_t PIN_HX_DOUT = 3;
const uint8_t PIN_HX_SCK  = 2;

// ---------------- Física ----------------
const float GRAVITY = 9.81f;  // N = kg * 9,81

// ---------------- Calibração ----------------
// "Chute" inicial de contagens por kg. SEMPRE calibre com um peso conhecido
// (comando cal=<kg>) — o valor calibrado fica salvo na EEPROM.
const float   DEFAULT_COUNTS_PER_KG = 100000.0f;
const uint8_t TARE_SAMPLES          = 20;

// ---------------- Detecção do empurrão ----------------
const float    PUSH_START_N   = 5.0f;    // passou disso = começou a empurrar
const float    PUSH_END_N     = 2.0f;    // abaixo disso ...
const uint16_t PUSH_END_HOLD_MS = 50;   // ... por esse tempo = terminou
const uint32_t PUSH_MAX_MS    = 10000;   // segurança: empurrão de no máx. 10 s

// ---------------- Saída serial ----------------
const long SERIAL_BAUD = 115200;
// true = imprime só o número da força por linha (para o Serial Plotter)
const bool SERIAL_PLOT = false;

