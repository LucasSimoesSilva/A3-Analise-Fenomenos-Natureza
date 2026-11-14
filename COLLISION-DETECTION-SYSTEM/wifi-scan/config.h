#pragma once
#include <Arduino.h>

namespace cfg {

// ---------------- Rede ----------------
constexpr bool     NET_ENABLED  = true;

// true  = ESP32 cria a própria rede (Access Point) -> IP fixo 192.168.4.1
// false = ESP32 entra numa rede existente (Station) -> IP aparece no Serial
constexpr bool     WIFI_AP_MODE = true;
constexpr char     AP_SSID[]    = "A3-Airbag";
constexpr char     AP_PASS[]    = "fisica2026";      // mínimo 8 caracteres
constexpr uint8_t  AP_CHANNEL   = 6;
constexpr uint8_t  AP_MAX_CONN  = 2;
constexpr char     STA_SSID[]   = "NOME_DA_REDE";
constexpr char     STA_PASS[]   = "SENHA_DA_REDE";
constexpr char     MDNS_NAME[]  = "airbag";
constexpr uint16_t HTTP_PORT    = 80;                 // WebSocket em ws://IP/ws

// Potência de transmissão do Wi-Fi (2 a 20 dBm). Menos potência = menos interferência no carrinho.
// O notebook precisa ficar perto do carrinho (poucos metros).
constexpr int      WIFI_TX_POWER_DBM = 5;

// ---------------- Pinos ----------------
constexpr int PIN_SDA          = 21;
constexpr int PIN_SCL          = 22;
constexpr int PIN_SERVO        = 18;
constexpr int PIN_REARM_BUTTON = 0;    // botão "BOOT" da própria placa

// ---------------- Amostragem ----------------
constexpr uint32_t SAMPLE_PERIOD_US    = 2000;  // 500 Hz de leitura do MPU-6050
constexpr uint32_t TELEMETRY_PERIOD_MS = 100;   // 10 Hz: cards do dashboard
constexpr uint32_t BATCH_PERIOD_MS     = 50;    // pacotes com ~25 amostras (curva do gráfico)
constexpr size_t   BATCH_MAX           = 60;
constexpr size_t   FILTER_WINDOW       = 3;
constexpr uint16_t IMU_MAX_FAILS       = 50;

// ---------------- Calibração (tara) ----------------
constexpr uint16_t CALIBRATION_SAMPLES = 500;    // ~1 s medindo a gravidade
constexpr bool     CAL_REQUIRE_STILL   = false;   // false = aceita a tara mesmo com vibração (só confere |g0| ~ 1 g)
constexpr float    CAL_MAX_STD_G       = 0.15f;  // ruído máximo aceito quando CAL_REQUIRE_STILL = true
constexpr uint32_t CAL_BOOT_WAIT_MS    = 15000;  // no boot, tenta calibrar por até 15 s

// ---------------- Detecção de colisão ----------------
constexpr float    DEFAULT_THRESHOLD_G = 3.0f;
constexpr float    MIN_THRESHOLD_G     = 0.5f;
constexpr float    MAX_THRESHOLD_G     = 15.0f;  // escala do sensor é ±16 g
constexpr uint8_t  CONFIRM_SAMPLES     = 2;
constexpr uint32_t ARM_DELAY_MS        = 500;    // após rearmar, ignora impactos

// Eixo do MPU-6050 alinhado com o comprimento do carrinho (0=X, 1=Y, 2=Z).
// Podem ser trocados pelo dashboard (ficam salvos na flash).
constexpr uint8_t  FORWARD_AXIS        = 0;
constexpr bool     FORWARD_INVERT      = false;

// ---------------- Servo (airbag) ----------------
constexpr int SERVO_REST_DEG   = 90;
constexpr int SERVO_DEPLOY_DEG = 180;
constexpr int SERVO_MIN_US     = 500;
constexpr int SERVO_MAX_US     = 2400;

// ---------------- Debug ----------------
constexpr bool SERIAL_PLOT = false;

}
