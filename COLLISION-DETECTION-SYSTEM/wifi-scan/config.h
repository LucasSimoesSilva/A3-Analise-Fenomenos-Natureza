#pragma once
#include <Arduino.h>
 
namespace cfg {
 
// ---------------- Wi-Fi ----------------
// true  = ESP32 cria a própria rede (Access Point) -> IP fixo 192.168.4.1
//         Recomendado para a apresentação: não depende do Wi-Fi da faculdade.
// false = ESP32 entra numa rede existente (Station) -> IP aparece no Serial
constexpr bool     WIFI_AP_MODE = true;
constexpr char     AP_SSID[]    = "A3-Airbag";
constexpr char     AP_PASS[]    = "fisica2026";      // mínimo 8 caracteres
constexpr uint8_t  AP_CHANNEL   = 6;
constexpr uint8_t  AP_MAX_CONN  = 2;                  // só o notebook (+1 reserva)
constexpr char     STA_SSID[]   = "NOME_DA_REDE";
constexpr char     STA_PASS[]   = "SENHA_DA_REDE";
constexpr char     MDNS_NAME[]  = "airbag";           // http://airbag.local (modo STA)
constexpr uint16_t HTTP_PORT    = 80;                 // WebSocket em ws://IP/ws
 
// ---------------- Pinos ----------------
constexpr int PIN_SDA          = 21;
constexpr int PIN_SCL          = 22;
constexpr int PIN_SERVO        = 18;
constexpr int PIN_REARM_BUTTON = 0;    // botão "BOOT" da própria placa
 
// ---------------- Amostragem ----------------
constexpr uint32_t SAMPLE_PERIOD_US    = 2000;  // 500 Hz de leitura do MPU-6050
constexpr uint32_t TELEMETRY_PERIOD_MS = 100;   // 10 Hz: cards do dashboard (X/Y/Z, estado, limiar)
constexpr uint32_t BATCH_PERIOD_MS     = 50;    // 20 pacotes/s com ~25 amostras (curva do gráfico)
constexpr size_t   BATCH_MAX           = 60;    // máx. de amostras guardadas se a rede engasgar
constexpr size_t   FILTER_WINDOW       = 3;     // média móvel: 3 amostras = 6 ms
constexpr uint16_t CALIBRATION_SAMPLES = 500;   // ~1 s parado para medir a gravidade
constexpr float    CAL_MAX_STD_G       = 0.05f; // acima disso o carrinho "mexeu": calibração rejeitada
constexpr uint16_t IMU_MAX_FAILS       = 50;    // leituras I2C falhas seguidas (~100 ms) -> reinicia o sensor
 
// ---------------- Detecção de colisão ----------------
constexpr float    DEFAULT_THRESHOLD_G = 3.0f;
constexpr float    MIN_THRESHOLD_G     = 0.5f;
constexpr float    MAX_THRESHOLD_G     = 15.0f; // escala do sensor é ±16 g
constexpr uint8_t  CONFIRM_SAMPLES     = 2;     // amostras seguidas acima do limite (anti-ruído)
constexpr uint32_t ARM_DELAY_MS        = 500;   // após rearmar, ignora impactos (dedo no botão BOOT etc.)
 
// Eixo do MPU-6050 alinhado com o comprimento do carrinho (0=X, 1=Y, 2=Z).
// FORWARD_INVERT = true se a seta do eixo na placa aponta para TRÁS.
// Ambos podem ser trocados pelo dashboard (ficam salvos na flash).
constexpr uint8_t  FORWARD_AXIS        = 0;
constexpr bool     FORWARD_INVERT      = false;
 
// ---------------- Servo (airbag) ----------------
constexpr int SERVO_REST_DEG   = 90;
constexpr int SERVO_DEPLOY_DEG = 180;
constexpr int SERVO_MIN_US     = 500;           // pulso p/ 0°  (SG90)
constexpr int SERVO_MAX_US     = 2400;          // pulso p/ 180° (SG90)
 
// ---------------- Debug ----------------
// true = imprime "forward:x threshold:y peak:z" no formato do Serial Plotter do Arduino IDE
constexpr bool SERIAL_PLOT = false;
 
}  // namespace cfg