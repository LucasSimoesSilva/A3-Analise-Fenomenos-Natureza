# Integrando Leis de Newton e Sistemas Embarcados em Tempo Real

> **Unidade Curricular:** Análise de Fenômenos Físicos da Natureza (Engenharia de Software)  
> **Instituição:** Faculdade / Universidade  
> **Autores:** Miguel Liz dos Santos, Lucas Simões Carvalho da Silva, Pedro Henrique Leal Martins, Lucas Campos de Souza, Rian Henrique Favretto, Ariana Oliveira  

---

## 📌 Visão Geral do Projeto

Este repositório contém os firmwares em C++ e as interfaces web desenvolvidas para a validação empírica da **Segunda e Terceira Lei de Newton** e do **Teorema do Impulso** ($I = \int F \, dt$). O trabalho integra microcontroladores de baixo custo (ESP32 e Arduino Uno) com protótipos físicos e *dashboards* interativos em tempo real.

O projeto é dividido em dois sistemas independentes:
1. **Sistema de Detecção de Colisão e Airbag (`COLLISION-DETECTION-SYSTEM`)**: Monitoramento inercial sem fio de um miniveículo remoto via ESP32 e acelerômetro MPU-6050 para detecção de desacelerações brutas e acionamento de atuador de segurança (micro servo SG90).
2. **Módulo de Deformação e Desafio do Empurrão (`PUSH-CHALLENGE`)**: Amostragem de microdeformações mecânicas através de uma célula de carga (20 kg) e amplificador HX711 acoplados a um Arduino Uno, integrando a curva Força × Tempo em tempo real e exibindo o resultado em uma interface gamificada estilo *arcade*.

---

## 📁 Estrutura do Repositório

```text
├── COLLISION-DETECTION-SYSTEM/
│   ├── COLLISION-DETECTION-SYSTEM.html   # Dashboard web de telemetria (WebSockets)
│   └── firmware/                         # Código em C++ para o ESP32 (Arduino IDE)
│       ├── wifi-scan.ino                 # Loop principal do firmware
│       ├── config.h                      # Parâmetros de rede, pinagem e limites
│       ├── CollisionDetector.h/.cpp      # Algoritmo de decisão de desaceleração
│       ├── Mpu6050.h/.cpp                # Leitura I2C do acelerômetro MEMS
│       ├── Airbag.h/.cpp                 # Controle PWM do servo SG90 (50 Hz)
│       ├── MovingAverage.h               # Filtro digital de média móvel
│       └── Net.h/.cpp                    # Servidor Wi-Fi Access Point e WebSocket
│
└── PUSH-CHALLENGE/
    ├── PUSH-CHALLENGE.html               # Interface gamificada estilo Arcade (Web Serial API)
    └── firmware/                         # Código em C++ para o Arduino Uno (Arduino IDE)
        ├── sketch.ino                    # Loop principal e comandos seriais
        ├── config.h                      # Configuração de pinos e limiares de força
        ├── ForceSensor.h/.cpp            # Leitura do HX711, tara e calibração
        └── PushAnalyzer.h/.cpp           # Integração trapezoidal do impulso (I = ∫ F dt)
```

---

## 🛠️ Requisitos de Hardware e Pinos de Conexão

### Projeto 1: Collision Detection System (ESP32)
- **Microcontrolador:** ESP32 DevKit v1 (Dual-Core 240 MHz, Wi-Fi 802.11 b/g/n)
- **Acelerômetro/Giroscópio:** MPU-6050 (Comunicação I2C)
  - SDA: GPIO 21
  - SCL: GPIO 22
- **Atuador de Segurança:** Micro Servo SG90 (Sinal PWM)
  - PWM: GPIO 18 (Alimentação 5V/GND)
- **Botão de Rearme Manual:** GPIO 0 (Botão BOOT da própria placa)
- **Base Móvel:** Miniveículo de controle remoto (massa instrumentada: $0{,}2\text{ kg}$)

### Projeto 2: Push Challenge (Arduino Uno)
- **Microcontrolador:** Arduino Uno R3 (ATmega328P)
- **Sensor de Força:** Célula de Carga em barra de alumínio (20 kg, ponte de Wheatstone)
- **Conversor ADC (24 bits):** Módulo HX711
  - DOUT: Pino Digital D3
  - SCK: Pino Digital D2
- **Conexão ao PC:** Cabo USB Type-A/B (Comunicação Serial USB a 115200 baud)
- **Estrutura:** Base rígida de madeira em formato de "L"

---

## 💻 Configuração do Ambiente de Desenvolvimento (Arduino IDE)

### 1. Configurar Placas na Arduino IDE

**Para o ESP32 (`COLLISION-DETECTION-SYSTEM`):**
1. Abra a Arduino IDE e vá em **Arquivo > Preferências**.
2. No campo **URLs Adicionais para o Gerenciador de Placas**, adicione a URL da Espressif.
3. Vá em **Ferramentas > Placa > Gerenciador de Placas...**, busque por `esp32` (Espressif Systems) e instale.
4. Selecione a placa: **ESP32 Dev Module**.

**Para o Arduino Uno (`PUSH-CHALLENGE`):**
1. Vá em **Ferramentas > Placa > Arduino AVR Boards** e selecione **Arduino Uno**.

---

### 2. Instalação de Bibliotecas Necessárias

Abra o Gerenciador de Bibliotecas na Arduino IDE (**Esquema > Incluir Biblioteca > Gerenciador de Bibliotecas...** ou `Ctrl + Shift + I`) e instale as seguintes bibliotecas:

**Bibliotecas para o ESP32 (`COLLISION-DETECTION-SYSTEM`):**
- **ESP32Servo** (por Kevin Harrington / John K. Bennett) — para controle do servo SG90 em 50 Hz.
- **AsyncTCP** (por me-no-dev) — suporte assíncrono para rede Wi-Fi.
- **ESPAsyncWebServer** (por me-no-dev) — servidor web e suporte a WebSockets no ESP32.

**Bibliotecas para o Arduino Uno (`PUSH-CHALLENGE`):**
- **HX711 Arduino Library** (por Bogdan Necula) — para leitura do conversor de 24 bits da célula de carga.

---

## 🚀 Como Executar os Projetos

### Executando o Projeto 1: Sistema de Colisão (`COLLISION-DETECTION-SYSTEM`)
1. Abra a pasta `COLLISION-DETECTION-SYSTEM/firmware` na Arduino IDE e selecione o arquivo `wifi-scan.ino`.
2. Conecte o ESP32 via cabo micro-USB ao computador e grave o código (Carregar / Upload).
3. Ao ligar, o ESP32 criará um Access Point Wi-Fi próprio com o nome `A3-Airbag` (senha: `fisica2026`).
4. Conecte o Wi-Fi do seu notebook/computador na rede `A3-Airbag`.
5. Abra o arquivo `COLLISION-DETECTION-SYSTEM.html` diretamente em qualquer navegador moderno (Chrome, Edge ou Firefox).
6. O dashboard conectará automaticamente ao WebSocket em `ws://192.168.4.1/ws`, exibindo os gráficos de desaceleração a 500 Hz e o estado de acionamento do airbag em tempo real.

### Executando o Projeto 2: Desafio do Empurrão (`PUSH-CHALLENGE`)
1. Abra a pasta `PUSH-CHALLENGE/firmware` na Arduino IDE e selecione o arquivo `sketch.ino`.
2. Conecte o Arduino Uno via cabo USB ao computador e grave o código.
3. Certifique-se de fechar o Monitor Serial da Arduino IDE após a gravação (a porta COM precisa estar livre).
4. Abra o arquivo `PUSH-CHALLENGE.html` no Google Chrome ou Microsoft Edge (navegadores com suporte à Web Serial API).
5. Clique no botão **Conectar Serial** na interface web e selecione a porta COM correspondente ao Arduino Uno (configurada para 115200 baud).
6. O sistema realizará a tara automática e iniciará a leitura das forças aplicadas ($N$) e o cálculo numérico do impulso ($N\cdot s$).

---

## 📜 Licença

Este projeto foi desenvolvido estritamente para fins acadêmicos como requisito de avaliação da Unidade Curricular de Análise de Fenômenos Físicos da Natureza (Engenharia de Software).