// =============================================================================
//  IGNIS VECTOR — Controle + Firebase
//  MPU6050 + PID + 4 Servos + Envio Firebase
//
//  Arquitetura dual-core:
//    Core 0 (taskControle) — MPU6050 + PID + Servos  @ 100 Hz
//    Core 1 (loop Arduino) — Firebase envia dados     @ ~13 Hz
//
//  Pinos:
//    MPU6050 SDA  -> GPIO 21
//    MPU6050 SCL  -> GPIO 22
//    Servo 1 (Pitch+, topo)    -> GPIO 5
//    Servo 2 (Roll+, esquerda) -> GPIO 14
//    Servo 3 (Pitch-, base)    -> GPIO 12
//    Servo 4 (Roll-, direita)  -> GPIO 2
//
//  Dependencias:
//    - ESP32Servo
//    - Adafruit MPU6050
//    - Adafruit Unified Sensor
//    - Firebase-ESP-Client (mobizt)
// =============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

// ─── Credenciais ──────────────────────────────────────────────────────────────
#define WIFI_SSID       "Orquidea"
#define WIFI_PASSWORD   "benigna99"
#define DATABASE_URL    "https://next2k26-1a3e7-default-rtdb.firebaseio.com/"
#define API_KEY         "AIzaSyAkPuU0caKcpApNAII_gC5CxaBmGLXv_9Q"

// ─── Pinos ────────────────────────────────────────────────────────────────────
#define PIN_SDA     21
#define PIN_SCL     22
#define PIN_SERVO1  5
#define PIN_SERVO2  14
#define PIN_SERVO3  12
#define PIN_SERVO4  2

// ─── Parametros mecanicos ─────────────────────────────────────────────────────
#define SERVO_NEUTRO   90
#define SERVO_MAX_DEV  20

// ─── Parametros PID ───────────────────────────────────────────────────────────
#define KP  2.2f
#define KI  0.04f
#define KD  0.40f

// ─── Offset de montagem do sensor ────────────────────────────────────────────
// O eixo X apresentava 15 graus de offset com o modelo nivelado.
// Convertido para m/s2: 15 * (9.8 / 90) = 2.61 m/s2
// Se ainda restar desvio apos gravar, ajuste em decimos ate zerar.
#define OFFSET_AX   1.3f
#define OFFSET_AY   0.0f

#define INTEGRAL_MAX  15.0f

// ─── Movimento incremental dos servos ────────────────────────────────────────
// O servo avanca 1 grau por vez a cada PASSO_MS milissegundos.
// Exemplo: posicao 50 -> alvo 55 percorre 50,51,52,53,54,55 com 8ms entre cada passo.
// Reduza PASSO_MS para movimento mais rapido, aumente para mais lento.
#define PASSO_MS  8

// ─── Timings ──────────────────────────────────────────────────────────────────
#define PERIODO_CONTROLE_MS  10
#define PERIODO_FIREBASE_MS  75

// =============================================================================
//  DADOS COMPARTILHADOS entre Core 0 (controle) e Core 1 (Firebase)
// =============================================================================
struct DadosVoo {
    float ax        = 0.0f;
    float ay        = 0.0f;
    int   s1        = SERVO_NEUTRO;
    int   s2        = SERVO_NEUTRO;
    int   s3        = SERVO_NEUTRO;
    int   s4        = SERVO_NEUTRO;
    float pitch_out = 0.0f;
    float roll_out  = 0.0f;
};

DadosVoo dadosVoo;
SemaphoreHandle_t xMutex;

// =============================================================================
//  OBJETOS GLOBAIS
// =============================================================================
Adafruit_MPU6050 mpu;
Servo servo1, servo2, servo3, servo4;

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// =============================================================================
//  PID
// =============================================================================
struct PID {
    float kp, ki, kd;
    float integral      = 0.0f;
    float erro_anterior = 0.0f;
    unsigned long t_us  = 0;

    float compute(float setpoint, float medido) {
        unsigned long agora = micros();
        float dt = (agora - t_us) / 1000000.0f;
        if (dt <= 0.0f || dt > 0.5f) dt = 0.01f;
        t_us = agora;

        float erro = setpoint - medido;
        float P    = kp * erro;

        integral  += erro * dt;
        integral   = constrain(integral, -INTEGRAL_MAX, INTEGRAL_MAX);
        float I    = ki * integral;

        float D    = kd * (erro - erro_anterior) / dt;
        erro_anterior = erro;

        return P + I + D;
    }

    void reset() {
        integral      = 0.0f;
        erro_anterior = 0.0f;
        t_us          = micros();
    }
};

PID pid_pitch = { KP, KI, KD };
PID pid_roll  = { KP, KI, KD };

// =============================================================================
//  MOVIMENTO INCREMENTAL DO SERVO
//  Move o servo 1 grau por vez com intervalo PASSO_MS entre cada passo.
//  Nao bloqueia o loop — a cada chamada avanca no maximo 1 grau.
// =============================================================================
void moverServoIncremental(Servo &s, int &posAtual, int alvo) {
    if (posAtual == alvo) return;

    static unsigned long ultimoPasso = 0;
    unsigned long agora = millis();

    if (agora - ultimoPasso < PASSO_MS) return;
    ultimoPasso = agora;

    if (posAtual < alvo) posAtual++;
    else                 posAtual--;

    posAtual = constrain(posAtual,
                         SERVO_NEUTRO - SERVO_MAX_DEV,
                         SERVO_NEUTRO + SERVO_MAX_DEV);
    s.write(posAtual);
}

// =============================================================================
//  FUNCOES DOS SERVOS
// =============================================================================
void servoSeguro(Servo &s, int angulo) {
    s.write(constrain(angulo, SERVO_NEUTRO - SERVO_MAX_DEV,
                              SERVO_NEUTRO + SERVO_MAX_DEV));
}

void servoSweep(Servo &s, int de, int ate, int passo_ms) {
    int passo = (ate > de) ? 1 : -1;
    for (int ang = de; ang != ate + passo; ang += passo) {
        s.write(ang);
        delay(passo_ms);
    }
}

// =============================================================================
//  VERIFICACAO DE GRAUS DE LIBERDADE
// =============================================================================
void verificacaoGDL() {
    const int MIN_ANG = SERVO_NEUTRO - SERVO_MAX_DEV;
    const int MAX_ANG = SERVO_NEUTRO + SERVO_MAX_DEV;
    const int NEUTRO  = SERVO_NEUTRO;
    const int VEL     = 8;
    const int PAUSA   = 300;

    Serial.println("\n[GDL]  Iniciando verificacao de graus de liberdade...");
    Serial.println("[GDL]  Amplitude: " + String(MIN_ANG) + " a " + String(MAX_ANG) + " graus");
    delay(500);

    Serial.println("[GDL]  Fase 1: amplitude individual");

    Serial.println("[GDL]    S1 (Pitch+, topo)...");
    servoSweep(servo1, NEUTRO, MAX_ANG, VEL); delay(PAUSA);
    servoSweep(servo1, MAX_ANG, MIN_ANG, VEL); delay(PAUSA);
    servoSweep(servo1, MIN_ANG, NEUTRO, VEL); delay(PAUSA);

    Serial.println("[GDL]    S2 (Roll+, esquerda)...");
    servoSweep(servo2, NEUTRO, MAX_ANG, VEL); delay(PAUSA);
    servoSweep(servo2, MAX_ANG, MIN_ANG, VEL); delay(PAUSA);
    servoSweep(servo2, MIN_ANG, NEUTRO, VEL); delay(PAUSA);

    Serial.println("[GDL]    S3 (Pitch-, base)...");
    servoSweep(servo3, NEUTRO, MAX_ANG, VEL); delay(PAUSA);
    servoSweep(servo3, MAX_ANG, MIN_ANG, VEL); delay(PAUSA);
    servoSweep(servo3, MIN_ANG, NEUTRO, VEL); delay(PAUSA);

    Serial.println("[GDL]    S4 (Roll-, direita)...");
    servoSweep(servo4, NEUTRO, MAX_ANG, VEL); delay(PAUSA);
    servoSweep(servo4, MAX_ANG, MIN_ANG, VEL); delay(PAUSA);
    servoSweep(servo4, MIN_ANG, NEUTRO, VEL); delay(PAUSA);

    delay(400);

    Serial.println("[GDL]  Fase 2: S1 + S3 espelhados (Pitch) - 3 ciclos");
    for (int ciclo = 0; ciclo < 3; ciclo++) {
        for (int d = 0; d <= SERVO_MAX_DEV; d++) {
            servo1.write(NEUTRO + d); servo3.write(NEUTRO - d); delay(VEL);
        }
        delay(PAUSA);
        for (int d = SERVO_MAX_DEV; d >= -SERVO_MAX_DEV; d--) {
            servo1.write(NEUTRO + d); servo3.write(NEUTRO - d); delay(VEL);
        }
        delay(PAUSA);
        for (int d = -SERVO_MAX_DEV; d <= 0; d++) {
            servo1.write(NEUTRO + d); servo3.write(NEUTRO - d); delay(VEL);
        }
        delay(PAUSA);
    }

    delay(400);

    Serial.println("[GDL]  Fase 3: S2 + S4 espelhados (Roll) - 3 ciclos");
    for (int ciclo = 0; ciclo < 3; ciclo++) {
        for (int d = 0; d <= SERVO_MAX_DEV; d++) {
            servo2.write(NEUTRO + d); servo4.write(NEUTRO - d); delay(VEL);
        }
        delay(PAUSA);
        for (int d = SERVO_MAX_DEV; d >= -SERVO_MAX_DEV; d--) {
            servo2.write(NEUTRO + d); servo4.write(NEUTRO - d); delay(VEL);
        }
        delay(PAUSA);
        for (int d = -SERVO_MAX_DEV; d <= 0; d++) {
            servo2.write(NEUTRO + d); servo4.write(NEUTRO - d); delay(VEL);
        }
        delay(PAUSA);
    }

    servo1.write(NEUTRO); servo2.write(NEUTRO);
    servo3.write(NEUTRO); servo4.write(NEUTRO);

    Serial.println("[GDL]  Concluida. Todos em neutro (90 graus).");
    Serial.println("[GDL]  Iniciando PID em 2 segundos...\n");
    delay(2000);
}

// =============================================================================
//  TOKEN CALLBACK
// =============================================================================
void myTokenStatusCallback(TokenInfo info) {
    if (info.status == token_status_error)
        Serial.printf("[FB]   Token erro: %s\n", info.error.message.c_str());
    if (info.status == token_status_ready)
        Serial.println("[FB]   Token pronto.");
}

// =============================================================================
//  TASK DE CONTROLE - Core 0
//  MPU6050 -> PID -> movimento incremental -> Servos
// =============================================================================
void taskControle(void *pvParameters) {
    Wire.begin(PIN_SDA, PIN_SCL);

    if (!mpu.begin()) {
        Serial.println("[CTRL] ERRO FATAL: MPU6050 nao encontrado!");
        vTaskDelete(NULL);
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
    Serial.println("[CTRL] MPU6050 OK. Range: +-4g | Filtro: 44Hz.");
    Serial.printf("[CTRL] Offset: ax=%.2f ay=%.2f\n", OFFSET_AX, OFFSET_AY);

    pid_pitch.reset();
    pid_roll.reset();

    // Posicoes atuais dos servos - iniciam no neutro
    int pos_s1 = SERVO_NEUTRO;
    int pos_s2 = SERVO_NEUTRO;
    int pos_s3 = SERVO_NEUTRO;
    int pos_s4 = SERVO_NEUTRO;

    TickType_t xLastWake = xTaskGetTickCount();
    unsigned long t_log  = millis();
    uint32_t ciclos      = 0;

    while (true) {
        sensors_event_t a, g, t;
        mpu.getEvent(&a, &g, &t);

        // Aplica offset de montagem
        float ax = a.acceleration.x - OFFSET_AX;
        float ay = a.acceleration.y - OFFSET_AY;

        // PID calcula correcao em graus de deflexao
        float pitch_out = constrain(pid_pitch.compute(0.0f, ax),
                                    -(float)SERVO_MAX_DEV, (float)SERVO_MAX_DEV);
        float roll_out  = constrain(pid_roll.compute(0.0f,  ay),
                                    -(float)SERVO_MAX_DEV, (float)SERVO_MAX_DEV);

        // Posicoes alvo do mixer de atitude
        int alvo_s1 = constrain(SERVO_NEUTRO + (int)pitch_out,
                                SERVO_NEUTRO - SERVO_MAX_DEV,
                                SERVO_NEUTRO + SERVO_MAX_DEV);
        int alvo_s2 = constrain(SERVO_NEUTRO + (int)roll_out,
                                SERVO_NEUTRO - SERVO_MAX_DEV,
                                SERVO_NEUTRO + SERVO_MAX_DEV);
        int alvo_s3 = constrain(SERVO_NEUTRO - (int)pitch_out,
                                SERVO_NEUTRO - SERVO_MAX_DEV,
                                SERVO_NEUTRO + SERVO_MAX_DEV);
        int alvo_s4 = constrain(SERVO_NEUTRO - (int)roll_out,
                                SERVO_NEUTRO - SERVO_MAX_DEV,
                                SERVO_NEUTRO + SERVO_MAX_DEV);

        // Movimento incremental: 1 grau por PASSO_MS — fluido e sem saltos
        moverServoIncremental(servo1, pos_s1, alvo_s1);
        moverServoIncremental(servo2, pos_s2, alvo_s2);
        moverServoIncremental(servo3, pos_s3, alvo_s3);
        moverServoIncremental(servo4, pos_s4, alvo_s4);

        // Atualiza dados compartilhados com o Firebase
        if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            dadosVoo.ax        = ax;
            dadosVoo.ay        = ay;
            dadosVoo.pitch_out = pitch_out;
            dadosVoo.roll_out  = roll_out;
            dadosVoo.s1        = pos_s1;
            dadosVoo.s2        = pos_s2;
            dadosVoo.s3        = pos_s3;
            dadosVoo.s4        = pos_s4;
            xSemaphoreGive(xMutex);
        }

        ciclos++;

        if (millis() - t_log >= 1000) {
            t_log = millis();
            Serial.printf("[CTRL] %uHz | ax=%.2f ay=%.2f | pitch=%.1f roll=%.1f | S:%d %d %d %d\n",
                          ciclos, ax, ay, pitch_out, roll_out,
                          pos_s1, pos_s2, pos_s3, pos_s4);
            ciclos = 0;
        }

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(PERIODO_CONTROLE_MS));
    }
}

// =============================================================================
//  SETUP
// =============================================================================
void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("\n=========================================");
    Serial.println("  IGNIS VECTOR — Controle + Firebase     ");
    Serial.println("  Core 0: MPU6050 + PID + Servos         ");
    Serial.println("  Core 1: Firebase                       ");
    Serial.println("=========================================");

    xMutex = xSemaphoreCreateMutex();
    if (!xMutex) {
        Serial.println("[BOOT] ERRO FATAL: mutex nao criado!");
        while (true) { delay(1000); }
    }

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    servo1.setPeriodHertz(50); servo1.attach(PIN_SERVO1, 500, 2400);
    servo2.setPeriodHertz(50); servo2.attach(PIN_SERVO2, 500, 2400);
    servo3.setPeriodHertz(50); servo3.attach(PIN_SERVO3, 500, 2400);
    servo4.setPeriodHertz(50); servo4.attach(PIN_SERVO4, 500, 2400);

    servo1.write(SERVO_NEUTRO); servo2.write(SERVO_NEUTRO);
    servo3.write(SERVO_NEUTRO); servo4.write(SERVO_NEUTRO);
    Serial.println("[SERVO] OK — S1(D5) S2(D14) S3(D12) S4(D2) -> 90 graus");

    verificacaoGDL();

    Serial.println("─────────────────────────────────────────────");
    Serial.printf("[WiFi] Conectando a: %s\n", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    unsigned long t_wifi = millis();
    int wifi_tent = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        wifi_tent++;
        Serial.printf("[WiFi] Tentativa %d | status=%d\n", wifi_tent, (int)WiFi.status());
        if (millis() - t_wifi > 15000) {
            Serial.println("[WiFi] ERRO: Timeout! Continuando sem Firebase...");
            Serial.println("─────────────────────────────────────────────");
            goto iniciar_controle;
        }
    }
    Serial.printf("[WiFi] OK! IP: %s | RSSI: %d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());

    {
        Serial.println("[FB]   Configurando...");
        config.database_url           = DATABASE_URL;
        config.api_key                = API_KEY;
        config.cert.data              = nullptr;
        config.token_status_callback  = myTokenStatusCallback;
        config.timeout.serverResponse = 10000;

        if (Firebase.signUp(&config, &auth, "", "")) {
            Serial.println("[FB]   OK: signUp concluido.");
        } else {
            Serial.printf("[FB]   AVISO signUp: %s\n",
                          config.signer.signupError.message.c_str());
        }

        Firebase.begin(&config, &auth);
        Firebase.reconnectWiFi(true);

        Serial.println("[FB]   Aguardando ready...");
        unsigned long t_fb = millis();
        int fb_tent = 0;
        while (!Firebase.ready()) {
            delay(300);
            fb_tent++;
            Serial.printf("[FB]   Tentativa %d (%.1fs)\n",
                          fb_tent, (millis() - t_fb) / 1000.0f);
            if (millis() - t_fb > 15000) {
                Serial.println("[FB]   ERRO: Timeout. Controle segue offline.");
                break;
            }
        }

        if (Firebase.ready()) {
            Serial.println("[FB]   OK: Firebase pronto!");
            Firebase.RTDB.setString(&fbdo, "/status", "IGNIS_VECTOR_ONLINE");
        }
    }
    Serial.println("─────────────────────────────────────────────");

iniciar_controle:
    xTaskCreatePinnedToCore(
        taskControle,
        "Controle",
        4096,
        NULL,
        2,
        NULL,
        0
    );

    Serial.println("[BOOT] Task de controle disparada no Core 0.");
    Serial.println("[BOOT] Firebase rodando no Core 1 (loop).");
    Serial.println("=========================================\n");
    Serial.println("ax\tay\tpitch\troll\tS1\tS2\tS3\tS4");
}

// =============================================================================
//  LOOP — Core 1 — Firebase
// =============================================================================
void loop() {
    static unsigned long t_envio     = 0;
    static uint32_t      envios_ok   = 0;
    static uint32_t      envios_erro = 0;
    static unsigned long t_status    = 0;

    if (!Firebase.ready()) {
        delay(100);
        return;
    }

    unsigned long agora = millis();

    if (agora - t_envio >= PERIODO_FIREBASE_MS) {
        t_envio = agora;

        DadosVoo snap;
        if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            snap.ax        = dadosVoo.ax;
            snap.ay        = dadosVoo.ay;
            snap.pitch_out = dadosVoo.pitch_out;
            snap.roll_out  = dadosVoo.roll_out;
            snap.s1        = dadosVoo.s1;
            snap.s2        = dadosVoo.s2;
            snap.s3        = dadosVoo.s3;
            snap.s4        = dadosVoo.s4;
            xSemaphoreGive(xMutex);
        } else {
            return;
        }

        bool ok = true;

        ok &= Firebase.RTDB.setInt(&fbdo,   "/Inclinacao",  (int)(snap.ax * 10));
        ok &= Firebase.RTDB.setInt(&fbdo,   "/Inclinacao2", (int)(snap.ay * 10));
        ok &= Firebase.RTDB.setInt(&fbdo,   "/Aletas/s1",   snap.s1);
        ok &= Firebase.RTDB.setInt(&fbdo,   "/Aletas/s2",   snap.s2);
        ok &= Firebase.RTDB.setInt(&fbdo,   "/Aletas/s3",   snap.s3);
        ok &= Firebase.RTDB.setInt(&fbdo,   "/Aletas/s4",   snap.s4);
        ok &= Firebase.RTDB.setFloat(&fbdo, "/PID/pitch",   snap.pitch_out);
        ok &= Firebase.RTDB.setFloat(&fbdo, "/PID/roll",    snap.roll_out);

        if (ok) {
            envios_ok++;
        } else {
            envios_erro++;
            Serial.printf("[FB]   ERRO envio #%lu: %s\n",
                          envios_erro, fbdo.errorReason().c_str());
        }
    }

    if (agora - t_status >= 10000) {
        t_status = agora;
        Serial.printf("[FB]   STATUS: %lu OK | %lu erros | RSSI: %d dBm\n",
                      envios_ok, envios_erro, WiFi.RSSI());
    }
}
