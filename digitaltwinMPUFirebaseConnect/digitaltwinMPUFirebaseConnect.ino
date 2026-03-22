// =============================================================================
//  IGNIS VECTOR — Controle + Firebase v4.3
//  Correcao: temporizador independente por servo
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
#define SERVO_MAX_DEV  55

// ─── PID — valores iniciais ───────────────────────────────────────────────────
#define KP_INICIAL  6.0f
#define KI_INICIAL  0.10f
#define KD_INICIAL  0.50f

#define INTEGRAL_MAX  50.0f

// ─── Timings ──────────────────────────────────────────────────────────────────
#define PERIODO_CONTROLE_MS   10
#define PERIODO_FIREBASE_MS   20
#define PERIODO_LER_PID_MS   500
#define PERIODO_TEMP_MS      1000

// ─── Movimento incremental ────────────────────────────────────────────────────
// Cada servo tem seu proprio temporizador — todos se movem simultaneamente
#define PASSO_MS  4

// =============================================================================
//  DADOS COMPARTILHADOS
// =============================================================================
struct DadosVoo {
    float ax = 0.0f;
    float ay = 0.0f;
    int   s1 = SERVO_NEUTRO;
    int   s2 = SERVO_NEUTRO;
    int   s3 = SERVO_NEUTRO;
    int   s4 = SERVO_NEUTRO;
};

struct GanhosPID {
    float kp = KP_INICIAL;
    float ki = KI_INICIAL;
    float kd = KD_INICIAL;
};

DadosVoo  dadosVoo;
GanhosPID ganhosPID;
SemaphoreHandle_t xMutex;

// =============================================================================
//  OBJETOS GLOBAIS
// =============================================================================
Adafruit_MPU6050 mpu;
Servo servo1, servo2, servo3, servo4;

FirebaseData fbdo_envio;
FirebaseData fbdo_pid;
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

    void setGanhos(float novo_kp, float novo_ki, float novo_kd) {
        kp = novo_kp;
        ki = novo_ki;
        kd = novo_kd;
    }

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

PID pid_pitch = { KP_INICIAL, KI_INICIAL, KD_INICIAL };
PID pid_roll  = { KP_INICIAL, KI_INICIAL, KD_INICIAL };

// =============================================================================
//  MOVIMENTO INCREMENTAL — temporizador independente por servo
//  Cada servo recebe seu proprio &ultimoPasso, nao compartilham mais o timer.
//  Resultado: todos os 4 servos se movem simultaneamente a cada PASSO_MS.
// =============================================================================
void moverServoIncremental(Servo &s, int &posAtual, int alvo,
                           unsigned long &ultimoPasso) {
    if (posAtual == alvo) return;
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

void servoSweep(Servo &s, int de, int ate, int passo_ms) {
    int passo = (ate > de) ? 1 : -1;
    for (int ang = de; ang != ate + passo; ang += passo) {
        s.write(ang);
        delay(passo_ms);
    }
}

// =============================================================================
//  VERIFICACAO GDL
// =============================================================================
void verificacaoGDL() {
    const int MIN_ANG = SERVO_NEUTRO - SERVO_MAX_DEV;
    const int MAX_ANG = SERVO_NEUTRO + SERVO_MAX_DEV;
    const int NEUTRO  = SERVO_NEUTRO;
    const int VEL     = 6;
    const int PAUSA   = 300;

    Serial.println("\n[GDL]  Iniciando verificacao...");
    Serial.println("[GDL]  Amplitude: " + String(MIN_ANG) + " a " + String(MAX_ANG) + " graus");
    delay(500);

    Serial.println("[GDL]  Fase 1: amplitude individual");
    Serial.println("[GDL]    S1..."); servoSweep(servo1, NEUTRO, MAX_ANG, VEL); delay(PAUSA); servoSweep(servo1, MAX_ANG, MIN_ANG, VEL); delay(PAUSA); servoSweep(servo1, MIN_ANG, NEUTRO, VEL); delay(PAUSA);
    Serial.println("[GDL]    S2..."); servoSweep(servo2, NEUTRO, MAX_ANG, VEL); delay(PAUSA); servoSweep(servo2, MAX_ANG, MIN_ANG, VEL); delay(PAUSA); servoSweep(servo2, MIN_ANG, NEUTRO, VEL); delay(PAUSA);
    Serial.println("[GDL]    S3..."); servoSweep(servo3, NEUTRO, MAX_ANG, VEL); delay(PAUSA); servoSweep(servo3, MAX_ANG, MIN_ANG, VEL); delay(PAUSA); servoSweep(servo3, MIN_ANG, NEUTRO, VEL); delay(PAUSA);
    Serial.println("[GDL]    S4..."); servoSweep(servo4, NEUTRO, MAX_ANG, VEL); delay(PAUSA); servoSweep(servo4, MAX_ANG, MIN_ANG, VEL); delay(PAUSA); servoSweep(servo4, MIN_ANG, NEUTRO, VEL); delay(PAUSA);

    delay(400);
    Serial.println("[GDL]  Fase 2: S1+S3 espelhados - 3 ciclos");
    for (int c = 0; c < 3; c++) {
        for (int d = 0;              d <= SERVO_MAX_DEV;  d++) { servo1.write(NEUTRO+d); servo3.write(NEUTRO-d); delay(VEL); } delay(PAUSA);
        for (int d = SERVO_MAX_DEV;  d >= -SERVO_MAX_DEV; d--) { servo1.write(NEUTRO+d); servo3.write(NEUTRO-d); delay(VEL); } delay(PAUSA);
        for (int d = -SERVO_MAX_DEV; d <= 0;              d++) { servo1.write(NEUTRO+d); servo3.write(NEUTRO-d); delay(VEL); } delay(PAUSA);
    }

    delay(400);
    Serial.println("[GDL]  Fase 3: S2+S4 espelhados - 3 ciclos");
    for (int c = 0; c < 3; c++) {
        for (int d = 0;              d <= SERVO_MAX_DEV;  d++) { servo2.write(NEUTRO+d); servo4.write(NEUTRO-d); delay(VEL); } delay(PAUSA);
        for (int d = SERVO_MAX_DEV;  d >= -SERVO_MAX_DEV; d--) { servo2.write(NEUTRO+d); servo4.write(NEUTRO-d); delay(VEL); } delay(PAUSA);
        for (int d = -SERVO_MAX_DEV; d <= 0;              d++) { servo2.write(NEUTRO+d); servo4.write(NEUTRO-d); delay(VEL); } delay(PAUSA);
    }

    servo1.write(NEUTRO); servo2.write(NEUTRO);
    servo3.write(NEUTRO); servo4.write(NEUTRO);
    Serial.println("[GDL]  Concluida. PID em 2s...\n");
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
//  TASK DE CONTROLE — Core 0  (100 Hz)
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
    Serial.println("[CTRL] MPU6050 OK.");

    pid_pitch.reset();
    pid_roll.reset();

    int pos_s1 = SERVO_NEUTRO;
    int pos_s2 = SERVO_NEUTRO;
    int pos_s3 = SERVO_NEUTRO;
    int pos_s4 = SERVO_NEUTRO;

    // Temporizador independente para cada servo
    unsigned long passo_s1 = 0;
    unsigned long passo_s2 = 0;
    unsigned long passo_s3 = 0;
    unsigned long passo_s4 = 0;

    TickType_t xLastWake = xTaskGetTickCount();
    unsigned long t_log  = millis();
    uint32_t ciclos      = 0;

    while (true) {
        if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            pid_pitch.setGanhos(ganhosPID.kp, ganhosPID.ki, ganhosPID.kd);
            pid_roll.setGanhos(ganhosPID.kp, ganhosPID.ki, ganhosPID.kd);
            xSemaphoreGive(xMutex);
        }

        sensors_event_t a, g, t;
        mpu.getEvent(&a, &g, &t);

        float ax = a.acceleration.x - 1.3f;
        float ay = a.acceleration.y - 0.0f;

        float pitch_out = constrain(pid_pitch.compute(0.0f, ax),
                                    -(float)SERVO_MAX_DEV, (float)SERVO_MAX_DEV);
        float roll_out  = constrain(pid_roll.compute(0.0f, ay),
                                    -(float)SERVO_MAX_DEV, (float)SERVO_MAX_DEV);

        int alvo_s1 = constrain(SERVO_NEUTRO + (int)pitch_out, SERVO_NEUTRO - SERVO_MAX_DEV, SERVO_NEUTRO + SERVO_MAX_DEV);
        int alvo_s2 = constrain(SERVO_NEUTRO + (int)roll_out,  SERVO_NEUTRO - SERVO_MAX_DEV, SERVO_NEUTRO + SERVO_MAX_DEV);
        int alvo_s3 = constrain(SERVO_NEUTRO - (int)pitch_out, SERVO_NEUTRO - SERVO_MAX_DEV, SERVO_NEUTRO + SERVO_MAX_DEV);
        int alvo_s4 = constrain(SERVO_NEUTRO - (int)roll_out,  SERVO_NEUTRO - SERVO_MAX_DEV, SERVO_NEUTRO + SERVO_MAX_DEV);

        // Cada servo com seu proprio temporizador — movem-se simultaneamente
        moverServoIncremental(servo1, pos_s1, alvo_s1, passo_s1);
        moverServoIncremental(servo2, pos_s2, alvo_s2, passo_s2);
        moverServoIncremental(servo3, pos_s3, alvo_s3, passo_s3);
        moverServoIncremental(servo4, pos_s4, alvo_s4, passo_s4);

        if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            dadosVoo.ax = ax;
            dadosVoo.ay = ay;
            dadosVoo.s1 = pos_s1;
            dadosVoo.s2 = pos_s2;
            dadosVoo.s3 = pos_s3;
            dadosVoo.s4 = pos_s4;
            xSemaphoreGive(xMutex);
        }

        ciclos++;
        if (millis() - t_log >= 1000) {
            t_log = millis();
            Serial.printf("[CTRL] %uHz | ax=%.2f ay=%.2f | p=%.1f r=%.1f | S:%d %d %d %d | KP=%.1f KI=%.2f KD=%.2f\n",
                          ciclos, ax, ay, pitch_out, roll_out,
                          pos_s1, pos_s2, pos_s3, pos_s4,
                          pid_pitch.kp, pid_pitch.ki, pid_pitch.kd);
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
    Serial.println("  IGNIS VECTOR — Controle + Firebase v4.3");
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
    Serial.printf("[SERVO] OK — zona: %d a %d graus\n",
                  SERVO_NEUTRO - SERVO_MAX_DEV, SERVO_NEUTRO + SERVO_MAX_DEV);

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
            Serial.println("[WiFi] ERRO: Timeout! Usando valores iniciais.");
            goto iniciar_controle;
        }
    }
    Serial.printf("[WiFi] OK! IP: %s | RSSI: %d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());

    {
        config.database_url           = DATABASE_URL;
        config.api_key                = API_KEY;
        config.cert.data              = nullptr;
        config.token_status_callback  = myTokenStatusCallback;
        config.timeout.serverResponse = 10000;

        if (Firebase.signUp(&config, &auth, "", ""))
            Serial.println("[FB]   OK: signUp concluido.");
        else
            Serial.printf("[FB]   AVISO signUp: %s\n",
                          config.signer.signupError.message.c_str());

        Firebase.begin(&config, &auth);
        Firebase.reconnectWiFi(true);

        unsigned long t_fb = millis();
        int fb_tent = 0;
        while (!Firebase.ready()) {
            delay(300);
            fb_tent++;
            Serial.printf("[FB]   Tentativa %d (%.1fs)\n",
                          fb_tent, (millis() - t_fb) / 1000.0f);
            if (millis() - t_fb > 15000) {
                Serial.println("[FB]   ERRO: Timeout.");
                break;
            }
        }

        if (Firebase.ready()) {
            Serial.println("[FB]   OK: Firebase pronto!");
            Firebase.RTDB.setString(&fbdo_envio, "/status", "IGNIS_VECTOR_ONLINE");

            FirebaseJson json_pid;
            json_pid.set("kp", KP_INICIAL);
            json_pid.set("ki", KI_INICIAL);
            json_pid.set("kd", KD_INICIAL);
            if (Firebase.RTDB.updateNode(&fbdo_pid, "/PID", &json_pid))
                Serial.printf("[FB]   /PID garantido: KP=%.1f KI=%.2f KD=%.2f\n",
                              KP_INICIAL, KI_INICIAL, KD_INICIAL);

            Serial.println("[FB]   Edite /PID/kp, /PID/ki, /PID/kd no Firebase Console.");
        }
    }
    Serial.println("─────────────────────────────────────────────");

iniciar_controle:
    xTaskCreatePinnedToCore(taskControle, "Controle", 4096, NULL, 2, NULL, 0);
    Serial.println("[BOOT] Core 0: Controle | Core 1: Firebase");
    Serial.println("=========================================\n");
}

// =============================================================================
//  LOOP — Core 1 — Firebase
// =============================================================================
void loop() {
    if (!Firebase.ready()) { delay(50); return; }

    static unsigned long t_envio     = 0;
    static unsigned long t_ler_pid   = 0;
    static unsigned long t_temp      = 0;
    static unsigned long t_status    = 0;
    static uint32_t      envios_ok   = 0;
    static uint32_t      envios_erro = 0;
    static bool          retry       = false;
    static DadosVoo      snap_retry;
    static float         tempFake    = 25.0f;

    unsigned long agora = millis();

    // ── Temperatura: +1°C por segundo, reinicia em 25 apos 100 ──────────────
    if (agora - t_temp >= PERIODO_TEMP_MS) {
        t_temp = agora;
        tempFake += 1.0f;
        if (tempFake > 100.0f) tempFake = 25.0f;
    }

    // ── Leitura dos ganhos PID a cada 500ms ───────────────────────────────────
    if (agora - t_ler_pid >= PERIODO_LER_PID_MS) {
        t_ler_pid = agora;

        FirebaseJson     json_lido;
        FirebaseJsonData dado;

        if (Firebase.RTDB.getJSON(&fbdo_pid, "/PID", &json_lido)) {
            float novo_kp = KP_INICIAL;
            float novo_ki = KI_INICIAL;
            float novo_kd = KD_INICIAL;

            json_lido.get(dado, "kp"); if (dado.success) novo_kp = dado.to<float>();
            json_lido.get(dado, "ki"); if (dado.success) novo_ki = dado.to<float>();
            json_lido.get(dado, "kd"); if (dado.success) novo_kd = dado.to<float>();

            if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                bool mudou = (ganhosPID.kp != novo_kp ||
                              ganhosPID.ki != novo_ki ||
                              ganhosPID.kd != novo_kd);
                ganhosPID.kp = novo_kp;
                ganhosPID.ki = novo_ki;
                ganhosPID.kd = novo_kd;
                xSemaphoreGive(xMutex);
                if (mudou)
                    Serial.printf("[PID]  Atualizado: KP=%.2f KI=%.3f KD=%.3f\n",
                                  novo_kp, novo_ki, novo_kd);
            }
        }
    }

    // ── Telemetria: updateNode nao toca em /PID ───────────────────────────────
    DadosVoo snap;
    if (!retry) {
        if (agora - t_envio < PERIODO_FIREBASE_MS) return;
        t_envio = agora;

        if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            snap = dadosVoo;
            xSemaphoreGive(xMutex);
        } else {
            return;
        }
    } else {
        snap = snap_retry;
    }

    FirebaseJson json;
    json.set("Inclinacao",  (int)(snap.ax * 10));
    json.set("Inclinacao2", (int)(snap.ay * 10));
    json.set("Temperatura", tempFake);
    json.set("Aletas/s1",   snap.s1);
    json.set("Aletas/s2",   snap.s2);
    json.set("Aletas/s3",   snap.s3);
    json.set("Aletas/s4",   snap.s4);

    if (Firebase.RTDB.updateNode(&fbdo_envio, "/", &json)) {
        envios_ok++;
        retry = false;
    } else {
        envios_erro++;
        retry      = true;
        snap_retry = snap;
        Serial.printf("[FB]   ERRO #%lu (retry): %s\n",
                      envios_erro, fbdo_envio.errorReason().c_str());
    }

    if (agora - t_status >= 10000) {
        t_status = agora;
        Serial.printf("[FB]   STATUS: %lu OK | %lu erros | Temp=%.0f | RSSI: %d dBm\n",
                      envios_ok, envios_erro, tempFake, WiFi.RSSI());
    }
}
