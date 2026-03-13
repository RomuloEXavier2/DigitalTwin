#include <Arduino.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "time.h"

// --- CREDENCIAIS ---
#define WIFI_SSID "iPhone de Helica"
#define WIFI_PASSWORD "helica123"
#define DATABASE_URL "https://next2k26-1a3e7-default-rtdb.firebaseio.com/"
#define API_KEY "AIzaSyAkPuU0caKcpApNAII_gC5CxaBmGLXv_9Q"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
Adafruit_MPU6050 mpu;

unsigned long ultimoEnvio = 0;
const long intervalo = 75; // 75ms para resposta instantânea

void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22);
    
    if (!mpu.begin()) {
        Serial.println("Erro MPU");
    }

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) { delay(100); }

    configTime(0, 0, "pool.ntp.org");
    config.database_url = DATABASE_URL;
    config.api_key = API_KEY;
    config.cert.data = nullptr;

    Firebase.signUp(&config, &auth, "", "");
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);
}

void loop() {
    if (Firebase.ready() && (millis() - ultimoEnvio > intervalo)) {
        ultimoEnvio = millis();
        
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        
        int incX = (int)(a.acceleration.x * 10);
        int incY = (int)(a.acceleration.y * 10);

        // Envio direto dos eixos e temperatura
        Firebase.RTDB.setInt(&fbdo, "/Inclinacao", incX);
        Firebase.RTDB.setInt(&fbdo, "/Inclinacao2", incY);
        Firebase.RTDB.setFloat(&fbdo, "/Temperatura", temp.temperature);
    }
}