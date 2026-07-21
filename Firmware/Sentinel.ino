/*********************************************************************
 *  SENTINEL V4.1 - FIRMWARE FINAL CONFORME AU RAPPORT
 *  ESP32-S3-CAM + OV3660, CONFIG B
 *  Auteur : TONDE Wendbénédo Daniel
 *  Institut : IST Ouaga 2000 - Licence EII3
 *
 *  Broches conformes au Tableau IV du rapport :
 *  PIR       → GPIO19
 *  Radar     → GPIO20
 *  Laser     → GPIO38
 *  Buzzer    → GPIO39
 *  Servo H   → GPIO14
 *  Servo V   → GPIO47
 *
 *  Seuil IA  : 80 % (conforme rapport section III.1.4)
 *  Délai confirmation : 2s (conforme rapport section II.3.1)
 *  Durée alerte       : 5s (conforme rapport section II.3.1)
 *********************************************************************/

// ========================================
// 1. CONFIGURATION BLYNK
// ========================================
#define BLYNK_TEMPLATE_ID   "TMPL2jqdidmR4"
#define BLYNK_TEMPLATE_NAME "Sentinel"
#define BLYNK_AUTH_TOKEN    "BOF2qRS-bEwfPWT3MFa0uWwa6DdJTFCE"

// ========================================
// 2. INCLUDES
// ========================================
#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include "esp_camera.h"
#include <SENTINELLE_inferencing.h>

// ========================================
// 3. CONFIGURATION WIFI
// ========================================
const char* WIFI_SSID = "DESKTOP-KR9LSRI 9495";
const char* WIFI_PASS = "C1r27*49";

// ========================================
// 4. BROCHES CAMÉRA (CONFIG B VALIDÉE)
// ========================================
#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM   15
#define SIOD_GPIO_NUM   4
#define SIOC_GPIO_NUM   5
#define Y9_GPIO_NUM     48
#define Y8_GPIO_NUM     11
#define Y7_GPIO_NUM     9
#define Y6_GPIO_NUM     8
#define Y5_GPIO_NUM     10
#define Y4_GPIO_NUM     12
#define Y3_GPIO_NUM     18
#define Y2_GPIO_NUM     17
#define VSYNC_GPIO_NUM  6
#define HREF_GPIO_NUM   7
#define PCLK_GPIO_NUM   13

// ========================================
// 5. BROCHES COMPOSANTS
// Conformes au Tableau IV du rapport
// ========================================
#define PIR_PIN       19   // PIR HC-SR501    → GPIO19
#define RADAR_PIN     20   // RCWL-0516       → GPIO20
#define LASER_PIN     38   // Laser KY-008    → GPIO38
#define BUZZER_PIN    39   // Buzzer actif    → GPIO39
#define SERVO_X_PIN   14   // Servo horizontal → GPIO14
#define SERVO_Y_PIN   47   // Servo vertical   → GPIO47

// ========================================
// 6. CONSTANTES CONFORMES AU RAPPORT
// ========================================
#define RAW_COLS  320
#define RAW_ROWS  240

// Seuil confiance IA : 80 % (rapport section III.1.4)
const float THRESHOLD = 0.80f;

// Délai confirmation temporelle : 2s (rapport section II.3.1)
const unsigned long DELAI_CONFIRMATION = 2000;

// Durée alerte : 5s (rapport section II.3.1)
const unsigned long ALARM_DURATION = 5000;

// Positions servos (rapport section II.4.4)
const int SERVO_NEUTRE = 90;
const int SERVO_ALERTE = 45;

// ========================================
// 7. MACHINE À ÉTATS
// ========================================
enum SystemState {
    SURVEILLANCE,
    DETECTION,
    ANALYSE,
    CONFIRMATION,
    ALARME,
    RETOUR
};

SystemState state = SURVEILLANCE;

// ========================================
// 8. VARIABLES GLOBALES
// ========================================
Servo servoX;
Servo servoY;

static uint8_t *snapshot_buf = NULL;

bool pirDetected   = false;
bool radarDetected = false;
bool systemEnabled = true;

bool   threatDetected = false;
String detectedClass  = "NEGATIF";
float  lastScore      = 0.0f;

unsigned long alarmStartTime   = 0;
unsigned long confirmStartTime = 0;

unsigned long totalDetections = 0;
unsigned long humanDetections = 0;
unsigned long droneDetections = 0;

// ========================================
// 9. BLYNK - Virtual pins (Tableau XI rapport)
// V0 : Activation/désactivation système
// V1 : État RCWL-0516
// V2 : État PIR
// V3 : État caméra
// V4 : Dernière classe détectée
// V5 : Score de confiance
// V6 : Historique des détections
// ========================================
BLYNK_WRITE(V0) {
    systemEnabled = param.asInt() == 1;
    Serial.printf("[BLYNK] Système %s\n",
        systemEnabled ? "ACTIVÉ" : "DÉSACTIVÉ");
}

// ========================================
// 10. INITIALISATION CAMÉRA (CONFIG B)
// Identique au code fonctionnel validé
// ========================================
bool initCameraConfigB() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;

    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;

    config.pin_xclk    = XCLK_GPIO_NUM;
    config.pin_pclk    = PCLK_GPIO_NUM;
    config.pin_vsync   = VSYNC_GPIO_NUM;
    config.pin_href    = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn    = PWDN_GPIO_NUM;
    config.pin_reset   = RESET_GPIO_NUM;

    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_RGB565;
    config.frame_size   = FRAMESIZE_QVGA;
    config.fb_count     = 1;
    config.fb_location  = CAMERA_FB_IN_DRAM;  // DRAM plus stable que PSRAM
    config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("[ERREUR] Caméra : 0x%x\n", err);
        return false;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s && s->id.PID == OV3660_PID) {
        s->set_vflip(s, 1);
        s->set_brightness(s, 1);
        s->set_saturation(s, 0);
        Serial.println("[OK] Caméra OV3660 - CONFIG B validée");
    }
    return true;
}

// ========================================
// 11. CAPTURE + REDIMENSIONNEMENT
// Méthode du code fonctionnel validé
// ========================================
bool captureAndDownsample(uint32_t img_width, uint32_t img_height,
                          uint8_t *out_buf) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("[ERREUR] Capture échouée");
        return false;
    }

    uint16_t *fb_buffer = (uint16_t*)fb->buf;

    // Redimensionnement par interpolation du plus proche voisin
    for (uint32_t oy = 0; oy < img_height; oy++) {
        uint32_t sy = (oy * RAW_ROWS) / img_height;
        for (uint32_t ox = 0; ox < img_width; ox++) {
            uint32_t sx = (ox * RAW_COLS) / img_width;

            uint16_t pixel = fb_buffer[sy * RAW_COLS + sx];
            uint8_t r = (pixel >> 11) & 0x1F;
            uint8_t g = (pixel >> 5)  & 0x3F;
            uint8_t b =  pixel        & 0x1F;
            r = (r << 3) | (r >> 2);
            g = (g << 2) | (g >> 4);
            b = (b << 3) | (b >> 2);

            size_t idx = (oy * img_width + ox) * 3;
            out_buf[idx]     = r;
            out_buf[idx + 1] = g;
            out_buf[idx + 2] = b;
        }
    }

    esp_camera_fb_return(fb);
    return true;
}

// ========================================
// 12. CALLBACK EDGE IMPULSE
// ========================================
static int ei_get_data(size_t offset, size_t length, float *out_ptr) {
    size_t pixel_ix = offset * 3;
    for (size_t i = 0; i < length; i++) {
        uint8_t r = snapshot_buf[pixel_ix];
        uint8_t g = snapshot_buf[pixel_ix + 1];
        uint8_t b = snapshot_buf[pixel_ix + 2];
        out_ptr[i] = (r << 16) + (g << 8) + b;
        pixel_ix += 3;
    }
    return 0;
}

// ========================================
// 13. INFÉRENCE IA
// ========================================
bool runInference() {
    Serial.println("[IA] Analyse en cours...");

    if (!captureAndDownsample(
            EI_CLASSIFIER_INPUT_WIDTH,
            EI_CLASSIFIER_INPUT_HEIGHT,
            snapshot_buf)) {
        return false;
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data     = &ei_get_data;

    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);

    if (err != EI_IMPULSE_OK) {
        Serial.printf("[ERREUR] Inférence : %d\n", err);
        return false;
    }

    // Ordre labels : [0]=drone [1]=humain [2]=negatif
    float droneScore = result.classification[0].value;
    float humanScore = result.classification[1].value;
    float otherScore = result.classification[2].value;

    Serial.println("========== RÉSULTAT IA ==========");
    Serial.printf("  DRONE   : %.2f %%\n", droneScore * 100.0f);
    Serial.printf("  HUMAIN  : %.2f %%\n", humanScore * 100.0f);
    Serial.printf("  NÉGATIF : %.2f %%\n", otherScore * 100.0f);
    Serial.printf("  Seuil   : %.0f %%\n", THRESHOLD  * 100.0f);

    threatDetected = false;
    detectedClass  = "NEGATIF";
    lastScore      = otherScore * 100.0f;

    // Décision finale - seuil 80 % conforme au rapport
    if (droneScore >= THRESHOLD &&
        droneScore > humanScore &&
        droneScore > otherScore) {
        detectedClass  = "DRONE";
        threatDetected = true;
        lastScore      = droneScore * 100.0f;
        droneDetections++;
        Serial.println("  >>> DRONE DÉTECTÉ !");
    }
    else if (humanScore >= THRESHOLD &&
             humanScore > droneScore &&
             humanScore > otherScore) {
        detectedClass  = "HUMAIN";
        threatDetected = true;
        lastScore      = humanScore * 100.0f;
        humanDetections++;
        Serial.println("  >>> HUMAIN DÉTECTÉ !");
    }
    else {
        Serial.println("  >>> AUCUNE MENACE");
    }

    totalDetections++;
    Serial.println("==================================");
    Serial.printf("[STATS] Total: %lu | Humain: %lu | Drone: %lu\n",
                  totalDetections, humanDetections, droneDetections);
    Serial.println();

    return true;
}

// ========================================
// 14. ACTIONNEURS
// ========================================
void moveCamera(int x, int y) {
    // Protection butées mécaniques (10°-170°)
    servoX.write(constrain(x, 10, 170));
    servoY.write(constrain(y, 10, 170));
}

void startAlarm() {
    digitalWrite(LASER_PIN,  HIGH);
    digitalWrite(BUZZER_PIN, HIGH);
    Serial.println("[ALARME] Buzzer + Laser ACTIVÉS");
}

void stopAlarm() {
    digitalWrite(LASER_PIN,  LOW);
    digitalWrite(BUZZER_PIN, LOW);
}

// ========================================
// 15. BLYNK - Notifications et historique
// ========================================
void sendNotification(String classe, float score) {
    if (!Blynk.connected()) return;

    String message = "ALERTE SENTINEL !\n";
    message += "Classe : " + classe + "\n";
    message += "Confiance : " + String(score, 1) + " %";

    Blynk.logEvent("alerte_sentinel", message);
    Blynk.virtualWrite(V4, classe);
    Blynk.virtualWrite(V5, score);
    Serial.println("[BLYNK] Notification envoyée");
}

void logDetectionToBlynk() {
    if (!Blynk.connected()) return;
    char line[96];
    snprintf(line, sizeof(line), "t+%lus - %s - %.1f %%",
             millis() / 1000, detectedClass.c_str(), lastScore);
    Blynk.virtualWrite(V6, line);
    Serial.printf("[BLYNK] Historique : %s\n", line);
}

// ========================================
// 16. SETUP
// ========================================
void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println();
    Serial.println("========================================");
    Serial.println("         SENTINEL V4.1");
    Serial.println("  ESP32-S3-CAM | MobileNetV1 96x96 INT8");
    Serial.printf("  Seuil IA : %.0f %% | Confirmation : %lus\n",
                  THRESHOLD * 100.0f, DELAI_CONFIRMATION / 1000);
    Serial.println("========================================\n");

    // GPIO
    pinMode(PIR_PIN,    INPUT);
    pinMode(RADAR_PIN,  INPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(LASER_PIN,  OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LASER_PIN,  LOW);

    // Servos
    servoX.setPeriodHertz(50);
    servoY.setPeriodHertz(50);
    servoX.attach(SERVO_X_PIN, 500, 2400);
    servoY.attach(SERVO_Y_PIN, 500, 2400);
    moveCamera(SERVO_NEUTRE, SERVO_NEUTRE);
    Serial.println("[OK] Servos initialisés à 90°");

    // Caméra
    if (!initCameraConfigB()) {
        Serial.println("[ERREUR] Arrêt système");
        while (true) delay(1000);
    }

    // Buffer IA en RAM standard (plus stable)
    snapshot_buf = (uint8_t*)malloc(
        EI_CLASSIFIER_INPUT_WIDTH *
        EI_CLASSIFIER_INPUT_HEIGHT * 3);
    if (snapshot_buf == NULL) {
        Serial.println("[ERREUR] Allocation buffer IA");
        while (true) delay(1000);
    }
    Serial.printf("[OK] Buffer IA : %d octets\n",
        EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT * 3);

    // WiFi
    Serial.print("[WiFi] Connexion... ");
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[OK] WiFi connecté - IP: %s\n",
            WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\n[INFO] WiFi non disponible - mode local actif");
    }

    // Blynk
    Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASS);
    Blynk.virtualWrite(V3, 1);  // Caméra OK

    state = SURVEILLANCE;

    Serial.printf("\n[OK] Heap libre  : %u octets\n", ESP.getFreeHeap());
    Serial.println("[SYSTÈME] PRÊT - Mode SURVEILLANCE\n");
}

// ========================================
// 17. LOOP PRINCIPAL
// Logique à 3 niveaux conforme rapport II.3.1
// ========================================
void loop() {
    Blynk.run();

    // Système désactivé via Blynk (V0)
    if (!systemEnabled) {
        moveCamera(SERVO_NEUTRE, SERVO_NEUTRE);
        stopAlarm();
        delay(100);
        return;
    }

    // Lecture capteurs
    pirDetected   = digitalRead(PIR_PIN)   == HIGH;
    radarDetected = digitalRead(RADAR_PIN) == HIGH;

    // Mise à jour état capteurs sur Blynk
    Blynk.virtualWrite(V1, radarDetected ? 1 : 0);
    Blynk.virtualWrite(V2, pirDetected   ? 1 : 0);

    switch (state) {

        // ---- NIVEAU 1 : MODE VEILLE ----
        case SURVEILLANCE:
            // Condition logicielle ET (conforme rapport II.3.1)
            if (pirDetected && radarDetected) {
                Serial.println("[NIVEAU 1] Radar + PIR actifs simultanément");
                state = DETECTION;
            }
            break;

        // ---- DÉTECTION : Activation caméra ----
        case DETECTION:
            moveCamera(SERVO_NEUTRE, SERVO_NEUTRE);
            delay(150);  // Stabilisation servo
            state = ANALYSE;
            break;

        // ---- NIVEAU 2 : ANALYSE IA ----
        case ANALYSE:
            if (runInference()) {
                Blynk.virtualWrite(V4, detectedClass);
                Blynk.virtualWrite(V5, lastScore);

                if (threatDetected) {
                    // Menace IA confirmée → confirmation temporelle
                    confirmStartTime = millis();
                    Serial.printf("[NIVEAU 2] Menace détectée : %s (%.1f %%)\n",
                        detectedClass.c_str(), lastScore);
                    Serial.printf("[NIVEAU 3] Confirmation dans %lus...\n",
                        DELAI_CONFIRMATION / 1000);
                    state = CONFIRMATION;
                } else {
                    // Non-menace → retour veille
                    Serial.println("[NIVEAU 2] Non-menace IA - Retour veille");
                    state = RETOUR;
                }
            } else {
                state = RETOUR;
            }
            break;

        // ---- NIVEAU 3 : CONFIRMATION TEMPORELLE (2 secondes) ----
        case CONFIRMATION:
            if (millis() - confirmStartTime >= DELAI_CONFIRMATION) {
                // Revérification capteurs après 2 secondes
                pirDetected   = digitalRead(PIR_PIN)   == HIGH;
                radarDetected = digitalRead(RADAR_PIN) == HIGH;

                if (pirDetected && radarDetected) {
                    Serial.println("[NIVEAU 3] Présence confirmée - Alerte !");
                    state = ALARME;
                } else {
                    Serial.println("[NIVEAU 3] Passage fugace - Fausse alerte ignorée");
                    state = RETOUR;
                }
            }
            break;

        // ---- MODE ALERTE ----
        case ALARME:
            if (alarmStartTime == 0) {
                startAlarm();
                alarmStartTime = millis();
                // Orientation servos vers zone détectée (45°)
                moveCamera(SERVO_ALERTE, SERVO_ALERTE);
                // Notification Blynk
                sendNotification(detectedClass, lastScore);
                logDetectionToBlynk();
            }
            // Maintien alerte 5 secondes
            if (millis() - alarmStartTime >= ALARM_DURATION) {
                stopAlarm();
                alarmStartTime = 0;
                state = RETOUR;
            }
            break;

        // ---- RETOUR EN VEILLE ----
        case RETOUR:
            stopAlarm();
            moveCamera(SERVO_NEUTRE, SERVO_NEUTRE);  // Position neutre 90°
            threatDetected = false;
            detectedClass  = "NEGATIF";
            delay(300);
            state = SURVEILLANCE;
            Serial.println("[SYSTÈME] Retour en mode surveillance\n");
            break;
    }

    delay(20);
}

// ========================================
// 18. VÉRIFICATION MODÈLE
// ========================================
#if !defined(EI_CLASSIFIER_SENSOR) || \
    EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Modèle non compatible avec capteur caméra"
#endif
