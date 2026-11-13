// =================================================================
// Program Pengujian & Kalibrasi U1: Encoder Roda (LM393 / FC-03)
// Praktikum Robotika (TC253421) - D3 Teknik Komputer Polsri
// Target: Arduino Uno R3
// =================================================================

#include <Arduino.h>
#include "../RobotNavigasiOtonom/Config.h"

// --- ISR Variables ---
volatile long pulsaKiri  = 0;
volatile long pulsaKanan = 0;
volatile unsigned long tDebounceKiri  = 0;
volatile unsigned long tDebounceKanan = 0;

bool isDriving100cm = false;
long targetPulsa = 0;
unsigned long lastPrintMs = 0;

void isrKiri() {
    unsigned long now = micros();
    if (now - tDebounceKiri >= 1000UL) {
        pulsaKiri++;
        tDebounceKiri = now;
    }
}

void isrKanan() {
    unsigned long now = micros();
    if (now - tDebounceKanan >= 1000UL) {
        pulsaKanan++;
        tDebounceKanan = now;
    }
}

void setMotors(int pwmLeft, int pwmRight) {
    if (pwmLeft > 0) {
        digitalWrite(PIN_IN1, HIGH);
        digitalWrite(PIN_IN2, LOW);
        analogWrite(PIN_ENA, constrain(pwmLeft, 0, 255));
    } else {
        digitalWrite(PIN_IN1, LOW);
        digitalWrite(PIN_IN2, LOW);
        analogWrite(PIN_ENA, 0);
    }

    if (pwmRight > 0) {
        digitalWrite(PIN_IN3, HIGH);
        digitalWrite(PIN_IN4, LOW);
        analogWrite(PIN_ENB, constrain(pwmRight, 0, 255));
    } else {
        digitalWrite(PIN_IN3, LOW);
        digitalWrite(PIN_IN4, LOW);
        analogWrite(PIN_ENB, 0);
    }
}

void stopMotors() {
    setMotors(0, 0);
    isDriving100cm = false;
}

void resetCounts() {
    noInterrupts();
    pulsaKiri = 0;
    pulsaKanan = 0;
    interrupts();
    Serial.println(F("[U1] Counter pulsa di-reset ke 0."));
}

void setup() {
    Serial.begin(115200);

    pinMode(PIN_ENA, OUTPUT);
    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_IN3, OUTPUT);
    pinMode(PIN_IN4, OUTPUT);
    pinMode(PIN_ENB, OUTPUT);

    pinMode(PIN_ENC_KIRI, INPUT);
    pinMode(PIN_ENC_KANAN, INPUT);

    attachInterrupt(digitalPinToInterrupt(PIN_ENC_KIRI), isrKiri, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_KANAN), isrKanan, RISING);

    stopMotors();

    Serial.println(F("================================================="));
    Serial.println(F(" UJI KALIBRASI U1: ENCODER RODA & DIAMETER"));
    Serial.println(F(" Perintah:"));
    Serial.println(F("   'f' : Maju lurus sejauh 100 cm"));
    Serial.println(F("   '0' : Reset counter pulsa"));
    Serial.println(F("   's' : Berhenti darurat"));
    Serial.println(F("================================================="));
}

void loop() {
    if (Serial.available() > 0) {
        char cmd = (char)Serial.read();
        if (cmd == 'f') {
            resetCounts();
            targetPulsa = (long)(100.0f / CM_PER_PULSA); // ~98 pulsa
            isDriving100cm = true;
            setMotors(160, 160);
            Serial.print(F("[U1] Mulai maju 100 cm (Target pulsa: "));
            Serial.print(targetPulsa);
            Serial.println(F(")..."));
        } else if (cmd == '0') {
            resetCounts();
        } else if (cmd == 's') {
            stopMotors();
            Serial.println(F("[U1] Motor dihentikan."));
        }
    }

    noInterrupts();
    long pk = pulsaKiri;
    long pr = pulsaKanan;
    interrupts();

    if (isDriving100cm) {
        long avgPulsa = (pk + pr) / 2;
        if (avgPulsa >= targetPulsa) {
            stopMotors();
            float jarakCm = (float)avgPulsa * CM_PER_PULSA;
            Serial.print(F("[U1] SELESAI MAJU! Pulsa L: "));
            Serial.print(pk);
            Serial.print(F(" | Pulsa R: "));
            Serial.print(pr);
            Serial.print(F(" | Estimasi Jarak: "));
            Serial.print(jarakCm, 2);
            Serial.println(F(" cm"));
            Serial.println(F("Ukur jarak fisik dengan meteran pita, hitung deviasi (E = |100 - aktual| <= 3 cm)."));
        }
    }

    if (millis() - lastPrintMs >= 200UL) {
        lastPrintMs = millis();
        float distL = (float)pk * CM_PER_PULSA;
        float distR = (float)pr * CM_PER_PULSA;
        Serial.print(F("L: "));
        Serial.print(pk);
        Serial.print(F(" ("));
        Serial.print(distL, 1);
        Serial.print(F(" cm) | R: "));
        Serial.print(pr);
        Serial.print(F(" ("));
        Serial.print(distR, 1);
        Serial.println(F(" cm)"));
    }
}
