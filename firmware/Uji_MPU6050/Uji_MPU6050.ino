// =================================================================
// Program Pengujian & Kalibrasi U2: IMU Gyroscope MPU6050
// Praktikum Robotika (TC253421) - D3 Teknik Komputer Polsri
// Target: Arduino Uno R3 (I2C SDA=A4, SCL=A5)
// =================================================================

#include <Arduino.h>
#include <Wire.h>
#include "../RobotNavigasiOtonom/Config.h"

float gyroBiasZ = 0.0f;
float sudutZ = 0.0f;
unsigned long lastLoopUs = 0;
unsigned long lastPrintMs = 0;

enum TestMode {
    MODE_IDLE,
    MODE_TURN_LEFT_90,
    MODE_TURN_RIGHT_90
};
TestMode currentMode = MODE_IDLE;

void setMotors(int pwmLeft, int pwmRight) {
    if (pwmLeft > 0) {
        digitalWrite(PIN_IN1, HIGH);
        digitalWrite(PIN_IN2, LOW);
        analogWrite(PIN_ENA, constrain(pwmLeft, 0, 255));
    } else if (pwmLeft < 0) {
        digitalWrite(PIN_IN1, LOW);
        digitalWrite(PIN_IN2, HIGH);
        analogWrite(PIN_ENA, constrain(-pwmLeft, 0, 255));
    } else {
        digitalWrite(PIN_IN1, LOW);
        digitalWrite(PIN_IN2, LOW);
        analogWrite(PIN_ENA, 0);
    }

    if (pwmRight > 0) {
        digitalWrite(PIN_IN3, HIGH);
        digitalWrite(PIN_IN4, LOW);
        analogWrite(PIN_ENB, constrain(pwmRight, 0, 255));
    } else if (pwmRight < 0) {
        digitalWrite(PIN_IN3, LOW);
        digitalWrite(PIN_IN4, HIGH);
        analogWrite(PIN_ENB, constrain(-pwmRight, 0, 255));
    } else {
        digitalWrite(PIN_IN3, LOW);
        digitalWrite(PIN_IN4, LOW);
        analogWrite(PIN_ENB, 0);
    }
}

void stopMotors() {
    setMotors(0, 0);
    currentMode = MODE_IDLE;
}

void calibrateGyro() {
    Serial.println(F("[U2] Mengukur bias gyro (Diamkan robot 2 detik)..."));
    float sum = 0.0f;
    for (int i = 0; i < 200; ++i) {
        Wire.beginTransmission(MPU6050_I2C_ADDR);
        Wire.write(0x47); // GYRO_ZOUT_H
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)MPU6050_I2C_ADDR, (uint8_t)2, (uint8_t)true);

        if (Wire.available() >= 2) {
            int16_t raw = (int16_t)((Wire.read() << 8) | Wire.read());
            sum += (float)raw / GYRO_SCALE_250DPS;
        }
        delay(10);
    }
    gyroBiasZ = sum / 200.0f;
    sudutZ = 0.0f;
    Serial.print(F("[U2] Kalibrasi selesai. Gyro Bias Z: "));
    Serial.print(gyroBiasZ, 4);
    Serial.println(F(" deg/s"));
}

float readGyroZRate() {
    Wire.beginTransmission(MPU6050_I2C_ADDR);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_I2C_ADDR, (uint8_t)2, (uint8_t)true);

    if (Wire.available() >= 2) {
        int16_t raw = (int16_t)((Wire.read() << 8) | Wire.read());
        float dps = ((float)raw / GYRO_SCALE_250DPS) - gyroBiasZ;
        return dps * GYRO_SIGN;
    }
    return 0.0f;
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    #if defined(WIRE_HAS_TIMEOUT)
    Wire.setWireTimeout(3000, true);
    #endif

    pinMode(PIN_ENA, OUTPUT);
    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_IN3, OUTPUT);
    pinMode(PIN_IN4, OUTPUT);
    pinMode(PIN_ENB, OUTPUT);
    stopMotors();

    // Inisialisasi MPU6050
    Wire.beginTransmission(MPU6050_I2C_ADDR);
    Wire.write(0x75); // WHO_AM_I
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_I2C_ADDR, (uint8_t)1, (uint8_t)true);
    uint8_t whoAmI = Wire.read();

    Serial.println(F("================================================="));
    Serial.println(F(" UJI KALIBRASI U2: MPU6050 GYROSCOPE & ROTASI"));
    Serial.print(F(" WHO_AM_I Register: 0x"));
    Serial.println(whoAmI, HEX);

    // Wake up MPU6050
    Wire.beginTransmission(MPU6050_I2C_ADDR);
    Wire.write(0x6B); // PWR_MGMT_1
    Wire.write(0x00);
    Wire.endTransmission(true);

    // Set gyro range +/- 250 dps
    Wire.beginTransmission(MPU6050_I2C_ADDR);
    Wire.write(0x1B);
    Wire.write(0x00);
    Wire.endTransmission(true);

    calibrateGyro();

    Serial.println(F(" Perintah:"));
    Serial.println(F("   'l' : Putar kiri in-place +90 derajat"));
    Serial.println(F("   'r' : Putar kanan in-place -90 derajat"));
    Serial.println(F("   '0' : Reset sudut ke 0 derajat"));
    Serial.println(F("   'c' : Rekalibrasi bias diam"));
    Serial.println(F("   's' : Berhenti darurat"));
    Serial.println(F("================================================="));

    lastLoopUs = micros();
}

void loop() {
    unsigned long nowUs = micros();
    float dt = (float)(nowUs - lastLoopUs) / 1000000.0f;
    lastLoopUs = nowUs;

    float rateZ = readGyroZRate();
    sudutZ += rateZ * dt;

    if (Serial.available() > 0) {
        char cmd = (char)Serial.read();
        if (cmd == 'l') {
            sudutZ = 0.0f;
            currentMode = MODE_TURN_LEFT_90;
            setMotors(-140, 140);
            Serial.println(F("[U2] Mulai rotasi kiri menuju +90 deg..."));
        } else if (cmd == 'r') {
            sudutZ = 0.0f;
            currentMode = MODE_TURN_RIGHT_90;
            setMotors(140, -140);
            Serial.println(F("[U2] Mulai rotasi kanan menuju -90 deg..."));
        } else if (cmd == '0') {
            sudutZ = 0.0f;
            Serial.println(F("[U2] Sudut di-reset ke 0.0 deg."));
        } else if (cmd == 'c') {
            calibrateGyro();
        } else if (cmd == 's') {
            stopMotors();
            Serial.println(F("[U2] Motor berhenti."));
        }
    }

    if (currentMode == MODE_TURN_LEFT_90) {
        if (sudutZ >= 90.0f) {
            stopMotors();
            Serial.print(F("[U2] TARGET TERCAPAI! Sudut akhir: "));
            Serial.print(sudutZ, 2);
            Serial.println(F(" deg. Ukur deviasi busur lantai (E <= 5 deg)."));
        }
    } else if (currentMode == MODE_TURN_RIGHT_90) {
        if (sudutZ <= -90.0f) {
            stopMotors();
            Serial.print(F("[U2] TARGET TERCAPAI! Sudut akhir: "));
            Serial.print(sudutZ, 2);
            Serial.println(F(" deg. Ukur deviasi busur lantai (E <= 5 deg)."));
        }
    }

    if (millis() - lastPrintMs >= 200UL) {
        lastPrintMs = millis();
        Serial.print(F("Sudut Z: "));
        Serial.print(sudutZ, 2);
        Serial.print(F(" deg | Rate Z: "));
        Serial.print(rateZ, 2);
        Serial.println(F(" deg/s"));
    }
}
