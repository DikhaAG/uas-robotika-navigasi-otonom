#include "Hal.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#ifdef ARDUINO
#include <EEPROM.h>

void Hal::init() {
#if (BOARD_TYPE == BOARD_ESP32)
    // ESP32 LEDC PWM setup
    ledcAttach(PIN_ENA, 1000, 8); // 1 kHz, 8-bit resolution
    ledcAttach(PIN_ENB, 1000, 8);
    // ESP32 Flash EEPROM partition emulation initialization
    EEPROM.begin(512);
#endif
}

void Hal::pinMode(uint8_t pin, uint8_t mode) {
    ::pinMode(pin, mode);
}

void Hal::digitalWrite(uint8_t pin, uint8_t val) {
    ::digitalWrite(pin, val);
}

uint8_t Hal::digitalRead(uint8_t pin) {
    return (uint8_t)::digitalRead(pin);
}

void Hal::setPwm(uint8_t pin, uint8_t val) {
#if (BOARD_TYPE == BOARD_ESP32)
    ledcWrite(pin, val);
#else
    ::analogWrite(pin, val);
#endif
}

uint32_t Hal::millis() {
    return ::millis();
}

uint32_t Hal::micros() {
    return ::micros();
}

void Hal::delay(uint32_t ms) {
    ::delay(ms);
}

void Hal::disableInterrupts() {
    noInterrupts();
}

void Hal::enableInterrupts() {
    interrupts();
}

uint8_t Hal::eepromRead(uint16_t address) {
    return EEPROM.read(address);
}

void Hal::eepromUpdate(uint16_t address, uint8_t val) {
#if (BOARD_TYPE == BOARD_ESP32)
    if (EEPROM.read(address) != val) {
        EEPROM.write(address, val);
        EEPROM.commit();
    }
#else
    EEPROM.update(address, val);
#endif
}

void Hal::serialBegin(uint32_t baud) {
    Serial.begin(baud);
}

void Hal::serialPrint(const char* str) {
    Serial.print(str);
}

void Hal::serialPrintln(const char* str) {
    Serial.println(str);
}

void Hal::serialPrint(const __FlashStringHelper* fstr) {
    Serial.print(fstr);
}

void Hal::serialPrintln(const __FlashStringHelper* fstr) {
    Serial.println(fstr);
}

void Hal::serialPrintF(const char* format, ...) {
    char buf[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    Serial.print(buf);
}

bool Hal::serialAvailable() {
    return Serial.available() > 0;
}

char Hal::serialRead() {
    return (char)Serial.read();
}

#else

// ==========================================
// Desktop / Unit Test Mock Implementation
// ==========================================
static uint8_t mock_pins[64] = {0};
static uint8_t mock_pwm[64]  = {0};
static uint8_t mock_eeprom[1024] = {0};
static uint32_t mock_time_ms = 0;
static uint32_t mock_time_us = 0;
static bool mock_interrupts_enabled = true;
static char mock_serial_rx[512] = {0};
static size_t mock_serial_rx_head = 0;
static size_t mock_serial_rx_tail = 0;
static char mock_serial_tx[1024] = {0};
static size_t mock_serial_tx_len = 0;

void Hal::init() {
    resetMock();
}

uint8_t Hal::eepromRead(uint16_t address) {
    if (address < 1024) return mock_eeprom[address];
    return 0xFF;
}

void Hal::eepromUpdate(uint16_t address, uint8_t val) {
    if (address < 1024 && mock_eeprom[address] != val) {
        mock_eeprom[address] = val;
    }
}

void Hal::pinMode(uint8_t pin, uint8_t mode) {
    (void)pin;
    (void)mode;
}

void Hal::digitalWrite(uint8_t pin, uint8_t val) {
    if (pin < 64) {
        mock_pins[pin] = val;
    }
}

uint8_t Hal::digitalRead(uint8_t pin) {
    if (pin < 64) {
        return mock_pins[pin];
    }
    return 0;
}

void Hal::setPwm(uint8_t pin, uint8_t val) {
    if (pin < 64) {
        mock_pwm[pin] = val;
    }
}

uint32_t Hal::millis() {
    return mock_time_ms;
}

uint32_t Hal::micros() {
    return mock_time_us;
}

void Hal::delay(uint32_t ms) {
    mock_time_ms += ms;
    mock_time_us += ms * 1000;
}

void Hal::disableInterrupts() {
    mock_interrupts_enabled = false;
}

void Hal::enableInterrupts() {
    mock_interrupts_enabled = true;
}

void Hal::serialBegin(uint32_t baud) {
    (void)baud;
}

void Hal::serialPrint(const char* str) {
    if (str != NULL && mock_serial_tx_len < sizeof(mock_serial_tx) - 1) {
        snprintf(mock_serial_tx + mock_serial_tx_len, sizeof(mock_serial_tx) - mock_serial_tx_len, "%s", str);
        mock_serial_tx_len = strlen(mock_serial_tx);
    }
}

void Hal::serialPrintln(const char* str) {
    if (str != NULL && mock_serial_tx_len < sizeof(mock_serial_tx) - 3) {
        snprintf(mock_serial_tx + mock_serial_tx_len, sizeof(mock_serial_tx) - mock_serial_tx_len, "%s\r\n", str);
        mock_serial_tx_len = strlen(mock_serial_tx);
    }
}

void Hal::serialPrintF(const char* format, ...) {
    if (format != NULL && mock_serial_tx_len < sizeof(mock_serial_tx) - 1) {
        va_list args;
        va_start(args, format);
        vsnprintf(mock_serial_tx + mock_serial_tx_len, sizeof(mock_serial_tx) - mock_serial_tx_len, format, args);
        va_end(args);
        mock_serial_tx_len = strlen(mock_serial_tx);
    }
}

bool Hal::serialAvailable() {
    return (mock_serial_rx_tail > mock_serial_rx_head);
}

char Hal::serialRead() {
    if (mock_serial_rx_tail > mock_serial_rx_head) {
        return mock_serial_rx[mock_serial_rx_head++];
    }
    return 0;
}

void Hal::injectMockSerial(const char* data) {
    if (data == NULL) return;
    while (*data && mock_serial_rx_tail < sizeof(mock_serial_rx) - 1) {
        mock_serial_rx[mock_serial_rx_tail++] = *data++;
    }
    mock_serial_rx[mock_serial_rx_tail] = '\0';
}

const char* Hal::getMockSerialOutput() {
    return mock_serial_tx;
}

void Hal::clearMockSerialOutput() {
    mock_serial_tx[0] = '\0';
    mock_serial_tx_len = 0;
}

uint8_t Hal::getPin(uint8_t pin) {
    if (pin < 64) return mock_pins[pin];
    return 0;
}

uint8_t Hal::getPwm(uint8_t pin) {
    if (pin < 64) return mock_pwm[pin];
    return 0;
}

void Hal::setMockPin(uint8_t pin, uint8_t val) {
    if (pin < 64) mock_pins[pin] = val;
}

void Hal::setMockTimeMs(uint32_t ms) {
    mock_time_ms = ms;
    mock_time_us = ms * 1000;
}

void Hal::setMockTimeUs(uint32_t us) {
    mock_time_us = us;
    mock_time_ms = us / 1000;
}

void Hal::advanceTimeMs(uint32_t ms) {
    mock_time_ms += ms;
    mock_time_us += ms * 1000;
}

void Hal::setMockEeprom(uint16_t address, uint8_t val) {
    if (address < 1024) mock_eeprom[address] = val;
}

uint8_t Hal::getMockEeprom(uint16_t address) {
    if (address < 1024) return mock_eeprom[address];
    return 0xFF;
}

void Hal::resetMock() {
    for (int i = 0; i < 64; ++i) {
        mock_pins[i] = 0;
        mock_pwm[i] = 0;
    }
    for (int i = 0; i < 1024; ++i) {
        mock_eeprom[i] = 0xFF;
    }
    mock_time_ms = 0;
    mock_time_us = 0;
    mock_interrupts_enabled = true;
    mock_serial_rx_head = 0;
    mock_serial_rx_tail = 0;
    mock_serial_rx[0] = '\0';
    mock_serial_tx_len = 0;
    mock_serial_tx[0] = '\0';
}

#endif
