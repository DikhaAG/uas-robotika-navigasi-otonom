#ifndef HAL_H
#define HAL_H

#include "Config.h"
#include <stdint.h>

#ifdef ARDUINO
  #include <Arduino.h>
#else
  // Desktop / unit test mock definitions
  #define LOW  0
  #define HIGH 1
  #define INPUT 0
  #define OUTPUT 1
  #define INPUT_PULLUP 2
  #define RISING 2
  #define FALLING 3
  #define CHANGE 4
#endif

class Hal {
public:
    static void init();
    static void pinMode(uint8_t pin, uint8_t mode);
    static void digitalWrite(uint8_t pin, uint8_t val);
    static uint8_t digitalRead(uint8_t pin);
    static void setPwm(uint8_t pin, uint8_t val);

    static uint32_t millis();
    static uint32_t micros();
    static void delay(uint32_t ms);

    static void noInterrupts();
    static void interrupts();

    // Non-volatile EEPROM storage
    static uint8_t eepromRead(uint16_t address);
    static void eepromUpdate(uint16_t address, uint8_t val);

    // Serial & Telemetry
    static void serialBegin(uint32_t baud);
    static void serialPrint(const char* str);
    static void serialPrintln(const char* str);
#ifdef ARDUINO
    static void serialPrint(const __FlashStringHelper* fstr);
    static void serialPrintln(const __FlashStringHelper* fstr);
#endif
    static void serialPrintF(const char* format, ...);
    static bool serialAvailable();
    static char serialRead();

#ifndef ARDUINO
    // Mock inspection methods for desktop TDD
    static uint8_t getPin(uint8_t pin);
    static uint8_t getPwm(uint8_t pin);
    static void setMockPin(uint8_t pin, uint8_t val);
    static void setMockTimeMs(uint32_t ms);
    static void setMockTimeUs(uint32_t us);
    static void advanceTimeMs(uint32_t ms);
    static void setMockEeprom(uint16_t address, uint8_t val);
    static uint8_t getMockEeprom(uint16_t address);
    static void resetMock();
#endif
};

#endif // HAL_H
