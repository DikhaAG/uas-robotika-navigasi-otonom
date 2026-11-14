#include <cassert>
#include <iostream>
#include <cmath>
#include "../firmware/RobotNavigasiOtonom/Config.h"
#include "../firmware/RobotNavigasiOtonom/Hal.h"
#include "../firmware/RobotNavigasiOtonom/Motors.h"
#include "../firmware/RobotNavigasiOtonom/CalibrationProfile.h"

int main() {
    std::cout << "[TDD] Running test_calibration_profile..." << std::endl;

    // Reset HAL mock (EEPROM initialized to 0xFF)
    Hal::resetMock();

    // Test 1: Default initialization
    CalibrationProfile profile;
    profile.resetToDefaults();
    assert(std::fabs(profile.getDiameterRodaMm() - DIAMETER_RODA_MM) < 0.001f);
    assert(std::fabs(profile.getTrimMotorKiri() - TRIM_MOTOR_KIRI) < 0.001f);
    assert(std::fabs(profile.getTrimMotorKanan() - TRIM_MOTOR_KANAN) < 0.001f);
    assert(std::fabs(profile.getGyroBiasZ() - 0.0f) < 0.001f);
    assert(profile.getInvertMotorKiri() == 1);
    assert(profile.getInvertMotorKanan() == 1);
    assert(profile.isValid());
    std::cout << "  ✓ Test 1: Default initialization passed." << std::endl;

    // Test 2: Loading from uninitialized EEPROM (all 0xFF) fails and falls back to defaults
    bool loaded = profile.load();
    assert(!loaded);
    assert(std::fabs(profile.getDiameterRodaMm() - DIAMETER_RODA_MM) < 0.001f);
    assert(profile.getInvertMotorKiri() == 1);
    assert(profile.getInvertMotorKanan() == 1);
    std::cout << "  ✓ Test 2: Uninitialized EEPROM fallback passed." << std::endl;

    // Test 3: Mutating values, saving to EEPROM, and reloading
    profile.setDiameterRodaMm(66.5f);
    profile.setTrimMotorKiri(0.96f);
    profile.setTrimMotorKanan(1.04f);
    profile.setGyroBiasZ(-0.025f);
    profile.setInvertMotorKiri(-1);
    profile.setInvertMotorKanan(1);

    bool saved = profile.save();
    assert(saved);

    // Verify EEPROM raw bytes at address 0 contain magic signature 0xCA1B (little endian: 0x1B, 0xCA)
    uint8_t magicLow = Hal::getMockEeprom(0);
    uint8_t magicHigh = Hal::getMockEeprom(1);
    uint16_t magic = ((uint16_t)magicHigh << 8) | magicLow;
    assert(magic == 0xCA1B);
    assert(Hal::getMockEeprom(2) == 0x01); // Version 0x01

    // Create a new fresh profile object and load
    CalibrationProfile loadedProfile;
    loaded = loadedProfile.load();
    assert(loaded);
    assert(loadedProfile.isValid());
    assert(std::fabs(loadedProfile.getDiameterRodaMm() - 66.5f) < 0.001f);
    assert(std::fabs(loadedProfile.getTrimMotorKiri() - 0.96f) < 0.001f);
    assert(std::fabs(loadedProfile.getTrimMotorKanan() - 1.04f) < 0.001f);
    assert(std::fabs(loadedProfile.getGyroBiasZ() - (-0.025f)) < 0.001f);
    assert(loadedProfile.getInvertMotorKiri() == -1);
    assert(loadedProfile.getInvertMotorKanan() == 1);
    std::cout << "  ✓ Test 3: Save and reload valid profile passed." << std::endl;

    // Test 4: Corruption rejection (tamper with checksum or payload byte)
    // Mutate a byte in EEPROM
    Hal::setMockEeprom(5, Hal::getMockEeprom(5) ^ 0xFF);
    CalibrationProfile corruptProfile;
    bool corruptLoaded = corruptProfile.load();
    assert(!corruptLoaded);
    // Should fall back to default values
    assert(std::fabs(corruptProfile.getDiameterRodaMm() - DIAMETER_RODA_MM) < 0.001f);
    assert(corruptProfile.getInvertMotorKiri() == 1);
    std::cout << "  ✓ Test 4: Checksum corruption rejection passed." << std::endl;

    // Test 5: Factory reset clears EEPROM magic signature
    // Resave a valid profile first
    assert(profile.save());
    assert(loadedProfile.load());
    // Execute reset
    profile.resetToDefaults();
    profile.clearEeprom();
    // Now loading should fail because magic is cleared
    CalibrationProfile resetProfile;
    assert(!resetProfile.load());
    std::cout << "  ✓ Test 5: Factory reset clears EEPROM passed." << std::endl;

    // Test 5b: Range validation in isValid() (trim 0.70-1.30, finite gyro bias)
    CalibrationProfile rangeProfile;
    rangeProfile.resetToDefaults();
    assert(rangeProfile.isValid());
    rangeProfile.setTrimMotorKiri(0.65f); // Out of spec (< 0.70)
    assert(!rangeProfile.isValid());
    rangeProfile.setTrimMotorKiri(1.0f);
    rangeProfile.setTrimMotorKanan(1.35f); // Out of spec (> 1.30)
    assert(!rangeProfile.isValid());
    rangeProfile.setTrimMotorKanan(1.0f);
    rangeProfile.setGyroBiasZ(100.0f); // Unreasonable gyro bias
    assert(!rangeProfile.isValid());
    rangeProfile.setGyroBiasZ(0.0f);
    assert(rangeProfile.isValid());
    std::cout << "  ✓ Test 5b: Range validation in isValid() passed." << std::endl;

    // Test 6: Software motor polarity inversion integration
    Motors::init();
    // Default polarity (+1, +1): Positive PWM -> IN1=1, IN2=0
    Motors::setMotor(170, 170);
    assert(Hal::getPin(PIN_IN1) == 1);
    assert(Hal::getPin(PIN_IN2) == 0);
    assert(Hal::getPin(PIN_IN3) == 1);
    assert(Hal::getPin(PIN_IN4) == 0);

    // Apply inversion: Left inverted (-1), Right normal (+1)
    Motors::setInversion(-1, 1);
    assert(Motors::getInvertLeft() == -1);
    assert(Motors::getInvertRight() == 1);

    // Positive PWM command: Left motor pins should be inverted (IN1=0, IN2=1)
    // while right motor remains normal (IN3=1, IN4=0)
    Motors::setMotor(170, 170);
    assert(Hal::getPin(PIN_IN1) == 0);
    assert(Hal::getPin(PIN_IN2) == 1);
    assert(Hal::getPin(PIN_IN3) == 1);
    assert(Hal::getPin(PIN_IN4) == 0);
    assert(Motors::getDirectionLeft() == 1);
    assert(Motors::getDirectionRight() == 1);

    // Negative PWM command: Left motor pins inverted (IN1=1, IN2=0)
    Motors::setMotor(-170, -170);
    assert(Hal::getPin(PIN_IN1) == 1);
    assert(Hal::getPin(PIN_IN2) == 0);
    assert(Hal::getPin(PIN_IN3) == 0);
    assert(Hal::getPin(PIN_IN4) == 1);
    assert(Motors::getDirectionLeft() == -1);
    assert(Motors::getDirectionRight() == -1);

    std::cout << "  ✓ Test 6: Software motor polarity inversion passed." << std::endl;

    std::cout << "[TDD] ALL test_calibration_profile passed successfully!" << std::endl;
    return 0;
}
