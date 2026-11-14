#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

// ==========================================
// 1. Target Board Profile Selection (ADR-0002)
// ==========================================
#define BOARD_ARDUINO_UNO 1
#define BOARD_ESP32       2

#ifndef BOARD_TYPE
#define BOARD_TYPE BOARD_ARDUINO_UNO
#endif

// ==========================================
// 2. Microcontroller Pinout Mapping
// ==========================================
#if (BOARD_TYPE == BOARD_ARDUINO_UNO)
  // L293D Dual H-Bridge Motor Driver
  #define PIN_ENA 9   // Timer1 PWM motor kiri (L293D pin 1)
  #define PIN_IN1 8   // Arah motor kiri (L293D pin 2)
  #define PIN_IN2 7   // Arah motor kiri (L293D pin 7)
  #define PIN_IN3 5   // Arah motor kanan (L293D pin 10)
  #define PIN_IN4 4   // Arah motor kanan (L293D pin 15)
  #define PIN_ENB 6   // Timer0 PWM motor kanan (L293D pin 9)

  // Wheel Speed Optical Encoders (LM393 FC-03)
  #define PIN_ENC_KIRI  2  // Hardware Interrupt INT0
  #define PIN_ENC_KANAN 3  // Hardware Interrupt INT1

  // Ultrasonic Distance Sensor (HC-SR04)
  #define PIN_TRIG 14  // Analog A0
  #define PIN_ECHO 15  // Analog A1

  // User Interface & Indicators
  #define PIN_BUZZER       16  // Analog A2
  #define PIN_BUTTON_START 12  // Digital D12 (INPUT_PULLUP)
  #define PIN_LED          13  // Digital D13 (Built-in status LED)

  // Lateral Corridor Centering IR Proximity Sensors (FC-51)
  #define PIN_IR_KIRI  17  // Analog A3
  #define PIN_IR_KANAN 11  // Digital D11

  // Optional Bluetooth Telemetry (HC-05 via SoftwareSerial, if D11 free or using Alt pins)
  #define PIN_BT_RX 10
  #define PIN_BT_TX 12     // Shared/Alt pin (D10 RX, D12 / SoftwareSerial)

  // I2C IMU MPU6050
  #define PIN_SDA 18  // Analog A4
  #define PIN_SCL 19  // Analog A5

#elif (BOARD_TYPE == BOARD_ESP32)
  // L293D Motor Pins on ESP32 DevKit v1
  #define PIN_ENA 25  // LEDC PWM Channel 0
  #define PIN_IN1 26
  #define PIN_IN2 27
  #define PIN_IN3 12
  #define PIN_IN4 13
  #define PIN_ENB 14  // LEDC PWM Channel 1

  #define PIN_ENC_KIRI  34
  #define PIN_ENC_KANAN 35

  #define PIN_TRIG 5
  #define PIN_ECHO 18

  #define PIN_BUZZER       4
  #define PIN_BUTTON_START 15
  #define PIN_LED          2

  #define PIN_IR_KIRI  32
  #define PIN_IR_KANAN 33

  #define PIN_BT_RX 16
  #define PIN_BT_TX 17

  #define PIN_SDA 21
  #define PIN_SCL 22
#endif

// ==========================================
// 3. Physical Chassis Geometry & Constants
// ==========================================
#ifndef PI
#define PI 3.14159265358979323846f
#endif

#define DIAMETER_RODA_MM     65.0f
#define JUMLAH_SLOT_ENCODER  20
#define JARAK_RODA_MM        135.0f

#define KELILING_RODA_MM     (PI * DIAMETER_RODA_MM)
#define MM_PER_PULSA         (KELILING_RODA_MM / (float)JUMLAH_SLOT_ENCODER)
#define CM_PER_PULSA         (MM_PER_PULSA / 10.0f)
#define TRACK_WIDTH_CM       (JARAK_RODA_MM / 10.0f)

// Gyroscope configuration
#define MPU6050_I2C_ADDR     0x68
#define GYRO_SCALE_250DPS    131.0f  // LSB / (deg/s)
#define GYRO_SIGN            1.0f    // +1 or -1 based on mounting orientation

// ==========================================
// 4. Motion Control & Speed Settings
// ==========================================
#define PWM_BASE             170
#define PWM_SLOW             120
#define PWM_TURN             140
#define KP_HEADING           3.0f

// Trimming / balancing kecepatan motor kiri dan kanan jika terdapat deviasi mekanik
#define TRIM_MOTOR_KIRI      1.0f  // Kalibrasi pengali motor kiri (contoh: 0.95 atau 1.05)
#define TRIM_MOTOR_KANAN     1.0f  // Kalibrasi pengali motor kanan (contoh: 0.95 atau 1.05)

#define JARAK_DECEL_CM       30.0f
#define TOL_WP_CM            5.0f
#define TOL_AKHIR_CM         2.0f
#define TOL_ORIENTASI_DEG     8.0f
#define AMBANG_BELOK_DEG     25.0f

// ==========================================
// 5. Obstacle Avoidance & Corridor Parameters
// ==========================================
#define JARAK_RINTANGAN_CM   25.0f
#define SUDUT_PINDAI_DEG     30.0f
#define JARAK_LOLOS_MIN_CM   30.0f
#define JARAK_MUNDUR_CM      15.0f
#define LANE_OFFSET_Y_CM     20.0f
#define CORRIDOR_HALF_WIDTH  40.0f

// ==========================================
// 6. Timing & Failsafe Parameters
// ==========================================
#define PERIODE_ODOMETRI_MS  20UL
#define PERIODE_SONAR_MS     60UL
#define PERIODE_TELEMETRI_MS 200UL
#define TIMEOUT_GLOBAL_MS    90000UL
#define TIMEOUT_STALL_MS     1000UL
#define MAX_RETRIES_ESCAPE   3
#define BATAS_KORIDOR_Y_CM   40.0f
#define BATAS_KORIDOR_X_MIN -30.0f

// ==========================================
// 7. Room Coordinate Waypoints (Arena Reference)
// ==========================================
// Room 1 (Right): Doorway (180, 0), Center (180, 70)
// Room 2 (Right): Doorway (295, 0), Center (295, 70)
// Room 3 (Left):  Doorway (235, 0), Center (235, -70)
// Room 4 (Left):  Doorway (355, 0), Center (355, -70)
static const float ROOM_WP1[4][2] = {
    {180.0f, 0.0f},
    {295.0f, 0.0f},
    {235.0f, 0.0f},
    {355.0f, 0.0f}
};

static const float ROOM_WP2[4][2] = {
    {180.0f, 70.0f},
    {295.0f, 70.0f},
    {235.0f, -70.0f},
    {355.0f, -70.0f}
};

#endif // CONFIG_H
