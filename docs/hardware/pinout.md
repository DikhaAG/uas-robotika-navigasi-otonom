# Pemetaan Pin Hardware (Pinout Specification)

Dokumentasi pemetaan pin mikrokontroler untuk robot navigasi otonom (Praktikum Robotika TC253421, D3 Teknik Komputer Polsri).

## 1. Pemetaan Pin Arduino Uno R3 (Baseline Evaluasi)

| Pin Arduino Uno | Terhubung Ke | Fungsi / Sinyal | Tipe I/O | Keterangan & Catatan Sambungan |
| :--- | :--- | :--- | :--- | :--- |
| **D9** | L293D Pin 1 (1,2EN) | PWM Motor Kiri | Output (PWM Timer1) | Mengatur kecepatan motor kiri (0–255) |
| **D8** | L293D Pin 2 (1A) | Arah Motor Kiri (IN1) | Output Digital | Logika arah maju/mundur motor kiri (via resistor 220 $\Omega$) |
| **D7** | L293D Pin 7 (2A) | Arah Motor Kiri (IN2) | Output Digital | Logika arah maju/mundur motor kiri (via resistor 220 $\Omega$) |
| **D6** | L293D Pin 9 (3,4EN) | PWM Motor Kanan | Output (PWM Timer0) | Mengatur kecepatan motor kanan (0–255) |
| **D5** | L293D Pin 10 (3A) | Arah Motor Kanan (IN3) | Output Digital | Logika arah maju/mundur motor kanan (via resistor 220 $\Omega$) |
| **D4** | L293D Pin 15 (4A) | Arah Motor Kanan (IN4) | Output Digital | Logika arah maju/mundur motor kanan (via resistor 220 $\Omega$) |
| **D2 (INT0)** | Modul LM393 Kiri (D0) | Pulsa Encoder Kiri | Input Interrupt | Interupsi eksternal INT0 (`CHANGE` + filter glitch), pulsa rotasi roda kiri |
| **D3 (INT1)** | Modul LM393 Kanan (D0) | Pulsa Encoder Kanan | Input Interrupt | Interupsi eksternal INT1 (`CHANGE` + filter glitch), pulsa rotasi roda kanan |
| **A0 (D14)** | HC-SR04 Trig | Trigger Ultrasonik | Output Digital | Pulsa trigger 10 $\mu\text{s}$ untuk sonar depan |
| **A1 (D15)** | HC-SR04 Echo | Echo Ultrasonik | Input Digital | Pulsa durasi pantulan gelombang (timeout 25 ms $\approx 400\text{ cm}$) |
| **A4 (SDA)** | MPU6050 SDA | I2C Data | Bidireksional (I2C) | Komunikasi I2C IMU (Alamat register 0x68) |
| **A5 (SCL)** | MPU6050 SCL | I2C Clock | Output (I2C) | Clock I2C 100 kHz/400 kHz |
| **A2 (D16)** | Buzzer Aktif (+) | Sinyal Audio | Output Digital | Sinyal beeper tiba (3x beep) dan failsafe (2 detik kontinu) |
| **D12** | Push Button START | Tombol Mulai Navigasi | Input (`INPUT_PULLUP`) | Dihubungkan ke GND saat ditekan, memulai kalibrasi & navigasi |
| **D13** | Built-in LED | Indikator State | Output Digital | Kedip lambat (IDLE), solid (Navigasi/TIBA), kedip cepat (GAGAL) |
| **A3 (D17)** | FC-51 Kiri (OUT) | Sensor Dinding Kiri | Input Digital | Deteksi batas dinding koridor sisi kiri (Active LOW) |
| **D11** | FC-51 Kanan (OUT) | Sensor Dinding Kanan | Input Digital | Deteksi batas dinding koridor sisi kanan (Active LOW) |
| **D0 (RX)** | HC-05 TXD *(opsional)* | Bluetooth → Arduino | Hardware UART | Berbagi `Serial` UART0 dengan USB pada `SERIAL_BAUD` = 9600 (baud bawaan HC-05); lepas dari `D0` setiap kali memakai kabel USB (upload, Serial Monitor, wizard) |
| **D1 (TX)** | HC-05 RXD *(opsional)* | Arduino → Bluetooth | Hardware UART | Lewat pembagi tegangan 1 kΩ / 2 kΩ (RXD HC-05 bekerja di 3.3 V) |

---

## 2. Pemetaan Pin Portabilitas ESP32 DevKit v1 (ADR-0002)

| Pin ESP32 | Terhubung Ke | Fungsi / Sinyal | Tipe I/O | Keterangan & Catatan Sambungan |
| :--- | :--- | :--- | :--- | :--- |
| **GPIO 25** | L293D Pin 1 (1,2EN) | PWM Motor Kiri | Output LEDC Ch 0 | Frekuensi 1 kHz, resolusi 8-bit |
| **GPIO 26** | L293D Pin 2 (1A) | Arah Motor Kiri (IN1) | Output Digital | Logika arah motor kiri |
| **GPIO 27** | L293D Pin 7 (2A) | Arah Motor Kiri (IN2) | Output Digital | Logika arah motor kiri |
| **GPIO 14** | L293D Pin 9 (3,4EN) | PWM Motor Kanan | Output LEDC Ch 1 | Frekuensi 1 kHz, resolusi 8-bit |
| **GPIO 12** | L293D Pin 10 (3A) | Arah Motor Kanan (IN3) | Output Digital | Logika arah motor kanan |
| **GPIO 13** | L293D Pin 15 (4A) | Arah Motor Kanan (IN4) | Output Digital | Logika arah motor kanan |
| **GPIO 34** | Modul LM393 Kiri (D0) | Pulsa Encoder Kiri | Input Interrupt | GPIO input-only, interupsi pulsa roda kiri |
| **GPIO 35** | Modul LM393 Kanan (D0) | Pulsa Encoder Kanan | Input Interrupt | GPIO input-only, interupsi pulsa roda kanan |
| **GPIO 5** | HC-SR04 Trig | Trigger Ultrasonik | Output Digital | Pulsa trigger 10 $\mu\text{s}$ |
| **GPIO 18** | HC-SR04 Echo | Echo Ultrasonik | Input Digital | Gunakan voltage divider (5V ke 3.3V) pada sinyal Echo! |
| **GPIO 21** | MPU6050 SDA | I2C Data | Bidireksional (I2C) | Default hardware I2C SDA pada ESP32 |
| **GPIO 22** | MPU6050 SCL | I2C Clock | Output (I2C) | Default hardware I2C SCL pada ESP32 |
| **GPIO 4** | Buzzer Aktif (+) | Sinyal Audio | Output Digital | Kontrol buzzer aktif 5V (via transistor 2N2222) |
| **GPIO 15** | Push Button START | Tombol Mulai | Input (`INPUT_PULLUP`) | Pushbutton ke GND |
| **GPIO 2** | Built-in Blue LED | Indikator State | Output Digital | LED indikator onboard ESP32 |
| **GPIO 32** | FC-51 Kiri (OUT) | Dinding Kiri | Input Digital | Proximity dinding kiri |
| **GPIO 33** | FC-51 Kanan (OUT) | Dinding Kanan | Input Digital | Proximity dinding kanan |
| **GPIO 16** | HC-05 TXD *(cadangan)* | Serial2 RX *(cadangan)* | Hardware UART | Pin cadangan di `Config.h`; firmware aktif memakai `Serial` UART0 |
| **GPIO 17** | HC-05 RXD *(cadangan)* | Serial2 TX *(cadangan)* | Hardware UART | Pin cadangan di `Config.h`; firmware aktif memakai `Serial` UART0 |
