# Panduan Perakitan Skematik Perkabelan (Anti-Salah)

Panduan perakitan kabel langkah-demi-langkah untuk **Robot Navigasi Otonom (Arduino Uno R3 + Modul Driver L298N)**, disusun agar mahasiswa dapat merakit setiap komponen secara berurutan tanpa kesalahan polaritas, salah pin, maupun *ground loop*.

---

## Diagram Skematik Lengkap

![Skematik Perkabelan Arduino Uno R3 + L298N](wiring_schematic_uno_l298n.svg)

> **Model 3D:** lihat rangkaian ini dalam 3D (41 kabel pada pin yang benar) di [renders/robot_iso.png](renders/robot_iso.png), [tampak atas](renders/robot_top.png), dan [tampak bawah](renders/robot_bottom.png). Cara membangunnya ada di [tools/README.md](../../tools/README.md#4-model-3d-rangkaian-blender-blender).

> **Tips Membaca Diagram:**
> - **Merah (`+VBAT`)**: Daya baterai (2S `7.4V` atau 3S `11.1V`) menuju `VIN` Arduino dan `+12V` L298N.
> - **Oranye (`+5V`)**: Suplai logika 5V dari pin `5V` Arduino menuju semua sensor.
> - **Hitam (`GND`)**: Semua jalur ground wajib bertemu di satu titik (**Star Ground**).
> - **Biru**: Sinyal PWM dan arah motor (`D9`–`D4`) ke L298N.
> - **Hijau / Ungu / Biru Muda**: Sinyal sensor (`HC-SR04`, `LM393`, `FC-51`), bus I2C (`MPU6050`), dan antarmuka (`START` & `Buzzer`).

---

## Urutan Perakitan 5 Tahap (Rakit Sesuai Urutan Ini!)

### Tahap 1: Siapkan Jalur Daya & *Star Ground* (Switch dalam posisi OFF!)
> **Peringatan:** Pastikan baterai belum terhubung atau sakelar utama dalam keadaan **OFF** selama proses perakitan.

> **Baterai:** Li-Ion 18650 **2S** (`7.4 V` nominal, `8.4 V` penuh) **atau 3S** (`11.1 V` nominal, `12.6 V` penuh). Samakan `TEGANGAN_BATERAI_V` di [`Config.h`](../../firmware/RobotNavigasiOtonom/Config.h) (default `11.1f` = 3S, `7.4f` = 2S) agar PWM motor dikompensasi otomatis.

![Visualisasi Tahap 1: Jalur Daya & Star Ground](wiring_stage1_power_starground.svg)

1. Siapkan satu titik terminal bersama (misalnya rel `-` breadboard mini atau terminal blok) sebagai **Star Ground Hub**.
2. Hubungkan kabel **Hitam (`-`) Baterai** ke **Star Ground Hub**.
3. Hubungkan kabel **Merah (`+`) Baterai** ke kaki `IN` **Switch ON/OFF Utama**.
4. Dari kaki `OUT` **Switch ON/OFF**, cabangkan dua kabel merah:
   - Cabang 1 $\rightarrow$ pin **`VIN`** Arduino Uno.
   - Cabang 2 $\rightarrow$ terminal **`+12V` (VCC)** modul **L298N**.
   - *(Baterai 3S — sangat disarankan)* Sisipkan **buck converter** (LM2596 / mini-360) yang diset **`7.5 V`** pada Cabang 1: `OUT` switch $\rightarrow$ `IN+` buck, `OUT+` buck $\rightarrow$ `VIN` Arduino, `IN-`/`OUT-` buck $\rightarrow$ **Star Ground Hub**. Tanpa buck, regulator onboard Arduino harus menurunkan `12.6 V` ke `5 V` untuk seluruh sensor (±1.5 W) sehingga bisa kepanasan dan me-reset Arduino. Pada baterai 2S, Cabang 1 langsung ke `VIN`.
5. Hubungkan pin **`GND`** Arduino Uno dan terminal **`GND`** modul **L298N** ke **Star Ground Hub**.
6. Siapkan satu rel `+` terpisah sebagai **Bus `+5V` Logika**, lalu hubungkan ke pin **`5V`** Arduino Uno.
7. **Aturan Jumper `5V-EN` L298N:** Jumper ini berupa 2 pin berdiri + tutup plastik kecil **tepat di belakang terminal sekrup biru `12V | GND | 5V`**, dekat regulator kecil `78M05` (kadang bertuliskan `5V-EN`/`5V_EN`, sering tanpa tulisan; jangan tertukar dengan jumper `ENA`/`ENB` di ujung baris pin `IN`).
   - **Biarkan terpasang** (aman untuk 2S maupun 3S): regulator onboard menyalakan logika L298N dan terminal `+5V` L298N menjadi *output*.
   - **Biarkan pin `+5V` L298N kosong (tidak disambung ke `5V` Arduino)** agar regulator onboard L298N dan Arduino tidak saling bertabrakan.
   - Jika jumper hilang, pakai salah satu jumper `ENA`/`ENB` yang dicabut di Tahap 2. Tanpa jumper ini logika L298N tidak mendapat daya dan motor tidak berputar sama sekali.
   - Uji (sakelar ON): LED merah modul L298N menyala, dan terminal `+5V`–`GND` L298N terukur ±5 V.

---

### Tahap 2: Pasang Driver Motor L298N & Kedua Motor DC

![Visualisasi Tahap 2: Driver Motor L298N & Kedua Motor DC](wiring_stage2_l298n_motors.svg)

1. **WAJIB:** Cabut/lepas kedua *jumper cap* hitam pada pin **`ENA`** dan **`ENB`** di modul L298N.
2. Sambungkan 6 kabel sinyal kontrol dari Arduino Uno ke L298N:

| Pin Arduino Uno | Pin Modul L298N | Fungsi |
| :---: | :---: | :--- |
| **`D9`** | **`ENA`** | Kecepatan (PWM) Motor **Kiri** |
| **`D8`** | **`IN1`** | Arah Motor **Kiri** |
| **`D7`** | **`IN2`** | Arah Motor **Kiri** |
| **`D6`** | **`ENB`** | Kecepatan (PWM) Motor **Kanan** |
| **`D5`** | **`IN3`** | Arah Motor **Kanan** |
| **`D4`** | **`IN4`** | Arah Motor **Kanan** |

3. Sambungkan kabel **Motor DC Kiri** ke terminal **`OUT1`** & **`OUT2`**, dan **Motor DC Kanan** ke terminal **`OUT3`** & **`OUT4`**.
4. *(Sangat Disarankan, tidak wajib untuk modul L298N)* Solder **kapasitor keramik `100 nF` (kode `104`)** melintang di antara dua terminal tembaga masing-masing motor DC untuk meredam percikan EMI agar pembacaan MPU6050 stabil. Modul L298N sudah memiliki kapasitor elektrolit di jalur dayanya dan firmware menyaring derau pulsa encoder, sehingga robot tetap bisa berjalan tanpa kapasitor ini; pasang bila Arduino sering *restart* atau MPU6050 sering macet saat motor berputar.

*(Catatan: Jika Anda menggunakan **IC L293D** alih-alih modul L298N, pin Arduino `D9–D4` tetap sama persis; lihat tabel padanan di [pinout.md](pinout.md) dan pasang resistor seri `220 Ω` pada `IN1–IN4` serta kapasitor `220 µF` pada `VCC2`.)*

---

### Tahap 3: Pasang Kedua Encoder Roda (LM393 / FC-03)

![Visualisasi Tahap 3: Kedua Encoder Roda (LM393 / FC-03)](wiring_stage3_encoders.svg)

1. **Encoder Kiri** (dipasang pada cakram 20-slot roda kiri):
   - `VCC` $\rightarrow$ Bus **`+5V`**
   - `GND` $\rightarrow$ **Star Ground Hub**
   - `D0` $\rightarrow$ Pin **`D2` (INT0)** Arduino Uno
2. **Encoder Kanan** (dipasang pada cakram 20-slot roda kanan):
   - `VCC` $\rightarrow$ Bus **`+5V`**
   - `GND` $\rightarrow$ **Star Ground Hub**
   - `D0` $\rightarrow$ Pin **`D3` (INT1)** Arduino Uno
3. Pin `A0` pada kedua modul encoder **tidak dipakai** (biarkan kosong). Putar roda pelan dengan tangan: LED indikator modul harus berkedip.

---

### Tahap 4: Pasang Sensor Navigasi (HC-SR04, MPU6050, & FC-51)

![Visualisasi Tahap 4: Sensor Navigasi (HC-SR04, MPU6050, & FC-51)](wiring_stage4_sensors.svg)

1. **Sensor Ultrasonik HC-SR04 (Depan):**
   - `VCC` $\rightarrow$ Bus **`+5V`**
   - `Trig` $\rightarrow$ Pin **`A0`** Arduino Uno
   - `Echo` $\rightarrow$ Pin **`A1`** Arduino Uno
   - `GND` $\rightarrow$ **Star Ground Hub**
2. **Sensor IMU MPU6050 / Modul GY-521 (I2C):**
   - `VCC` $\rightarrow$ Bus **`+5V`**
   - `GND` $\rightarrow$ **Star Ground Hub**
   - `SDA` $\rightarrow$ Pin **`A4`** Arduino Uno
   - `SCL` $\rightarrow$ Pin **`A5`** Arduino Uno
   - `AD0` $\rightarrow$ Biarkan kosong (tidak disambung) atau hubungkan ke `GND` agar alamat I2C tetap **`0x68`**.
   - *(Pemasangan fisik: papan MPU6050 **datar** sejajar sasis (tidak miring, tidak berdiri), dipasang kencang di atas poros roda dengan sumbu X searah depan robot sesuai brief. Firmware hanya membaca gyro sumbu Z, jadi chip boleh menghadap **atas** atau **bawah** dan panah X boleh ke mana saja — pilih orientasinya di wizard `auto_calibrate.py` (U0.1 atau menu [8]) agar tanda gyro tersimpan di EEPROM.)*
3. **Sensor IR Dinding Kiri (FC-51 Kiri):**
   - `VCC` $\rightarrow$ Bus **`+5V`**
   - `OUT` $\rightarrow$ Pin **`A3`** Arduino Uno
   - `GND` $\rightarrow$ **Star Ground Hub**
4. **Sensor IR Dinding Kanan (FC-51 Kanan):**
   - `VCC` $\rightarrow$ Bus **`+5V`**
   - `OUT` $\rightarrow$ Pin **`D11`** Arduino Uno
   - `GND` $\rightarrow$ **Star Ground Hub**

---

### Tahap 5: Pasang Tombol `START` & Buzzer Indikator

![Visualisasi Tahap 5: Tombol START, Buzzer Aktif 5V, & LED Indikator (D13)](wiring_stage5_start_buzzer.svg)

1. **Push Button `START`:**
   - Kaki 1 $\rightarrow$ Pin **`D12`** Arduino Uno
   - Kaki 2 $\rightarrow$ **Star Ground Hub**
   - *(Tidak perlu resistor eksternal karena firmware sudah mengaktifkan `INPUT_PULLUP` internal).*
   - *(Tanpa push button: pasang satu kabel jumper di pin `D12` dan sentuhkan ujungnya ke GND sebentar untuk memulai; firmware cukup membaca level LOW saat `IDLE`.)*
2. **Buzzer Aktif 5V:**
   - Kaki `+` (positif / kabel merah) $\rightarrow$ Pin **`A2`** Arduino Uno
   - Kaki `-` (negatif / kabel hitam) $\rightarrow$ **Star Ground Hub**
3. **LED Indikator State:** Sudah tersedia langsung di papan Arduino Uno (LED `L` bawaan pada pin **`D13`**), tidak perlu kabel tambahan.

---

### Tahap 6 (Opsional): Modul Bluetooth HC-05

> Tahap ini tidak termasuk diagram di atas maupun model 3D. HC-05 berbagi `Serial` (UART0) dengan USB pada `SERIAL_BAUD` = **9600** (baud bawaan pabrik HC-05, sehingga tidak perlu setting mode AT).

1. **Modul Bluetooth HC-05:**
   - `VCC` $\rightarrow$ Bus **`+5V`**
   - `GND` $\rightarrow$ **Star Ground Hub**
   - `TXD` $\rightarrow$ Pin **`D0` (RX)** Arduino Uno
   - `RXD` $\leftarrow$ Pin **`D1` (TX)** Arduino Uno melalui pembagi tegangan: `D1` — `1 kΩ` — titik — `2 kΩ` — GND, lalu titik tengah ke `RXD` (pin `RXD` HC-05 bekerja di 3.3 V).
   - `EN` dan `STATE` tidak dipakai.
2. **Aturan pakai:**
   - **Saat upload firmware:** cabut kabel `TXD` HC-05 dari `D0` (jika tidak, upload gagal `stk500_recv(): programmer is not responding`).
   - **Saat kalibrasi lewat kabel USB:** cabut `VCC` HC-05 agar perintah dari laptop sampai ke Arduino.
   - **Lewat Bluetooth:** USB dicabut, robot dari baterai; pasangkan HC-05 di laptop (PIN bawaan `1234`) lalu gunakan port Bluetooth-nya (macOS: `/dev/cu.<nama-HC-05>`).

---

## Checklist Verifikasi Sebelum Menyalakan Switch

Periksa 8 poin berikut sebelum memindahkan sakelar baterai ke posisi **ON**:

- [ ] **Jumper `ENA` & `ENB` L298N sudah dilepas** sebelum kabel `D9` dan `D6` dipasang.
- [ ] **Jumper `5V-EN` L298N terpasang**, dan **pin `+5V` L298N tidak dihubungkan ke `5V` Arduino**.
- [ ] **`TEGANGAN_BATERAI_V` di `Config.h` sesuai baterai** (`11.1f` untuk 3S, `7.4f` untuk 2S).
- [ ] **Tidak ada kabel `VCC` dan `GND` yang terbalik** (terutama pada Baterai, MPU6050, dan HC-SR04).
- [ ] **Semua kabel `GND` bertemu di satu titik (*Star Ground*)** — baterai, Arduino, L298N, dan semua sensor.
- [ ] **Pin `SDA` (`A4`) dan `SCL` (`A5`) MPU6050 tidak tertukar**, dan pin `AD0` tidak terhubung ke `5V`.
- [ ] **Kabel `TXD` HC-05 dicabut dari `D0` saat upload** (jika memakai HC-05).
- [ ] **Robot diganjal/diangkat** dari lantai sebelum menjalankan pengujian pertama kali (`python3 tools/auto_calibrate.py`).

> **Catatan Jika Arah Putar Roda Terbalik:** Anda **tidak perlu** membongkar kabel `OUT1`–`OUT4` di terminal L298N. Cukup jalankan `python3 tools/auto_calibrate.py` (Stage U0.5), jawab `n` ketika ditanya apakah roda kiri/kanan berputar maju, maka firmware akan otomatis membalik arah motor di EEPROM.

---

## Catatan Jika Menggunakan Target ESP32 DevKit v1
Jika Anda menggunakan **ESP32 DevKit v1** (`#define BOARD_TYPE BOARD_ESP32` di `Config.h`), ikuti alur 5 tahap yang sama dengan penyesuaian pin pada [pinout.md](pinout.md#2-pemetaan-pin-portabilitas-esp32-devkit-v1-adr-0002), serta **wajib** memasang rangkaian *voltage divider* ($1\,\text{k}\Omega$ dan $2\,\text{k}\Omega$) pada sinyal **`Echo` HC-SR04** agar tegangan $5\text{ V}$ turun menjadi $3.3\text{ V}$ sebelum masuk ke `GPIO 18`.
