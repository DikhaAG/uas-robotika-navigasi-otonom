# Lembar Kerja & Protokol Uji Kalibrasi (U1 – U4)

Protokol kalibrasi mandiri untuk memverifikasi sensor, aktuator, dan estimasi dead-reckoning sebelum robot diuji di arena navigasi utama (Praktikum Robotika TC253421, D3 Teknik Komputer Polsri).

---

## Ringkasan Matriks Pengujian

| Kode | Nama Pengujian | Tujuan Kalibrasi | Standar Ground Truth | Batas Toleransi Galat |
| :---: | :--- | :--- | :--- | :---: |
| **U1** | Uji Pulsa & Diameter Encoder | Mengkalibrasi `DIAMETER_RODA_MM` | Meteran pita (100 cm) | $\le 3.0\text{ cm}$ |
| **U2** | Uji Polaritas & Akurasi Gyro | Memverifikasi tanda gyro (`gyroSign`) & sudut | Busur derajat lantai ($\pm 90^\circ$) | $\le 5.0^\circ$ |
| **U3** | Uji Lintasan Lurus Berpemandu | Mengukur deviasi lateral lurus | Garis acuan lantai (150 cm) | $\le 5.0\text{ cm}$ |
| **U4** | Uji Akurasi Sensor Ultrasonik | Mengkalibrasi HC-SR04 sonar | Mistar ukur (10, 20, 40 cm) | $\le 1.0\text{ cm}$ |

---

## Protokol U1: Kalibrasi Pulsa Encoder & Diameter Roda

### 1. Tujuan
Memastikan konstanta keliling roda dan jumlah pulsa menghasilkan estimasi jarak translasi yang akurat pada permukaan lantai arena.

### 2. Peralatan
- Robot dengan firmware `Uji_Encoder.ino`.
- Meteran pita atau penggaris 100 cm.
- Lakban penanda garis start dan garis 100 cm.

### 3. Langkah Kerja
1. Letakkan titik tengah roda robot tepat di atas garis start ($0\text{ cm}$).
2. Buka Serial Monitor Arduino IDE pada baud rate 115200.
3. Ketik perintah `'0'` lalu kirim untuk mereset counter pulsa.
4. Ketik perintah `'f'` lalu kirim. Robot akan maju otomatis hingga counter pulsa mencapai target translasi 100 cm ($\approx 98$ pulsa).
5. Setelah robot berhenti, ukur posisi fisik aktual ujung roda menggunakan meteran pita.
6. Catat jarak aktual $D_{\text{aktual}}$ dan pulsa yang tercatat.

### 4. Formula Penyetelan (*Tuning*)

#### A. Koreksi Diameter Roda Efektif terhadap Permukaan
Permukaan lantai yang berbeda (keramik licin, karpet bertekstur, atau tripleks) menyebabkan deformasi ban karet dan tingkat selip yang berbeda. Lakukan koreksi diameter roda di `Config.h`:
$$D_{\text{baru}} = D_{\text{lama}} \times \left( \frac{D_{\text{aktual}}}{100.0} \right)$$

#### B. Kalibrasi Ketidakseimbangan Motor Kiri vs Kanan (*Motor Trimming*)
Jika salah satu motor berputar lebih cepat dari motor lainnya pada PWM yang sama (misal teramati pulsa $P_L \ne P_R$ saat uji maju lurus tanpa beban luar), hitung rasio pulsa kedua roda:
$$R_{\text{motor}} = \frac{P_L}{P_R}$$
- Jika $P_L > P_R$ (motor kiri lebih cepat, robot membelok ke kanan):
  - Turunkan `TRIM_MOTOR_KIRI` di `Config.h`: $\text{TRIM\_MOTOR\_KIRI} = \frac{P_R}{P_L}$, ATAU
  - Naikkan `TRIM_MOTOR_KANAN` di `Config.h`: $\text{TRIM\_MOTOR\_KANAN} = \frac{P_L}{P_R}$
- Jika $P_R > P_L$ (motor kanan lebih cepat, robot membelok ke kiri):
  - Turunkan `TRIM_MOTOR_KANAN` di `Config.h`: $\text{TRIM\_MOTOR\_KANAN} = \frac{P_L}{P_R}$
Nilai trim standar berkisar antara $0.85$ hingga $1.15$.

### 5. Tabel Pengamatan U1
| Percobaan | Pulsa Kiri ($P_L$) | Pulsa Kanan ($P_R$) | Jarak Firmware (cm) | Jarak Fisik Meteran (cm) | Galat $\Delta D$ (cm) | Status ($\le 3\text{ cm}$) |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| 1 | 98 | 98 | 100.0 | 99.2 | 0.8 | LULUS |
| 2 | 98 | 97 | 99.5 | 98.7 | 0.8 | LULUS |
| 3 | 99 | 98 | 100.5 | 100.1 | 0.4 | LULUS |

---

## Protokol U2: Kalibrasi Gyroscope MPU6050 & Rotasi In-Place

### 1. Tujuan
Memverifikasi polaritas putaran gyro (`gyroSign` $= +1$ atau $-1$; pada firmware utama dipilih lewat orientasi MPU6050 di wizard U0.1 / menu [8], pada sketch `Uji_MPU6050.ino` lewat konstanta `GYRO_SIGN`), menghitung bias statis, dan memvalidasi keakuratan integrasi sudut belok $90^\circ$ dan $360^\circ$.

### 2. Peralatan
- Robot dengan firmware `Uji_MPU6050.ino`.
- Lembar busur derajat lantai atau tanda siku lantai keramik.

### 3. Langkah Kerja
1. Letakkan robot di atas lantai datar dengan arah hadap tepat sejajar garis $0^\circ$.
2. Nyalakan robot. Firmware otomatis melakukan pengambilan 200 sampel saat robot diam selama 2 detik untuk menghitung `gyroBiasZ`.
3. Pastikan nilai `Rate Z` saat diam berada di sekitar $0.00 \pm 0.15^\circ/\text{s}$.
4. Kirim karakter `'l'`. Robot akan berputar di tempat (*in-place*) ke arah kiri hingga pembacaan firmware mencapai $+90^\circ$.
5. Amati arah fisik robot: jika robot berputar ke kiri, maka `GYRO_SIGN = 1.0f` (chip menghadap atas). Jika berputar ke kanan atau sudut membaca terbalik, ubah `GYRO_SIGN = -1.0f` di sketch uji ini (chip menghadap bawah), lalu pilih orientasi yang sama di wizard untuk firmware utama.
6. Ukur sudut fisik aktual pada busur lantai. Hitung galat sudut $E = |90^\circ - \theta_{\text{aktual}}|$.

### 4. Tabel Pengamatan U2
| Skenario Uji | Perintah | Sudut Firmware | Sudut Aktual Lantai | Galat Sudut | Status ($\le 5^\circ$) |
| :--- | :---: | :---: | :---: | :---: | :---: |
| Putar Kiri | `'l'` | $+90.0^\circ$ | $+91.5^\circ$ | $1.5^\circ$ | LULUS |
| Putar Kanan | `'r'` | $-90.0^\circ$ | $-88.5^\circ$ | $1.5^\circ$ | LULUS |
| Putar Penuh $360^\circ$ | Manual | $360.0^\circ$ | $357.0^\circ$ | $3.0^\circ$ | LULUS |

---

## Protokol U3: Uji Lintasan Lurus Berpemandu Gyro (Straight Line Tracking)

### 1. Tujuan
Memverifikasi bahwa kontrol kemudi proporsional berbasis heading gyro mampu mengoreksi ketidakseimbangan mekanik motor TT dan menjaga robot tetap melaju lurus di sepanjang koridor 150 cm tanpa menabrak dinding.

### 2. Peralatan
- Robot dengan firmware navigasi `RobotNavigasiOtonom.ino`.
- Lintasan koridor lurus sepanjang 150 cm dengan garis tengah bertanda sumbu Y.

### 3. Langkah Kerja
1. Posisikan robot pada koordinat $(0, 0)$ menghadap sumbu $+X$.
2. Jalankan perintah gerak lurus 150 cm.
3. Saat robot berhenti pada $X = 150\text{ cm}$, ukur pergeseran lateral pada sumbu Y (jarak tegak lurus dari garis tengah lintasan).
4. Pastikan penyimpangan lateral $|y_{\text{deviasi}}| \le 5.0\text{ cm}$.

### 4. Tabel Pengamatan U3
| Percobaan | Jarak Translasi X (cm) | Posisi Y Akhir (cm) | Deviasi Sumbu Y (cm) | Status ($\le 5\text{ cm}$) |
| :---: | :---: | :---: | :---: | :---: |
| 1 | 150.0 | +1.8 | 1.8 | LULUS |
| 2 | 150.0 | -2.3 | 2.3 | LULUS |
| 3 | 150.0 | +1.2 | 1.2 | LULUS |

---

## Protokol U4: Kalibrasi Akurasi Sensor Ultrasonik HC-SR04

### 1. Tujuan
Memverifikasi bahwa sensor jarak ultrasonik HC-SR04 mendeteksi rintangan kotak kardus pada jarak 10 cm, 20 cm, dan 40 cm secara akurat dan tidak terpengaruh oleh pantulan lantai.

### 2. Peralatan
- Kotak rintangan $20 \times 20\text{ cm}$ (kotak kardus arena).
- Penggaris meteran kayu.

### 3. Langkah Kerja
1. Letakkan kotak rintangan di depan robot tegak lurus sensor pada jarak 10 cm, 20 cm, dan 40 cm.
2. Amati data jarak yang dikirimkan oleh firmware pada telemetri serial.
3. Catat selisih antara nilai firmware dan jarak fisik mistar.

### 4. Tabel Pengamatan U4
| Jarak Fisik Mistar (cm) | Pembacaan Sonar (cm) | Galat Mutlak (cm) | Status ($\le 1.0\text{ cm}$) |
| :---: | :---: | :---: | :---: |
| 10.0 | 10.2 | 0.2 | LULUS |
| 20.0 | 20.1 | 0.1 | LULUS |
| 40.0 | 39.6 | 0.4 | LULUS |

---

## 5. Eksekusi Otomatis Berpemandu CLI (`tools/auto_calibrate.py`)

Selain pengujian manual dengan firmware mandiri, seluruh rangkaian protokol kalibrasi (U0 hingga U4) dapat dijalankan secara otomatis menggunakan utilitas Python Host Wizard.

### Keunggulan Eksekusi Otomatis:
1. **Hard-Gated Safety**: Menjamin pengujian gerak U1–U4 hanya dapat berjalan setelah tahap diagnostik hardware U0 (MPU6050, HC-SR04, FC-51 IR, Buzzer, LED, dan Motor/Encoder) 100% lulus.
2. **Auto-Retry & Convergence Loop**: Jika galat translasi U1 $> 3.0\text{ cm}$ atau disparitas pulsa $> 3\%$, tool otomatis menghitung $D_{\text{baru}}$ dan rasio motor trim, memperbarui EEPROM via serial, lalu mengulang pengujian hingga toleransi terpenuhi (maksimal 5 iterasi).
3. **Ekspor Log Telemetri Otomatis**: Hasil pengujian dicatat secara terstruktur ke `tools/calibration_log.csv` dengan kolom lengkap (waktu, ID pengujian, iterasi, target, nilai terukur, ground truth, galat, dan profil EEPROM) untuk dimasukkan ke laporan akhir praktikum.

### Perintah Eksekusi:
```bash
# Jalankan seluruh suite kalibrasi secara otomatis:
python3 tools/auto_calibrate.py --port /dev/ttyUSB0

# Jalankan dalam mode menu individual:
python3 tools/auto_calibrate.py --port /dev/ttyUSB0 --mode individual
```
