# Development Tools & Telemetry Utilities

Perangkat bantu pengujian offline dan analisis telemetri navigasi robot (Praktikum Robotika TC253421, D3 Teknik Komputer Polsri).

---

## 1. Simulator Navigasi Kinematika (`simulate_navigation.py`)

Simulator offline berbasis Python untuk memvalidasi algoritma Go-To-Goal, Finite State Machine (FSM 8-state), dan penghindaran rintangan dinamis tanpa memerlukan robot fisik.

### Fitur:
- Pemodelan kinematika *differential drive*.
- Geometri arena koridor 480 × 80 cm dan 4 ruangan tujuan (Room 1–4).
- Pengecekan *clearance* dinding koridor ($\ge 10\text{ cm}$) dan batas rintangan ($\ge 10\text{ cm}$).
- Verifikasi presisi lingkaran target ($R = 5\text{ cm}$, diameter $10\text{ cm}$).
- Ekspor telemetri format CSV standar: `tools/sample_telemetry.csv`.

### Cara Menjalankan:

```bash
# 1. Jalankan seluruh suite simulasi standar (Babak 1, 2, 3, dan 3-Alt):
python3 tools/simulate_navigation.py

# 2. Jalankan simulasi untuk ruangan tertentu (misal: Room 3) dan langsung buat plot lintasan SVG:
python3 tools/simulate_navigation.py --room 3 --plot

# 3. Jalankan simulasi dengan rintangan kustom (format 'x,y,w,h'):
python3 tools/simulate_navigation.py --room 2 --obs "110,0,20,20" --plot

# 4. Lihat seluruh opsi perintah:
python3 tools/simulate_navigation.py --help
```

---

## 2. Plotter Lintasan Telemetri (`plot_trajectory.py`)

Utilitas pengolah file CSV telemetri serial untuk menghasilkan diagram lintasan vektor (SVG) beresolusi tinggi untuk laporan akhir praktikum.

### Format Telemetri CSV:
File CSV input memiliki kolom header:
```csv
t_ms,state,x,y,theta,jarak,iWp
```

### Cara Menjalankan:
```bash
# Render file SVG dari telemetri
python3 tools/plot_trajectory.py tools/sample_telemetry.csv -o tools/trajectory_plot.svg
```

Diagram yang dihasilkan memvisualisasikan:
- Batas koridor 480 × 80 cm dan garis tengah (*centerline*).
- Pintu masuk dan lingkaran target Room 1, 2, 3, dan 4.
- Kotak rintangan $20 \times 20\text{ cm}$.
- Garis lintasan aktual robot berwarna biru.
- Titik START $(0, 0)$ hijau dan titik TIBA merah beserta koordinat akhir.

---

## 3. Wizard Kalibrasi & Diagnostik Pre-Flight (`auto_calibrate.py`)

Utilitas interaktif berbasis terminal (CLI) untuk mengeksekusi pengujian perangkat keras Pre-Flight (Stage U0), penguncian keamanan (*hard-gated safety*), serta kalibrasi parameter kinematika robot secara dinamis tanpa kompilasi ulang kode sumber.

### Fitur Stage U0:
- **U0.1 MPU6050 I2C Bus**: Memeriksa register `WHO_AM_I` pada address 0x68 (Pin SDA A4, SCL A5).
- **U0.2 Sonar Ultrasonik HC-SR04**: Validasi pantulan echo bebas timeout (Trig A0, Echo A1).
- **U0.3 Sensor IR Lateral FC-51**: Pengujian logika active-low sensor dinding kiri (A3) dan kanan (D11).
- **U0.4 Indikator Buzzer & LED**: Pengujian audio-visual buzzer piezo (A2) dan status LED (D13).
- **U0.5 Motor Driver & Dual Encoder**: Memutar roda independen, memverifikasi pembacaan pulsa optical encoder (D2/D3), dan otomatis mengaplikasikan *software polarity inversion* ke EEPROM jika arah putaran terbalik.
- **Hard-Gated Safety**: Memblokir kalibrasi gerak jika salah satu komponen hardware gagal dan menampilkan petunjuk perbaikan pin perkabelan (*wiring repair*).

### Cara Menjalankan:

```bash
# 1. Jalankan pengujian interaktif terhadap robot fisik (autodetect port serial):
python3 tools/auto_calibrate.py

# 2. Tentukan port serial secara spesifik:
python3 tools/auto_calibrate.py --port /dev/ttyUSB0 --baud 115200

# 3. Jalankan pengujian offline/CI dengan mock robot firmware:
python3 tools/auto_calibrate.py --mock --headless

# 4. Uji simulasi kegagalan sensor tertentu pada mock mode:
python3 tools/auto_calibrate.py --mock --fail-mpu
python3 tools/auto_calibrate.py --mock --fail-sonar
python3 tools/auto_calibrate.py --mock --reverse-left
```
