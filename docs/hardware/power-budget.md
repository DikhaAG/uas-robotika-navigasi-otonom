# Analisis Catu Daya & Anggaran Arus (Power Budget Analysis)

Dokumentasi kelistrikan, kalkulasi arus beban, dan mitigasi derau kelistrikan untuk Robot Navigasi Otonom (Praktikum Robotika TC253421, D3 Teknik Komputer Polsri).

---

## 1. Arsitektur Catu Daya Terisolasi (Dual Power Rail)

Untuk mencegah reset mikrokontroler (*brownout*) dan derau switching motor merusak komunikasi I2C sensor MPU6050, robot menerapkan pemisahan jalur daya:

```
[ Baterai 2S Li-Ion 7.4V (Nominal 7.4V, Penuh 8.4V) ]
         │
         ├─── Switch ON/OFF Utama
         │         │
         │         ├───► Masuk ke L293D Pin 8 (VCC2 / Motor Power)
         │         │     (Dilengkapi Kapasitor Elektrolit 220 µF)
         │         │
         │         └───► Masuk ke Vin Arduino Uno / Regulator 5V (LM7805/Buck)
         │                     │
         │                     └───► Bus Logika 5.0V (VCC1 L293D, MPU6050, Sonar, Encoder)
         │                           (Dilengkapi Kapasitor Keramik 100 nF)
         │
         └─────────────► Common Star Ground (GND Satu Titik Bintang)
```

> **Aturan Wajib Star Ground:** Jalur Ground motor dan Ground logika Arduino Uno **wajib disatukan pada satu titik bersama** (*single-point star ground*) dekat terminal negatif baterai. Jangan melakukan pengkabelan ground secara daisy-chain antar motor dan modul sensor!

---

## 2. Kalkulasi Beban Arus Listrik (Current Consumption)

### A. Komponen Elektronika & Logika (Rel 5V)

| Modul / Komponen | Arus Nominal (mA) | Arus Maksimum (mA) | Keterangan |
| :--- | :---: | :---: | :--- |
| **Arduino Uno R3 (ATmega328P)** | 45 | 55 | Konsumsi inti, regulator onboard, & indikator |
| **IMU MPU6050** | 3.5 | 4.0 | Modul gyro & akselerometer (I2C) |
| **Sensor Ultrasonik HC-SR04** | 15 | 20 | Saat transmisi gelombang ultrasonik 40 kHz |
| **2x Optocoupler LM393 (FC-03)** | 20 | 30 | 10–15 mA per modul encoder roda |
| **2x Sensor IR Dinding (FC-51)** | 30 | 40 | IR LED emiter & komparator op-amp |
| **Buzzer Aktif 5V** | 25 | 30 | Aktif saat tiba atau failsafe |
| **Modul Bluetooth HC-05** | 30 | 40 | Saat transmisi paket data serial |
| **L293D VCC1 (Logic)** | 35 | 60 | Sirkuit gerbang logika internal L293D |
| **TOTAL BEBAN LOGIKA (5V)** | **~203.5 mA** | **~279 mA** | Dipasok regulator 5V Arduino |

### B. Beban Aktuator Motor DC (Rel 7.4V)

Robot menggunakan 2 unit motor DC TT Kuning (gear ratio 1:48):

| Kondisi Kerja Motor | Arus per Motor | Arus Total (2 Motor) | Catatan L293D (Batas 600 mA/ch) |
| :--- | :---: | :---: | :--- |
| **Tanpa Beban (Free Run)** | ~120 mA | ~240 mA | Kecepatan penuh di udara |
| **Beban Normal (Lantai Arena)** | ~250 mA | ~500 mA | **Aman** (di bawah rating 600 mA L293D) |
| **Beban Puncak / Belok In-Place** | ~400 mA | ~800 mA | Masih dalam batas aman dinamik |
| **Kondisi Terjebak / Stall** | ~900 mA – 1.0 A | ~1.8 – 2.0 A | **BAHAYA!** Melebihi limit L293D (600 mA cont) |

> **Perlindungan Software Failsafe:** Karena arus motor saat roda terkunci (*stall*) mencapai ~1 A per motor yang dapat merusak driver L293D, firmware menerapkan **Watchdog Stall Detection (Failsafe 2)**: jika PWM aktif namun pulsa encoder bernilai nol selama $\ge 1.0$ detik, motor otomatis dimatikan seketika (`PWM = 0`).

---

## 3. Karakteristik Penurunan Tegangan L293D (*Voltage Drop*)

Driver motor L293D berbasis transistor bipolar BJT darlington internal yang memiliki resistansi internal dan tegangan saturasi:
$$V_{\text{drop}} = V_{CE(\text{sat})} \approx 1.5\text{ V} – 2.0\text{ V}$$

Jika menggunakan baterai 2S Li-ion $7.4\text{ V}$ nominal:
$$V_{\text{motor\_efektif}} = V_{\text{baterai}} - V_{\text{drop}} = 7.4\text{ V} - 1.8\text{ V} = 5.6\text{ V}$$

Tegangan efektif $5.6\text{ V}$ ini sangat ideal untuk motor DC TT (yang dirancang untuk rentang $3\text{ V} – 6\text{ V}$), menghasilkan torsi yang memadai tanpa membuat motor mengalami *overheat*.

---

## 4. Estimasi Daya Tahan Baterai (*Battery Lifetime*)

Menggunakan baterai 2S 18650 dengan kapasitas rata-rata $2200\text{ mAh}$:
- Rata-rata konsumsi arus saat navigasi bergerak: $I_{\text{total}} \approx 200\text{ mA (logika)} + 500\text{ mA (motor)} = 700\text{ mA}$.
- Estimasi durasi operasional:
  $$\text{Waktu Tempuh} = \frac{2200\text{ mAh}}{700\text{ mA}} \times 0.8 \approx 2.5\text{ Jam}$$
- Setiap uji navigasi di arena koridor 480 cm berlangsung maksimal 90 detik, sehingga satu set baterai 2S terisi penuh mampu melayani $\ge 50$ kali uji coba navigasi berturut-turut.

---

## 5. Kapasitor Decoupling & Penanganan Derau (*Noise Filtering*)

1. **Kapasitor Keramik 100 nF (0.1 µF):**
   - Dipasang sedekat mungkin antara pin 16 L293D (VCC1) dan GND.
   - Menyerap transien switching frekuensi tinggi agar tidak merambat ke rail 5V mikrokontroler.
2. **Kapasitor Elektrolit 100 µF – 220 µF (16V atau 25V):**
   - Dipasang paralel pada pin 8 L293D (VCC2) dan GND.
   - Berfungsi sebagai reservoir energi lokal saat motor mulai berakselerasi (*inrush current*), mencegah *voltage sag* pada baterai.
3. **Kapasitor Keramik 100 nF pada Terminal Motor:**
   - Disolder langsung melintang di antara kedua kutub motor DC TT untuk meredam percikan komutator sikat arang (*brush spark EMI*).
