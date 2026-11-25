# Desain: Model 3D Rangkaian Robot + Perkabelan di Blender

Tanggal: 2026-11-25 · Status: draft (menunggu persetujuan)

## 1. Maksud & Alasan

Pemilik proyek ingin **melihat rangkaian fisik robot secara visual**: semua komponen
dimodelkan sesuai bentuk dan ukuran aslinya, dirakit di sasis, dan **setiap kabel
terhubung ke pin yang benar** sesuai dokumentasi perkabelan repo. Model ini menjadi
alat bantu untuk memahami dan merakit rangkaian, serta untuk materi laporan praktikum.

## 2. Tujuan, Non-Tujuan, Anti-Tujuan

**Tujuan**
- Ada file `.blend` berisi robot lengkap: sasis, komponen, dan kabel.
- Bentuk dan ukuran setiap komponen sesuai aslinya. Ukuran utama meleset paling jauh
  ±1 mm dari datasheet, dan urutan serta label pin sesuai papan aslinya.
- Ada 41 kabel, persis sesuai `docs/hardware/wiring-schematic.md` (varian **Arduino Uno + L298N**).
- Ada render PNG (tampak isometrik, atas, dan bawah) untuk dokumentasi.
- Semuanya bisa dibangun ulang dengan satu perintah, dan ketepatan setiap kabel dicek oleh tes otomatis.

**Non-Tujuan**
- Varian L293D, ESP32, dan modul HC-05 (opsional, tidak ada di skematik L298N).
- Animasi, fisika, atau simulasi gerak (sudah ada `replay` CSV terpisah sebagai ide lain).
- Detail sampai tingkat jalur tembaga PCB atau komponen SMD kecil pada model buatan skrip.

**Anti-Tujuan** (gagal meskipun "jalan")
- Ada kabel yang tersambung ke pin yang salah, atau tidak cocok dengan `Config.h` dan dokumen perkabelan.
- Ada kabel yang terlihat menembus badan papan atau komponen.
- Model unduhan tanpa lisensi redistribusi ikut ter-commit ke git.

## 3. Batasan

- Blender 5.3.0 (`/Applications/Blender.app`, Python 3.13 bawaan) bisa membuka STL, OBJ, dan glTF,
  tetapi **tidak bisa membuka STEP**. FreeCAD tidak terpasang.
- Sumber model harus bisa diunduh **tanpa login**. Printables dapat diakses secara anonim lewat GraphQL
  `getDownloadLink` (sudah diverifikasi), begitu juga raw GitHub.
- Aturan Python repo (`CODING_STANDARDS.md` §6): ada type hint, memakai `@dataclass` untuk data
  multi-atribut, dan lolos `tools/check_python_standards.py` serta `make lint`.
- `make check` tidak boleh bergantung pada jaringan maupun Blender.

## 4. Pendekatan

| | Pendekatan | Kelebihan / kekurangan |
|---|---|---|
| A | Semua komponen dibangun prosedural lewat skrip | Bisa direproduksi dan dites. Tampilan kurang realistis. |
| **B (dipilih)** | **Model unduhan (STEP/STL) + perkabelan oleh skrip** | Tampilan paling realistis. Perlu konversi STEP, ada urusan lisensi, dan koordinat pin harus diukur per model. |
| C | Hibrida A→B | Fleksibel, tetapi dimulai dengan kualitas visual A. |

Pemilik memilih **B**, dengan modifikasi: komponen yang **tidak tersedia tanpa login**
dibuat lewat skrip (prosedural) mengikuti ukuran asli.

## 5. Keputusan yang Sudah Diambil Pemilik

1. Driver motor: **modul L298N** (bukan IC L293D).
2. Sasis: **akrilik 2WD satu lantai, ±210×150 mm**. Motor TT dan holder baterai di bawah. Uno,
   L298N, dan breadboard mini (Bus +5V dan Star Ground Hub) di atas. HC-SR04 di depan,
   FC-51 di sisi kiri dan kanan, caster di belakang.
3. Model dicari sendiri dari sumber tanpa login. Yang tidak ketemu dibuat lewat skrip.

## 6. Daftar Komponen & Sumber Model

| Komponen | Jml | Sumber | Format | Lisensi → commit? |
|---|---|---|---|---|
| Arduino Uno R3 | 1 | GitHub `Sajitha-Aldeniya/KiCad-Simple-Libraries` `3dmodels/SL_Development_Boards.3dshapes/ArduinoUNO_3D.step` | STEP | tidak ada lisensi → **tidak di-commit** |
| Modul L298N | 1 | Printables 796750 `L298N.obj` | OBJ | CC-BY → boleh di-commit + atribusi |
| HC-SR04 | 1 | KiCad-Simple `SL_Breakout_Boards.3dshapes/HC-SR04.step` | STEP | tidak di-commit |
| MPU6050 / GY-521 | 1 | KiCad-Simple `SL_Breakout_Boards.3dshapes/MPU6050.step` | STEP | tidak di-commit |
| Motor TT kuning 1:48 | 2 | Printables 1336089 `TT Motor.step` | STEP | CC-BY |
| Roda TT Ø65 | 2 | Printables 306463 | STL | CC0 |
| Cakram encoder 20 slot | 2 | Printables 129515 (panBot) | STL | CC-BY-NC-SA (penggunaan non-komersial: oke) |
| Holder 18650 2-cell | 1 | Printables 407681 `18650x2_w_tab.stl` | STL | CC0 |
| Push button START | 1 | KiCad-Simple `SL_Devices.3dshapes/Push_Button_W12mm.step` | STEP | tidak di-commit |
| Baterai 18650 | 2 | prosedural (silinder Ø18×65 mm, pembungkus berwarna) | — | — |
| **FC-51** | 2 | **prosedural** | — | — |
| **Encoder LM393 (FC-03)** | 2 | **prosedural** | — | — |
| **Sasis akrilik 2WD** | 1 | **prosedural** (transparan, lubang baut, slot kabel) | — | — |
| **Saklar rocker ON/OFF** | 1 | **prosedural** | — | — |
| **Buzzer aktif 5V (Ø12×9.5 mm)** | 1 | **prosedural** | — | — |
| **Breadboard half-size 400 titik** (82.5×54.5 mm) | 1 | **prosedural** (rel +5V dan rel GND sebagai hub) | — | — |
| **Caster** | 1 | **prosedural** | — | — |
| Kapasitor keramik 100 nF di terminal motor | 2 | prosedural (disolder, tanpa kabel jumper) | — | — |

Jika sebuah URL ternyata mati atau kualitas modelnya buruk saat dicek, pembangun boleh
**mengganti ke sumber tanpa login lain atau beralih ke prosedural**, lalu mencatatnya di `SOURCES.md`.

Ukuran acuan model prosedural (pembangun memverifikasi dari datasheet/foto produk):
FC-51 ±32×14 mm, header 3 pin (OUT, GND, VCC), LED IR + fotodioda di ujung, trimpot biru.
LM393 FC-03 ±32×14 mm, slot optik 5 mm, header 4 pin (VCC, GND, D0, A0).
Header berjarak 2.54 mm. Lubang baut M3.

## 7. Netlist (Sumber Kebenaran Perkabelan)

Netlist ditulis sebagai data Python (`tools/blender/netlist.py`, memakai `@dataclass`). Isinya
41 kabel, dikelompokkan menurut tahap di `wiring-schematic.md`:

- **Daya (7):** Batt− → GND hub. Batt+ → Switch IN. Switch OUT → Uno VIN. Switch OUT → L298N +12V.
  Uno GND → GND hub. L298N GND → GND hub. Uno 5V → rel +5V. Pin +5V L298N **dibiarkan kosong**.
  Jumper 5V-EN terpasang, jumper ENA/ENB **dilepas**, dan keduanya terlihat di model.
- **Sinyal motor (6):** D9→ENA, D8→IN1, D7→IN2, D6→ENB, D5→IN3, D4→IN4.
- **Motor (4):** Motor kiri → OUT1/OUT2, motor kanan → OUT3/OUT4.
- **Encoder (6):** Kiri: VCC→+5V, GND→hub, D0→D2. Kanan: VCC→+5V, GND→hub, D0→D3.
- **HC-SR04 (4):** VCC→+5V, Trig→A0, Echo→A1, GND→hub.
- **MPU6050 (4):** VCC→+5V, GND→hub, SDA→A4, SCL→A5. AD0 tidak disambung.
- **FC-51 (6):** Kiri OUT→A3, kanan OUT→D11, serta VCC dan GND masing-masing ke rel.
- **START (2):** D12 dan hub. **Buzzer (2):** + → A2, − → hub.

Warna kabel mengikuti `docs/hardware/wiring_schematic_uno_l298n.svg` per net (merah 7.4V,
oranye 5V, hitam GND, biru sinyal motor, hijau/ungu/biru muda sinyal sensor). Pembangun
membaca warna persisnya dari SVG tersebut.

**Tes** `tests/test_blender_netlist.py` (Python murni, tanpa Blender, masuk ke `make test`):
- Setiap pin Arduino di netlist harus cocok dengan `#define PIN_*` di cabang Uno pada `Config.h`.
- Setiap baris tabel sambungan di `wiring-schematic.md` harus ada di netlist, dan sebaliknya.
- Setiap rel +5V dan GND hanya boleh terhubung ke pin bertipe VCC atau GND.
- Tidak ada pin yang terpakai dua kali, kecuali rel hub.

## 8. Pipeline Pembangunan

```
assets/3d/sources.json ──fetch_models.py──► assets/3d/vendor/*.step|stl|obj   (vendor/ di-gitignore)
                                   │
                      convert_step.py (cadquery-ocp di .venv/) ──► assets/3d/cache/*.glb  (di-gitignore)
                                   │
 netlist.py + anchors.json + procedural_parts.py
                                   │
          blender -b -P tools/blender/build_robot.py ──► build/robot_3d.blend
                                                     └─► docs/hardware/renders/robot_{iso,top,bottom}.png
```

- **`fetch_models.py`**: mengunduh file sesuai `sources.json` (URL, SHA-256, lisensi, boleh-commit),
  lalu memverifikasi hash-nya.
- **`convert_step.py`**: mengonversi STEP ke glTF dengan mempertahankan warna dari STEP (XCAF), memakai
  paket pip `cadquery-ocp` di `.venv/` repo (sudah di-gitignore). Keputusan saya: cara ini menghindari
  pemasangan aplikasi GUI. FreeCAD CLI menjadi cadangan.
- **`anchors.json`**: untuk setiap model unduhan, berisi transformasi normalisasi (skala ke mm, rotasi
  agar PCB rata dengan pin menghadap +Z) dan **koordinat lokal setiap pin yang dipakai**. Koordinat
  diukur sekali dari mesh dan diverifikasi dengan render close-up. Model prosedural menghasilkan
  anchor-nya sendiri dari kode.
- **`build_robot.py`**: membangun scene di Blender. Unit scene dalam meter (1 BU = 1 m), objek diberi
  nama `<komponen>`, dan kabel diberi nama `wire_<net>_<dari>_<ke>`.
- **Perkabelan**: kabel jumper dupont dibuat sebagai kurva Bezier dengan bevel Ø1.6 mm, ditambah
  housing dupont 2.54 mm di ujung yang menancap ke header. Kabel ke terminal sekrup (L298N, baterai,
  switch) memakai ujung kabel telanjang. Rutenya: naik tegak lurus dari pin, lalu bergerak di "lapisan
  rute" di atas komponen tertinggi, lalu turun ke pin tujuan. Kabel dari komponen di bawah sasis lewat
  slot sasis. Penyeberangan antarkabel diperbolehkan, tetapi kabel menembus badan komponen tidak.
- **Make**: `make blender_fetch` (perlu jaringan) dan `make blender_model` (perlu Blender). Keduanya
  **tidak** masuk ke `make check`. Tes netlist masuk ke `make test`.
- **Lisensi**: `assets/3d/SOURCES.md` memuat atribusi CC-BY. File yang tidak boleh di-commit
  dimasukkan ke `.gitignore`.

## 9. Tata Letak di Sasis (koordinat robot: +X depan, +Y kiri, +Z atas, asal di tengah poros roda)

- Jarak antar-pusat roda **135 mm** dan roda **Ø65 mm** (`Config.h`: `JARAK_RODA_MM`, `DIAMETER_RODA_MM`).
- **Bawah sasis:** motor TT di kiri dan kanan pada poros Y. Cakram encoder di poros dalam motor, dengan
  LM393 mengapit cakram. Holder 18650 di belakang poros. Caster di belakang.
- **Atas sasis:** L298N di depan-tengah, MPU6050 **datar di dekat pusat poros**, Uno di
  tengah-belakang, breadboard mini di samping Uno, serta saklar, START, dan buzzer di tepi belakang.
- **Depan:** HC-SR04 berdiri tegak menghadap +X. **Samping:** FC-51 menghadap ±Y.

Posisi persisnya diserahkan kepada pembangun, selama mengikuti zona di atas dan tidak ada komponen
yang saling bertabrakan.

## 10. Kriteria Selesai

1. `make blender_fetch && make blender_model` menghasilkan `build/robot_3d.blend` dan 3 PNG tanpa error.
2. `make check` lulus, termasuk `tests/test_blender_netlist.py` yang baru.
3. Ada 41 objek `wire_*` di scene, dan ujung setiap kabel berada ≤0.5 mm dari anchor pinnya
   (diperiksa oleh `build_robot.py` saat dijalankan).
4. Render close-up setiap header (Uno, L298N, dan setiap sensor) memperlihatkan housing dupont pada pin
   yang benar. Pemilik meninjau render tersebut.

## 11. Keputusan Penulis (bisa dikoreksi pemilik)

- Hub daya memakai **satu pasang rel atas breadboard half-size 400 titik**: rel merah = Bus +5V, rel biru = Star Ground Hub. Breadboard mini 170 titik **tidak punya rel daya**, sedangkan GND butuh 11 sambungan.
- `.blend` dihasilkan ke `build/` (sudah di-gitignore). Hanya render PNG yang di-commit.
- `cadquery-ocp` (±150 MB) dipasang di `.venv/` lokal repo, bukan secara global.
- Untuk setiap model unduhan, versi aslinya dipakai apa adanya. Jika ternyata berbeda dari versi
  yang Anda pakai (misalnya Uno SMD vs DIP), pembangun memilih versi DIP standar.

## 12. Diserahkan kepada Pembangun

- Bentuk detail model prosedural, material, pencahayaan, dan sudut kamera render.
- Algoritme rute kabel, selama memenuhi aturan di §8.
- Penggantian sumber model jika ada yang rusak (dicatat di `SOURCES.md`).
