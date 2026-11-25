# Sumber Model 3D

Model unduhan dipakai oleh `make blender_model` untuk membangun `build/robot_3d.blend`.
File-file ini **tidak diredistribusi**: `make blender_fetch` mengunduhnya ke `assets/3d/vendor/`
(di-gitignore) dan memverifikasi SHA-256 yang dipin di [`sources.json`](sources.json).
Hasil konversi STEP → GLB disimpan di `assets/3d/cache/` (juga di-gitignore).

| Key | File | Sumber | Lisensi | Penulis / atribusi |
|---|---|---|---|---|
| `uno` | `ArduinoUNO_3D.step` | [KiCad-Simple-Libraries](https://github.com/Sajitha-Aldeniya/KiCad-Simple-Libraries) @ `bcb4717` | tidak ada lisensi (pemakaian pribadi, tidak diredistribusi) | Sajitha Aldeniya |
| `sonar` | `HC-SR04.step` | KiCad-Simple-Libraries @ `bcb4717` | tidak ada lisensi (idem) | Sajitha Aldeniya |
| `mpu` | `MPU6050.step` | KiCad-Simple-Libraries @ `bcb4717` | tidak ada lisensi (idem) | Sajitha Aldeniya |
| `btn` | `Push_Button_W12mm.step` | KiCad-Simple-Libraries @ `bcb4717` | tidak ada lisensi (idem) | Sajitha Aldeniya |
| `l298n` | `L298N.obj` | [Printables 796750 — L298N Motor Drive](https://www.printables.com/model/796750) | CC-BY 4.0 | Davide Bucci |
| `motor_tt` | `TT_Motor.step` | [Printables 1336089 — TT Motor (reference)](https://www.printables.com/model/1336089) | CC-BY 4.0 | Tom3r |
| `enc_disk` | `Encoder_TT_20_slot_22mm.stl` | [Printables 129515 — panBot Encoder Disk TT-Motor](https://www.printables.com/model/129515) | CC-BY-NC-SA 4.0 (non-komersial) | ZDKderPHB |
| `batt_holder` | `18650x2_w_tab.stl` | [Printables 407681 — 18650 Battery Holder 2 cell](https://www.printables.com/model/407681) | CC0 1.0 | syumijin |

## Atribusi (CC-BY / CC-BY-NC-SA)

- *L298N Motor Drive* oleh **Davide Bucci**, CC-BY 4.0. Dipakai tanpa perubahan geometri; jumper diatur di Blender.
- *TT Motor (reference)* oleh **Tom3r**, CC-BY 4.0. Dikonversi STEP → GLB.
- *panBot - Encoder Disk TT-Motor* oleh **ZDKderPHB**, CC-BY-NC-SA 4.0. Hanya untuk keperluan praktikum non-komersial.

## Catatan Perubahan Sumber

- Roda Printables 306463 (*Wheel and Tyre for TT motor*, CC0) dicoba lalu **ditolak**: ban berukuran Ø60×12 mm, sedangkan
  `Config.h` (`DIAMETER_RODA_MM`) dan roda TT asli berukuran Ø65×26 mm. Roda dibuat prosedural.
- Model L298N (Printables 796750) dibuat dalam satuan cm dan menyertakan objek penggaris `righelloDM`; keduanya ditangani di `anchors.json`.
- Model MPU6050 (KiCad-Simple) menyertakan socket header betina di bawah papan; objek `PinSocket` dibuang saat import.
- Komponen tanpa sumber unduhan tanpa login dibuat secara prosedural di `tools/blender/bl_parts.py`:
  roda TT Ø65, FC-51, LM393 (FC-03), sasis akrilik, saklar rocker, buzzer, breadboard half-size, caster, sel 18650, dan kapasitor 100 nF.
