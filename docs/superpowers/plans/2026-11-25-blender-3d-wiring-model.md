# Model 3D Rangkaian Robot + Perkabelan (Blender) — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Membangun `build/robot_3d.blend` dan render PNG berisi robot 2WD lengkap, dengan komponen berukuran asli dan 41 kabel yang menancap di pin yang benar sesuai `docs/hardware/wiring-schematic.md`.

**Architecture:** Bagian yang bisa dites tanpa Blender ditulis sebagai modul Python murni di `tools/blender/` dan dites di `make test`: daftar kabel (netlist), tata letak, rute kabel, dan manifest sumber model. Bagian yang bergantung pada Blender ditulis di modul `bl_*.py` dan dijalankan lewat `blender -b -P tools/blender/build_robot.py`. Skrip build ini memeriksa hasilnya sendiri (jumlah kabel, posisi ujung kabel, tabrakan, dimensi) dan keluar dengan status non-zero jika ada yang gagal. Model unduhan diambil oleh `fetch_models.py` ke `assets/3d/vendor/` (di-gitignore). File STEP dikonversi menjadi GLB oleh `convert_step.py` di `.venv/`.

**Tech Stack:** Python 3 (stdlib, `unittest`), Blender 5.3.0 `bpy` (Python 3.13 bawaan), `cadquery-ocp` (OCP/OpenCascade) di `.venv/`, Printables GraphQL API, raw GitHub.

**Spec:** `docs/superpowers/specs/2026-11-25-blender-3d-wiring-model-design.md`

## Global Constraints

- Varian yang dimodelkan hanya **Arduino Uno R3 + modul L298N**. L293D, ESP32, dan HC-05 tidak dimodelkan.
- Jumlah kabel tepat **41**. Pin `+5V` L298N dan `AD0` MPU6050 **tidak disambung**. Jumper ENA/ENB **dilepas**, jumper 5V-EN **terpasang**.
- Roda Ø**65 mm** dan jarak antar-pusat roda **135 mm** (`Config.h` `DIAMETER_RODA_MM`, `JARAK_RODA_MM`).
- Sasis akrilik 2WD satu lantai **210×150×3 mm**.
- Ukuran utama setiap komponen meleset paling jauh **±1 mm** dari datasheet. Ujung kabel paling jauh **≤0.5 mm** dari anchor pin.
- Frame robot: **+X depan, +Y kiri, +Z atas, titik asal di tengah poros roda**. Modul murni bekerja dalam **mm**. Scene Blender memakai **meter** (×0.001).
- Warna kabel (dari `wiring_schematic_uno_l298n.svg`): 7.4V `#dc2626`, 5V `#ea580c`, GND `#1e293b`, sinyal motor (D4–D9) `#2563eb`, keluaran motor OUT1–4 `#b45309`, sensor (encoder/sonar/IR) `#059669`, I2C `#7c3aed`, antarmuka (START/buzzer) `#0891b2`.
- Python mengikuti `CODING_STANDARDS.md` §6: type hint pada semua fungsi publik, `@dataclass` untuk data multi-atribut, dan lolos `tools/check_python_standards.py`.
- `make check` **tidak boleh** butuh jaringan maupun Blender. Target `blender_fetch` dan `blender_model` berdiri sendiri.
- **Tidak ada file model unduhan yang di-commit.** `assets/3d/vendor/`, `assets/3d/cache/`, `.venv/`, dan `build/` di-gitignore. Yang di-commit hanya `sources.json`, `anchors.json`, `SOURCES.md`, dan render PNG di `docs/hardware/renders/`.
- Path repo mengandung spasi (`POLSRI D3 TEKKOM`). Semua path di Makefile dan argumen CLI harus diberi tanda kutip.
- Path Blender: `BLENDER ?= /Applications/Blender.app/Contents/MacOS/Blender`.

## Review Focus

1. **Hash file upstream berubah atau URL mati.** `fetch_models.py` harus gagal dengan pesan yang menyebut nama file, dan tidak boleh diam-diam memakai file yang berbeda. Dites di Task 2 (`test_verify_sha256_mismatch_names_file`).
2. **Membangun model tanpa fetch lebih dulu** (misalnya di clone baru). Build harus berhenti dengan pesan `jalankan: make blender_fetch` yang mendaftar file yang hilang, bukan traceback `bpy`. Dites di Task 2 (`test_missing_assets_lists_absent_files`).
3. **Skala model unduhan salah** (STL dalam inch/cm, GLB dalam meter). Komponen akan tampak besar atau kecil tapi tetap "jalan". Build membandingkan bbox setiap model dengan `footprint_mm` di layout (±1 mm). Dites di Task 6 (`check_dimensions`).
4. **Path repo mengandung spasi.** Make target dan argumen Blender yang tidak dikutip akan pecah. Diverifikasi di Task 8 dengan menjalankan `make blender_model` langsung dari path repo asli.
5. **Kabel menembus badan komponen.** Ini anti-tujuan spec. Rute harus berada di atas komponen tertinggi (dites di Task 4, `test_route_clears_tallest_top_component`), dan build memeriksa tabrakan dengan BVH (Task 7).

---

### Task 0: Branch & Prasyarat

**Files:** tidak ada.

- [ ] **Step 1:** Jalankan `git status --short`. Jika `Makefile`, `README.md`, atau `.gitignore` masih punya perubahan milik pemilik yang belum di-commit, **berhenti dan minta pemilik meng-commit atau men-stash** perubahan tersebut. Task 1, 2, dan 8 mengubah file-file itu, dan perubahan pemilik tidak boleh ikut ter-commit.
- [ ] **Step 2:** `git checkout -b feat/blender-3d-model`.
- [ ] **Step 3:** Commit spec dan plan.

```bash
git add docs/superpowers/specs/2026-11-25-blender-3d-wiring-model-design.md docs/superpowers/plans/2026-11-25-blender-3d-wiring-model.md
git commit -m "docs(blender): add 3D wiring model design and plan"
```

---

### Task 1: Netlist (sumber kebenaran perkabelan)

**Files:**
- Create: `tools/blender/__init__.py` (kosong), `tools/blender/netlist.py`
- Create: `tests/test_blender_netlist.py`
- Modify: `Makefile`, target `test`: tambahkan `$(TEST_TIMEOUT) python3 tests/test_blender_netlist.py` setelah `test_auto_calibrate.py`

**Interfaces — Produces:**
```python
PinKind = Literal["power7v4", "power5v", "gnd", "signal", "motor"]
@dataclass(frozen=True)
class PinRef:
    component: str; pin: str
@dataclass(frozen=True)
class Wire:
    net: str; src: PinRef; dst: PinRef; color: str
    stage: int                 # 1..5 sesuai Tahap di wiring-schematic.md
    src_end: Literal["dupont", "bare"]; dst_end: Literal["dupont", "bare"]
COMPONENTS: Dict[str, Dict[str, PinKind]]   # id komponen -> {nama pin: jenis}
MULTI_USE_PINS: FrozenSet[PinRef]           # {PinRef("switch", "OUT")}
def build_netlist() -> List[Wire]
def wire_name(w: Wire) -> str   # f"wire_{net}_{src.component}.{src.pin}_{dst.component}.{dst.pin}"
```

ID komponen dan pinnya (dipakai di semua task berikutnya):
- `uno`: `D2`..`D13`, `A0`..`A5`, `VIN`, `5V`, `GND`
- `l298n`: `+12V`, `GND`, `+5V`, `ENA`, `IN1`, `IN2`, `IN3`, `IN4`, `ENB`, `OUT1`..`OUT4`
- `batt`: `+`, `-`. `switch`: `IN`, `OUT`. `motor_l` / `motor_r`: `T1`, `T2`
- `bb`: lubang rel `P5V_1`..`P5V_25` (jenis `power5v`) dan `GND_1`..`GND_25` (jenis `gnd`). Setiap kabel ke rel memakai lubang berikutnya secara berurutan.
- `enc_l` / `enc_r`: `VCC`, `GND`, `D0`, `A0`. `sonar`: `VCC`, `Trig`, `Echo`, `GND`
- `mpu`: `VCC`, `GND`, `SCL`, `SDA`, `XDA`, `XCL`, `AD0`, `INT`. `ir_l` / `ir_r`: `OUT`, `GND`, `VCC`
- `btn`: `1`, `2`. `buzzer`: `+`, `-`

Nama net: `vbat`, `v5`, `gnd`, `ena`, `in1`, …, `out1`..`out4`, `enc_l`, `enc_r`, `trig`, `echo`, `sda`, `scl`, `ir_l`, `ir_r`, `start`, `buzzer`.
Ujung kabel: `bare` untuk terminal sekrup L298N (`+12V`, `GND`, `OUT1-4`), `batt`, `switch`, dan `motor_*` (solder). Sisanya `dupont`.

- [ ] **Step 1: Tulis tes yang gagal** di `tests/test_blender_netlist.py` (pola `unittest` + `sys.path.insert` seperti `tests/test_auto_calibrate.py`):
  - `test_total_wire_count`: `len(build_netlist()) == 41`.
  - `test_arduino_pins_match_config_h`: parse blok `#if BOARD_TYPE == BOARD_ARDUINO_UNO` di `Config.h`. Untuk setiap `PIN_X n` di peta berikut, harus ada kabel antara `PinRef("uno", nama)` dan pasangannya, dengan `nama = f"D{n}"` jika n≤13, atau `f"A{n-14}"` jika 14≤n≤19:
    `PIN_ENA→l298n.ENA, PIN_IN1→l298n.IN1, PIN_IN2→l298n.IN2, PIN_IN3→l298n.IN3, PIN_IN4→l298n.IN4, PIN_ENB→l298n.ENB, PIN_ENC_KIRI→enc_l.D0, PIN_ENC_KANAN→enc_r.D0, PIN_TRIG→sonar.Trig, PIN_ECHO→sonar.Echo, PIN_BUZZER→buzzer.+, PIN_BUTTON_START→btn.1, PIN_IR_KIRI→ir_l.OUT, PIN_IR_KANAN→ir_r.OUT, PIN_SDA→mpu.SDA, PIN_SCL→mpu.SCL`.
  - `test_wiring_doc_matches_netlist`: parse `docs/hardware/wiring-schematic.md`:
    - (a) 6 baris tabel Tahap 2 → pasangan `(uno.Dx, l298n.Y)`.
    - (b) Butir Tahap 3–5 di bawah judul komponen yang dipetakan: `Encoder Kiri→enc_l`, `Encoder Kanan→enc_r`, `HC-SR04→sonar`, `MPU6050→mpu`, `FC-51 Kiri→ir_l`, `FC-51 Kanan→ir_r`, `Push Button→btn`, `Buzzer→buzzer`. Setiap butir `<pin> $\rightarrow$ <tujuan>` dinormalkan menjadi tujuan `"+5V"` (Bus +5V), `"GND"` (Star Ground Hub), atau nama pin Uno. `Kaki 1`→`1`, `Kaki 2`→`2`, `Kaki +`→`+`, `Kaki -`→`-`. Butir `AD0` dilewati.
    - Himpunan hasil parse **sama persis** dengan himpunan yang diturunkan dari netlist (rel `bb.P5V_*`→`"+5V"`, `bb.GND_*`→`"GND"`).
  - `test_power_stage_wires`: himpunan 7 kabel daya sama persis dengan: `batt.-→bb.GND_*`, `batt.+→switch.IN`, `switch.OUT→uno.VIN`, `switch.OUT→l298n.+12V`, `uno.GND→bb.GND_*`, `l298n.GND→bb.GND_*`, `uno.5V→bb.P5V_*`.
  - `test_motor_outputs`: `motor_l.{T1,T2}` ke `l298n.{OUT1,OUT2}`, dan `motor_r.{T1,T2}` ke `l298n.{OUT3,OUT4}`.
  - `test_rails_only_take_matching_power_pins`: setiap kabel yang menyentuh `bb.P5V_*` punya ujung lain berjenis `power5v`. Setiap kabel yang menyentuh `bb.GND_*` punya ujung lain berjenis `gnd`.
  - `test_no_pin_used_twice`: setiap `PinRef` muncul paling banyak sekali, kecuali yang ada di `MULTI_USE_PINS`.
  - `test_unconnected_pins`: `l298n.+5V` dan `mpu.AD0` tidak muncul di netlist.
  - `test_all_pinrefs_exist`: setiap `PinRef` ada di `COMPONENTS`.
  - `test_wire_colors`: setiap net berwarna sesuai tabel warna di Global Constraints (vbat→`#dc2626`, v5→`#ea580c`, gnd→`#1e293b`, ena/in*/enb→`#2563eb`, out*→`#b45309`, enc_*/trig/echo/ir_*→`#059669`, sda/scl→`#7c3aed`, start/buzzer→`#0891b2`).
  - `test_wire_names_unique`.
- [ ] **Step 2:** `python3 tests/test_blender_netlist.py` → FAIL (`ModuleNotFoundError: tools.blender.netlist`).
- [ ] **Step 3:** Implementasikan `tools/blender/netlist.py` sesuai interface di atas. Daftar kabel ditulis eksplisit per tahap, bukan dihasilkan dari dokumen. Parsing dokumen hanya ada di tes.
- [ ] **Step 4:** `python3 tests/test_blender_netlist.py` → semua OK. Lalu `make lint` → lulus.
- [ ] **Step 5:** Commit.

```bash
git add tools/blender/__init__.py tools/blender/netlist.py tests/test_blender_netlist.py Makefile
git commit -m "feat(blender): add wiring netlist cross-checked against Config.h and wiring docs"
```

---

### Task 2: Manifest Sumber Model + Fetcher

**Files:**
- Create: `assets/3d/sources.json`, `assets/3d/SOURCES.md`, `tools/blender/assets.py`, `tools/blender/fetch_models.py`
- Create: `tests/test_blender_assets.py`
- Modify: `.gitignore` (tambah `assets/3d/vendor/`, `assets/3d/cache/`), `Makefile` (target `blender_fetch`, dan tambahkan tes baru ke `test`)

**Interfaces — Produces:**
```python
@dataclass(frozen=True)
class ModelSource:
    key: str                 # "uno", "l298n", "sonar", "mpu", "motor_tt", "wheel", "enc_disk", "batt_holder", "btn"
    filename: str            # nama file di assets/3d/vendor/
    kind: Literal["github", "printables"]
    url: str                 # github: raw URL dengan commit SHA; printables: "" (dipakai print_id/file_id)
    print_id: int; file_id: int   # 0 untuk github
    sha256: str              # 64 hex, wajib
    license: str; attribution: str
VENDOR_DIR: Path   # <repo>/assets/3d/vendor
CACHE_DIR: Path    # <repo>/assets/3d/cache
def load_sources(path: Path = ...) -> List[ModelSource]
def verify_sha256(path: Path, expected: str) -> None      # raise ValueError berisi nama file, hash expected, dan hash actual
def missing_assets(sources: List[ModelSource], vendor_dir: Path) -> List[str]
```
`fetch_models.py` CLI: `python3 tools/blender/fetch_models.py [--pin]`. Opsi `--pin` mengisi atau memperbarui `sha256` di `sources.json`, dan hanya dipakai saat menyusun manifest pertama kali. Printables diunduh lewat `POST https://api.printables.com/graphql/` dengan mutation `getDownloadLink(id: file_id, printId: print_id, fileType: stl|other, source: model_detail)` (pilih `fileType` sesuai daftar tempat file itu muncul pada `print { stls otherFiles }`). Hanya pakai `urllib` dari stdlib.

Isi manifest (dari spec §6):
- GitHub `Sajitha-Aldeniya/KiCad-Simple-Libraries`, dipin ke SHA commit `HEAD` saat ini (ambil lewat `gh api repos/Sajitha-Aldeniya/KiCad-Simple-Libraries/commits/HEAD -q .sha`):
  - `3dmodels/SL_Development_Boards.3dshapes/ArduinoUNO_3D.step` (uno)
  - `SL_Breakout_Boards.3dshapes/HC-SR04.step` (sonar)
  - `SL_Breakout_Boards.3dshapes/MPU6050.step` (mpu)
  - `SL_Devices.3dshapes/Push_Button_W12mm.step` (btn)
  - Lisensinya `"none (personal use, not redistributed)"`.
- Printables:
  - 796750 `L298N.obj` (l298n, CC-BY)
  - 1336089 `TT Motor.step` (motor_tt, CC-BY)
  - 306463 roda (wheel, CC0; pilih file roda+ban yang ada di print tersebut)
  - 129515 (enc_disk, CC-BY-NC-SA)
  - 407681 `18650x2_w_tab.stl` (batt_holder, CC0)

- [ ] **Step 1: Tulis tes yang gagal** di `tests/test_blender_assets.py`:
  - `test_sources_schema`: setiap entri punya `sha256` 64 hex, `license` tidak kosong, `key` unik, dan semua key spec ada.
  - `test_verify_sha256_ok` pada file sementara.
  - `test_verify_sha256_mismatch_names_file`: `ValueError`, dan pesannya memuat nama file.
  - `test_missing_assets_lists_absent_files`: direktori kosong → mengembalikan semua `filename`.
  - `test_gitignore_excludes_vendor_and_cache`: `.gitignore` memuat `assets/3d/vendor/` dan `assets/3d/cache/`.
- [ ] **Step 2:** `python3 tests/test_blender_assets.py` → FAIL.
- [ ] **Step 3:** Implementasikan `assets.py` dan `fetch_models.py`. Tulis `sources.json` dengan `sha256` sementara, jalankan `python3 tools/blender/fetch_models.py --pin`, lalu periksa setiap file yang terunduh tidak kosong dan berformat benar (`head -c 200`: STEP diawali `ISO-10303-21`). Jika sebuah sumber mati atau isinya salah, ganti dengan sumber tanpa login lain **atau** tandai komponen itu menjadi prosedural (hapus dari manifest, tambahkan ke Task 5). Catat perubahan tersebut di `SOURCES.md`.
- [ ] **Step 4:** Tulis `SOURCES.md`: tabel key → sumber, URL halaman, lisensi, dan atribusi penulis (untuk CC-BY / CC-BY-NC-SA), beserta catatan bahwa file tidak diredistribusi.
- [ ] **Step 5:** Tambahkan target berikut ke `Makefile`:

```make
blender_fetch:
	python3 tools/blender/fetch_models.py
```

  Lalu tambahkan `blender_fetch` ke `.PHONY`.
- [ ] **Step 6:** `rm -rf assets/3d/vendor && make blender_fetch` → semua file terunduh dan hash cocok. `python3 tests/test_blender_assets.py` → OK. `make lint` → lulus. `git status --short assets/` hanya menampilkan `sources.json` dan `SOURCES.md`.
- [ ] **Step 7:** Commit.

```bash
git add assets/3d/sources.json assets/3d/SOURCES.md tools/blender/assets.py tools/blender/fetch_models.py tests/test_blender_assets.py .gitignore Makefile
git commit -m "feat(blender): add pinned 3D model manifest and hash-verified fetcher"
```

---

### Task 3: Konversi STEP → GLB

**Files:**
- Create: `tools/blender/requirements-cad.txt` (`cadquery-ocp`), `tools/blender/convert_step.py`
- Modify: `tests/test_blender_assets.py` (tambah tes), `Makefile` (target `blender_convert`)

**Interfaces — Produces:**
```python
def glb_path_for(source: ModelSource) -> Path            # CACHE_DIR / f"{source.key}.glb"
def needs_conversion(sources: List[ModelSource]) -> List[ModelSource]   # hanya .step/.stp yang GLB-nya belum ada atau lebih tua
def convert(step_file: Path, out_glb: Path, linear_deflection_mm: float = 0.05) -> None
```
Gunakan OCP: `STEPCAFControl_Reader` (dengan warna dan nama aktif) → `XCAFDoc` → `BRepMesh_IncrementalMesh(shape, 0.05, False, 0.3)` → `RWGltf_CafWriter` (biner). Atur konversi koordinat sistem agar keluarannya dalam **meter** dengan sumbu Z-up dipertahankan: `RWMesh_CoordinateSystemConverter`, input unit 0.001 m, input sistem Z-up, output sistem glTF Y-up. Importer glTF Blender akan mengembalikannya ke Z-up. Import `OCP` dilakukan di dalam `convert()`, supaya `py_compile` dan tes tanpa `.venv` tetap jalan.

- [ ] **Step 1: Tes gagal**:
  - `test_glb_path_for_uses_key`.
  - `test_needs_conversion_skips_non_step_and_fresh_cache`: manifest campuran STEP/STL dengan GLB palsu yang lebih baru → hanya mengembalikan STEP yang GLB-nya belum ada.
- [ ] **Step 2:** Jalankan → FAIL.
- [ ] **Step 3:** Implementasikan. Tambahkan target berikut ke Makefile:

```make
blender_convert: blender_fetch
	test -x .venv/bin/python || (python3 -m venv .venv && .venv/bin/pip install -r tools/blender/requirements-cad.txt)
	.venv/bin/python tools/blender/convert_step.py
```

  Jika `cadquery-ocp` tidak punya wheel untuk Python sistem, pakai `python3.12 -m venv`. Catat versi yang dipakai di `requirements-cad.txt`.
- [ ] **Step 4:** `make blender_convert` menghasilkan `assets/3d/cache/{uno,sonar,mpu,btn,motor_tt}.glb`. Verifikasi dengan Blender:
  skrip kecil di scratchpad yang mengimpor `uno.glb` dan mencetak bbox gabungan semua objek dalam mm → dimensi bbox gabungan Uno ≈ 68.6×53.4 mm (±1). Tes Python → OK. `make lint` → lulus.
- [ ] **Step 5:** Commit.

```bash
git add tools/blender/requirements-cad.txt tools/blender/convert_step.py tests/test_blender_assets.py Makefile
git commit -m "feat(blender): convert STEP models to GLB via OCP with colours preserved"
```

---

### Task 4: Tata Letak & Rute Kabel (modul murni)

**Files:**
- Create: `tools/blender/layout.py`, `tools/blender/routing.py`
- Create: `tests/test_blender_layout.py`
- Modify: `Makefile` (target `test`: tambah `tests/test_blender_layout.py` dan `tests/test_blender_assets.py`)

**Interfaces — Produces:**
```python
Vec3 = Tuple[float, float, float]
@dataclass(frozen=True)
class Placement:
    component: str           # id dari netlist.COMPONENTS, ditambah "chassis", "wheel_l", "wheel_r", "disk_l", "disk_r", "caster", "cell_1", "cell_2", "cap_l", "cap_r"
    position_mm: Vec3        # titik asal lokal komponen di frame robot
    rotation_deg: Vec3       # Euler XYZ
    footprint_mm: Vec3       # ukuran bbox lokal (L, W, H) dari datasheet
    zone: Literal["top", "bottom", "front", "left", "right"]
CHASSIS_MM: Vec3 = (210.0, 150.0, 3.0)
WHEEL_DIAMETER_MM: float = 65.0
TRACK_WIDTH_MM: float = 135.0
DUPONT_STRAIGHT_MM: float = 16.0
def placements() -> Dict[str, Placement]
def world_aabb(p: Placement) -> Tuple[Vec3, Vec3]
def route_height_mm() -> float           # z tertinggi dari aabb zona "top" + 10.0
def route(a: Vec3, a_dir: Vec3, b: Vec3, b_dir: Vec3, lane: int, vias: Sequence[Vec3] = ()) -> List[Vec3]
```
Footprint (datasheet, mm): Uno 68.6×53.4×(tinggi model), L298N 43×43×27, HC-SR04 45×20×15 (berdiri, transduser menghadap +X), GY-521 21×16×3, FC-51 32×14×7, LM393 FC-03 32×14×12, breadboard half-size 400 titik 82.5×54.5×8.5 (pakai satu pasang rel atas, 25 lubang kontinu), holder 18650×2 77×41×21, motor TT 70×22.5×18.8, saklar rocker 21×15×18, buzzer Ø12×9.5, push button 12×12×7, caster (tinggi disesuaikan agar sasis rata, yaitu dasar sasis pada z yang sama dengan di poros). Pusat roda di `(0, ±67.5, 0)`. Lantai di `z = -32.5`.

Algoritme `route` (tidak ditentukan oleh signature):
`a` → `a + a_dir*DUPONT_STRAIGHT_MM` → naik vertikal ke `z = route_height_mm() + lane*2.0` → melewati setiap `via` (setiap via didekati dari atas dan bawah secara vertikal) → bergerak horizontal (X dulu, lalu Y) → turun vertikal ke `b + b_dir*DUPONT_STRAIGHT_MM` → `b`. Kabel dari komponen di zona `bottom` memakai `vias`, yaitu titik slot sasis yang didefinisikan di `layout.CHASSIS_SLOTS_MM: Dict[str, Vec3]` (slot kiri, kanan, dan belakang).

- [ ] **Step 1: Tes gagal** `tests/test_blender_layout.py`:
  - `test_wheel_track_and_diameter`: posisi `wheel_l` y = +67.5 dan `wheel_r` y = −67.5 (±0.01). Footprint roda: diameter 65.
  - `test_top_components_do_not_overlap`: AABB antar-komponen di zona `top` tidak beririsan.
  - `test_top_components_inside_chassis`: proyeksi XY setiap komponen `top` berada di dalam 210×150.
  - `test_mpu_near_axle_center`: jarak XY `mpu` ke (0,0) ≤ 30 mm, dan `rotation_deg` x = y = 0 (rata).
  - `test_sonar_faces_forward_at_front`: `sonar` x adalah yang terbesar di antara semua komponen, dengan zona `front`.
  - `test_ir_sensors_face_outward`: zona `ir_l` adalah `left` dengan y > 0, dan zona `ir_r` adalah `right` dengan y < 0.
  - `test_every_netlist_component_placed`: setiap key `netlist.COMPONENTS` ada di `placements()`.
  - `test_route_endpoints_and_straight_exit`: `pts[0]==a`, `pts[-1]==b`, `pts[1]==a+a_dir*16`, `pts[-2]==b+b_dir*16`.
  - `test_route_clears_tallest_top_component`: semua titik selain 2 titik pertama dan 2 titik terakhir (beserta segmen naik/turun) mencapai z ≥ `route_height_mm()` pada bagian horizontalnya.
  - `test_route_lanes_separate`: lane 0 dan lane 3 berbeda z sebesar 6.0 mm.
  - `test_route_passes_vias`: setiap via muncul di polyline.
- [ ] **Step 2:** Jalankan → FAIL.
- [ ] **Step 3:** Implementasikan `layout.py` (koordinat persisnya ditentukan implementer, dalam batas zona spec §9) dan `routing.py`.
- [ ] **Step 4:** Tes → OK. `make test` → lulus. `make lint` → lulus.
- [ ] **Step 5:** Commit.

```bash
git add tools/blender/layout.py tools/blender/routing.py tests/test_blender_layout.py Makefile
git commit -m "feat(blender): add chassis layout and wire routing with clearance tests"
```

---

### Task 5: Komponen Prosedural (Blender)

**Files:**
- Create: `tools/blender/bl_common.py`, `tools/blender/bl_parts.py`
- Create: `tools/blender/check_parts.py` (skrip verifikasi headless)

**Interfaces — Produces:**
```python
Anchor = Tuple[Vec3, Vec3]   # (posisi lokal mm, arah keluar kabel, unit vektor)
@dataclass
class PartResult:
    obj: "bpy.types.Object"          # root (Empty) dengan child mesh
    anchors: Dict[str, Anchor]       # nama pin -> anchor, nama pin = netlist.COMPONENTS
# bl_common.py
def mm(v: Vec3) -> "mathutils.Vector"                      # ×0.001
def material(name: str, hex_color: str, roughness: float = 0.5, transmission: float = 0.0, alpha: float = 1.0) -> "bpy.types.Material"
def box(name: str, size_mm: Vec3, center_mm: Vec3, mat: "bpy.types.Material", parent: Optional["bpy.types.Object"] = None) -> "bpy.types.Object"
def cylinder(name: str, radius_mm: float, depth_mm: float, center_mm: Vec3, axis: Literal["X","Y","Z"], mat: ..., parent: ... = None) -> "bpy.types.Object"
def header_pins(name: str, count: int, origin_mm: Vec3, axis: Literal["X","Y"], pin_dir: Vec3, parent: ...) -> List[Vec3]   # pin 0.64 mm persegi, pitch 2.54, housing hitam 2.54; mengembalikan posisi ujung pin
# bl_parts.py
def make_chassis() -> PartResult
def make_fc51(name: str) -> PartResult        # anchors: OUT, GND, VCC
def make_lm393(name: str) -> PartResult       # anchors: VCC, GND, D0, A0
def make_switch() -> PartResult               # IN, OUT
def make_buzzer() -> PartResult               # +, -
def make_breadboard() -> PartResult           # P5V_1..P5V_25, GND_1..GND_25
def make_caster() -> PartResult
def make_cell(name: str) -> PartResult        # 18650, Ø18×65, tanpa anchor
def make_cap_100nf(name: str) -> PartResult   # keramik biru kode "104", tanpa anchor
```
Detail visual setiap part ditentukan implementer. Ukuran bbox wajib sesuai `layout.Placement.footprint_mm` (±1 mm). Ketentuan yang wajib:
- FC-51: PCB biru, LED IR bening dan fotodioda hitam di ujung, trimpot biru, header 3 pin berurutan **OUT, GND, VCC**.
- LM393: PCB biru, fork optik hitam dengan celah 5 mm, header 4 pin **VCC, GND, D0, A0**.
- Breadboard: rel merah (+) dan biru (−) diberi label.
- Sasis: material akrilik bening (`transmission=1.0`), lubang M3, dan slot kabel di `layout.CHASSIS_SLOTS_MM`.

- [ ] **Step 1:** Tulis `check_parts.py`. Skrip ini membangun setiap part pada scene kosong dan memeriksa:
  - bbox dunia (mm) sesuai footprint layout ±1.
  - Set nama anchor sama persis dengan `netlist.COMPONENTS[id]`, dengan pemetaan: `ir_*`→fc51, `enc_*`→lm393, `switch`, `buzzer`, `bb`.
  - Setiap anchor berada di dalam bbox part.

  Skrip mencetak `PARTS OK` atau daftar error lalu `sys.exit(1)`.
- [ ] **Step 2:** `"$BLENDER" -b --factory-startup -P tools/blender/check_parts.py` → gagal (modul belum ada).
- [ ] **Step 3:** Implementasikan `bl_common.py` dan `bl_parts.py`. Di bagian atas setiap skrip Blender, tambahkan root repo ke `sys.path` supaya `tools.blender.*` bisa diimpor.
- [ ] **Step 4:** Jalankan ulang → `PARTS OK`. Render close-up setiap part ke `build/renders/parts/<part>.png` (EEVEE, 800×600), lalu **buka dan lihat** PNG-nya untuk memastikan urutan label pin tampak benar. `make lint` → lulus.
- [ ] **Step 5:** Commit.

```bash
git add tools/blender/bl_common.py tools/blender/bl_parts.py tools/blender/check_parts.py
git commit -m "feat(blender): add procedural FC-51, LM393, chassis, breadboard, switch, buzzer, caster parts"
```

---

### Task 6: Import Model Unduhan + Anchor Pin

**Files:**
- Create: `tools/blender/bl_import.py`, `assets/3d/anchors.json`, `tools/blender/measure_model.py`
- Modify: `tools/blender/check_parts.py` (ikut memeriksa part hasil import)

**Interfaces:**
- Consumes: `assets.load_sources`, `assets.missing_assets`, `convert_step.glb_path_for`, `bl_common.PartResult`, `layout.placements`
- Produces:
```python
def import_part(key: str, name: str) -> PartResult      # key = ModelSource.key
def check_dimensions(result: PartResult, expected_mm: Vec3, tol_mm: float = 1.0) -> List[str]
```
Format `anchors.json`, satu entri per key:
```json
{"uno": {"scale": 1.0, "rotation_deg": [0, 0, 0], "offset_mm": [0, 0, 0],
         "pins": {"D9": {"pos": [x, y, z], "dir": [0, 0, 1]}, "...": {}}}}
```
- `scale` dikalikan setelah konversi satuan bawaan: GLB sudah dalam meter → 1.0; STL/OBJ diasumsikan mm → 0.001 dikali `scale`.
- `offset_mm` dan `rotation_deg` menormalkan model agar titik asalnya di pojok/pusat yang konsisten dengan layout, dengan PCB rata dan header menghadap +Z.
- Pin yang wajib diberi anchor: semua pin di `netlist.COMPONENTS` untuk `uno` (termasuk `GND`, `5V`, `VIN` di header power), `l298n` (terminal sekrup dan header kontrol), `sonar`, `mpu`, `btn`, serta `motor_tt` (`T1`, `T2`, tab solder).
- Roda, cakram, dan holder tidak punya pin, kecuali holder: `+` dan `-` (ujung kabel bawaan holder).

`measure_model.py <key>` mengimpor model, mencetak bbox dalam mm, lalu merender tampak atas ortografis 1600 px ke `build/renders/measure/<key>.png` dengan grid 2.54 mm. Ini alat bantu untuk mengukur letak pin pertama. Pin selanjutnya dihitung dari pitch 2.54.

- [ ] **Step 1:** Tambahkan ke `check_parts.py` pemeriksaan untuk setiap key manifest:
  - `check_dimensions` sesuai footprint layout. Untuk roda, diameter 65 ±1.
  - Set anchor sama persis dengan pin netlist.
  - Jika `missing_assets` tidak kosong, cetak `jalankan: make blender_fetch && make blender_convert` beserta daftar filenya, lalu exit 1.
- [ ] **Step 2:** Jalankan `check_parts.py` → gagal.
- [ ] **Step 3:** Implementasikan `bl_import.py` (GLB → `bpy.ops.import_scene.gltf`, STL → `bpy.ops.wm.stl_import`, OBJ → `bpy.ops.wm.obj_import`; gabungkan semua objek impor di bawah satu Empty root). Ukur setiap model dengan `measure_model.py` dan isi `anchors.json`. Untuk Uno, cocokkan dengan gambar mekanik resmi Arduino UNO R3 (pin D0 header digital dan header power).
- [ ] **Step 4:** `check_parts.py` → `PARTS OK`. Render close-up header setiap model dengan bola penanda kecil (Ø1 mm, merah) di setiap anchor ke `build/renders/anchors/<key>.png`. **Buka setiap PNG dan pastikan setiap penanda duduk di pin yang labelnya benar** (silkscreen model, atau urutan dari datasheet jika model tidak bertuliskan label). `make lint` → lulus.
- [ ] **Step 5:** Commit.

```bash
git add tools/blender/bl_import.py tools/blender/measure_model.py tools/blender/check_parts.py assets/3d/anchors.json
git commit -m "feat(blender): import downloaded models with measured pin anchors and dimension checks"
```

---

### Task 7: Kabel di Blender + Pemeriksaan Scene

**Files:**
- Create: `tools/blender/bl_wires.py`, `tools/blender/bl_checks.py`

**Interfaces:**
- Consumes: `netlist.build_netlist`, `netlist.wire_name`, `routing.route`, `layout.CHASSIS_SLOTS_MM`, `PartResult.anchors` (setelah ditransformasikan ke dunia)
- Produces:
```python
def world_anchor(part: PartResult, pin: str) -> Anchor        # posisi & arah dunia (mm)
def make_wire(w: Wire, points_mm: List[Vec3]) -> "bpy.types.Object"   # kurva bevel Ø1.6 mm + housing dupont/ujung telanjang
def assign_lanes(wires: List[Wire]) -> Dict[str, int]        # wire_name -> lane, kabel satu bundel (stage sama) di lane berdekatan
def check_scene(parts: Dict[str, PartResult], wire_objs: Dict[str, "bpy.types.Object"]) -> List[str]
```
- `make_wire`: kurva `POLY` dengan sudut dibulatkan (fillet radius 4 mm, atau Bezier `AUTO`), `bevel_depth=0.0008`, dengan material warna `w.color`.
  - Ujung `dupont`: housing hitam 2.54×2.54×14 mm sejajar `dir` yang menutupi pin.
  - Ujung `bare`: 5 mm tembaga terbuka masuk ke terminal.
- `check_scene` mengembalikan error jika:
  - (1) Jumlah objek `wire_*` ≠ 41.
  - (2) Ujung kurva berjarak > 0.5 mm dari `world_anchor`.
  - (3) Mesh kabel (setelah dikonversi, **tidak termasuk** 18 mm di setiap ujung) beririsan dengan mesh komponen mana pun (`mathutils.bvhtree.BVHTree.overlap`). Sasis dikecualikan di titik via slot.

- [ ] **Step 1:** Tulis `bl_checks.check_scene` dan skrip uji sementara `build/smoke_wires.py` (tidak di-commit). Skrip ini menaruh `uno` + `sonar` dan satu kabel `trig` yang ujungnya sengaja digeser 2 mm. Jalankan dengan Blender headless → `check_scene` melaporkan error (2) untuk kabel itu.
- [ ] **Step 2:** Implementasikan `bl_wires.py`. Jalankan smoke dengan kabel yang benar → tidak ada error.
- [ ] **Step 3:** `make lint` → lulus.
- [ ] **Step 4:** Commit (tanpa `build/`).

```bash
git add tools/blender/bl_wires.py tools/blender/bl_checks.py
git commit -m "feat(blender): build dupont/bare-end wires and scene integrity checks"
```

---

### Task 8: Rakitan Penuh, Render, Make Target, Dokumentasi

**Files:**
- Create: `tools/blender/build_robot.py`, `docs/hardware/renders/robot_iso.png`, `robot_top.png`, `robot_bottom.png`
- Modify: `Makefile` (target `blender_model`, variabel `BLENDER`), `tools/README.md` (bagian baru "5. Model 3D Rangkaian (Blender)"), `docs/hardware/wiring-schematic.md` (satu baris tautan ke render)

**Interfaces:**
- Consumes: semua modul di atas.
- Produces CLI: `"$BLENDER" -b --factory-startup -P tools/blender/build_robot.py -- --out build/robot_3d.blend --renders docs/hardware/renders [--no-render]`

Isi `build_robot.py` secara berurutan:
1. Scene kosong, satuan metrik dengan scale 1.0.
2. Cek `missing_assets` → pesan lalu exit 1.
3. Bangun semua part (prosedural dan impor) sesuai `layout.placements()`. Lepas jumper ENA/ENB dan pasang jumper 5V-EN (sembunyikan atau tampilkan objek jumper di model L298N; jika model OBJ tidak memisahkan jumper, tambahkan jumper 5V-EN prosedural dan pastikan tidak ada jumper di ENA/ENB).
4. Kapasitor 100 nF disolder di terminal kedua motor.
5. Bangun 41 kabel.
6. `check_scene` → jika ada error, cetak semuanya lalu exit 1.
7. Simpan `.blend`. Kumpulkan objek ke collection `Chassis`, `Electronics`, `Wires/<stage>` supaya bisa disembunyikan per tahap.
8. Render EEVEE 1920×1200: iso (kamera dari depan-kiri-atas), top (ortografis), dan bottom (ortografis dari bawah). Latar putih dan pencahayaan studio tiga titik.
9. Cetak `ROBOT MODEL OK: 41 wires, <n> parts`.

- [ ] **Step 1:** Tambahkan ke Makefile:

```make
BLENDER ?= /Applications/Blender.app/Contents/MacOS/Blender
blender_model: blender_convert
	"$(BLENDER)" -b --factory-startup -P tools/blender/build_robot.py -- --out build/robot_3d.blend --renders docs/hardware/renders
```

  Tambahkan juga ke `.PHONY`.
- [ ] **Step 2:** Implementasikan `build_robot.py`.
- [ ] **Step 3:** Dari path repo asli (yang mengandung spasi) jalankan `make blender_model` → output terakhir `ROBOT MODEL OK: 41 wires, …`, dan `build/robot_3d.blend` serta 3 PNG ada.
- [ ] **Step 4:** **Buka ketiga PNG** dan periksa:
  - Semua komponen ada di zona yang benar.
  - Tidak ada kabel yang tampak menembus papan.
  - Warna kabel sesuai tabel warna.
  - Render close-up (`--renders build/renders/closeup`, tambahkan kamera per header Uno, L298N, dan setiap sensor) memperlihatkan housing dupont di pin yang benar.
- [ ] **Step 5:** Uji jalur kegagalan: `mv assets/3d/vendor assets/3d/vendor.bak && "$BLENDER" -b --factory-startup -P tools/blender/build_robot.py -- --out /tmp/x.blend --no-render` → pesan `jalankan: make blender_fetch` dan exit ≠ 0. Kembalikan folder dengan `mv assets/3d/vendor.bak assets/3d/vendor`.
- [ ] **Step 6:** Perbarui `tools/README.md`. Isinya: prasyarat (Blender 5.3, koneksi untuk fetch), urutan perintah `make blender_fetch` → `make blender_model`, cara membuka `build/robot_3d.blend` dan menyembunyikan kabel per tahap lewat collection, serta tautan ke `assets/3d/SOURCES.md`. Tambahkan satu baris tautan render di `wiring-schematic.md`.
- [ ] **Step 7:** `make check` → lulus (tanpa Blender dan tanpa jaringan).
- [ ] **Step 8:** Commit.

```bash
git add tools/blender/build_robot.py docs/hardware/renders/robot_iso.png docs/hardware/renders/robot_top.png docs/hardware/renders/robot_bottom.png Makefile tools/README.md docs/hardware/wiring-schematic.md
git commit -m "feat(blender): assemble full robot 3D model with 41 verified wires and renders"
```
