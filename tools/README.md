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
python3 tools/simulate_navigation.py
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
