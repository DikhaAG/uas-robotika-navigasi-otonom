#!/usr/bin/env python3
"""
Friendly line-based serial console for the robot, over USB or HC-05 Bluetooth.

Replaces pyserial's miniterm for day-to-day use: typed text is echoed and
editable, one line is sent per Enter (terminated with a single newline, as the
firmware's ASCII protocol expects), `exit` or Ctrl+C quits and always releases
the port, the port is locked exclusively so a second program cannot silently
steal the replies, and telemetry rows can be written straight to a plottable CSV.

    python3 tools/serial_console.py --port /dev/cu.HC-05
    python3 tools/serial_console.py --port /dev/cu.HC-05 --csv babak1_room2.csv
    python3 tools/serial_console.py --port /dev/cu.HC-05 --send '!PING' --duration 2
"""

import argparse
import os
import sys
import threading
import time
from typing import Any, Callable, List, Optional, TextIO

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from tools.calibration.adapters.transport import DEFAULT_BAUD, port_problem_hints

TELEMETRY_HEADER: str = "t_ms,state,x,y,theta,jarak,iWp"
EXIT_WORDS = ("exit", "quit", "keluar")


def is_telemetry_row(line: str) -> bool:
    """True for a firmware CSV pose row (t_ms,state,x,y,theta,jarak,iWp)."""
    parts = line.split(",")
    return len(parts) == 7 and parts[0].isdigit()


def open_serial_link(port: str, baud: int) -> Any:
    """Opens the port with an exclusive lock so no second program can share it."""
    import serial
    return serial.Serial(port, baud, timeout=0.2, exclusive=True)


class ConsoleSession:
    """Owns an open serial link: forwards typed lines and records every received line."""

    def __init__(self, link: Any, screen: TextIO,
                 log: Optional[TextIO] = None, csv: Optional[TextIO] = None) -> None:
        self.link: Any = link
        self.screen: TextIO = screen
        self.log: Optional[TextIO] = log
        self.csv: Optional[TextIO] = csv
        self._lock: threading.Lock = threading.Lock()
        if self.csv is not None:
            self.csv.write(TELEMETRY_HEADER + "\n")

    def send_line(self, text: str) -> None:
        self.link.write((text.strip() + "\n").encode("ascii", errors="replace"))

    def handle_received(self, raw: bytes) -> None:
        line = raw.decode("ascii", errors="replace").strip()
        if not line:
            return
        with self._lock:
            self.screen.write(line + "\n")
            self.screen.flush()
            if self.log is not None:
                self.log.write(line + "\n")
                self.log.flush()
            if self.csv is not None and is_telemetry_row(line):
                self.csv.write(line + "\n")
                self.csv.flush()

    def pump(self, stop: threading.Event) -> None:
        """Reads lines until stop is set; readline() returns b'' on each read timeout."""
        while not stop.is_set():
            try:
                raw = self.link.readline()
            except Exception as e:  # link dropped (Bluetooth out of range, cable pulled)
                self.screen.write(f"\n[konsol] Koneksi terputus: {e}\n")
                stop.set()
                return
            if raw:
                self.handle_received(raw)


def _interactive_loop(session: ConsoleSession, stop: threading.Event) -> None:
    while not stop.is_set():
        try:
            text = input()
        except (EOFError, KeyboardInterrupt):
            return
        if text.strip().lower() in EXIT_WORDS:
            return
        if text.strip():
            session.send_line(text)


def main(argv: Optional[List[str]] = None,
         open_link: Optional[Callable[[str, int], Any]] = None) -> int:
    parser = argparse.ArgumentParser(description="Konsol serial robot (USB atau Bluetooth HC-05)")
    parser.add_argument("-p", "--port", required=True, help="Port serial, mis. /dev/cu.HC-05 atau /dev/cu.usbmodem14101")
    parser.add_argument("-b", "--baud", type=int, default=DEFAULT_BAUD,
                        help=f"Baud rate, sama dengan SERIAL_BAUD di Config.h (default: {DEFAULT_BAUD})")
    parser.add_argument("--log", help="Simpan SEMUA baris yang diterima ke file teks")
    parser.add_argument("--csv", help="Simpan baris telemetri (t_ms,state,...) ke file CSV siap plot")
    parser.add_argument("--send", action="append", default=[],
                        help="Kirim perintah lalu berjalan non-interaktif (boleh diulang), mis. --send '!PING'")
    parser.add_argument("--duration", type=float, default=None,
                        help="Mode non-interaktif: rekam selama N detik lalu keluar")
    parser.add_argument("--wait", type=float, default=2.0,
                        help="Mode non-interaktif: jeda sebelum --send (USB me-reset Uno saat port dibuka; default 2.0 s)")
    args = parser.parse_args(argv)

    opener = open_link or open_serial_link
    try:
        link = opener(args.port, args.baud)
    except ImportError:
        print("Modul 'pyserial' belum terpasang untuk Python ini. Aktifkan venv proyek: source .venv/bin/activate")
        return 1
    except Exception as e:
        print(f"Gagal membuka port '{args.port}': {e}")
        for hint in port_problem_hints(args.port, e):
            print(f"  - {hint}")
        return 1

    log_file: Optional[TextIO] = open(args.log, "w", encoding="utf-8") if args.log else None
    csv_file: Optional[TextIO] = open(args.csv, "w", encoding="utf-8") if args.csv else None
    session = ConsoleSession(link, sys.stdout, log=log_file, csv=csv_file)
    stop = threading.Event()
    reader = threading.Thread(target=session.pump, args=(stop,), daemon=True)
    interactive = not args.send and args.duration is None
    try:
        print(f"--- Tersambung ke {args.port} ({args.baud} baud) ---")
        if interactive:
            print("--- Ketik perintah lalu Enter (mis. !PING, !DIAG:ALL, 1-4). Keluar: exit atau Ctrl+C ---")
            print("--- Lewat USB, Arduino reset saat port dibuka: tunggu ±2 detik sebelum mengetik ---")
        reader.start()
        if interactive:
            _interactive_loop(session, stop)
        else:
            time.sleep(max(args.wait, 0.0))
            for cmd in args.send:
                session.send_line(cmd)
                time.sleep(0.15)
            time.sleep(max(args.duration or 1.0, 0.0))
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        if reader.is_alive():
            reader.join(timeout=1.0)
        link.close()
        for f in (log_file, csv_file):
            if f is not None:
                f.close()
        print(f"--- Port {args.port} dilepas ---")
        if args.csv:
            print(f"--- Telemetri tersimpan: {args.csv} (plot: python3 tools/plot_trajectory.py {args.csv} -o lintasan.svg) ---")
    return 0


if __name__ == "__main__":
    sys.exit(main())
