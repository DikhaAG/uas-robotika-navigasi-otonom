#!/usr/bin/env python3
"""
Unit tests for the Blender 3D wiring netlist (tools/blender/netlist.py).
The netlist is the single source of truth for the 41 wires in the 3D model.
It is cross-checked against firmware Config.h (Uno branch) and the assembly
guide docs/hardware/wiring-schematic.md so the model can never drift from them.
"""

import os
import re
import sys
import unittest
from collections import Counter
from typing import Dict, List, Optional, Set, Tuple

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, REPO_ROOT)

from tools.blender.netlist import (  # noqa: E402
    COMPONENTS,
    MULTI_USE_PINS,
    PinRef,
    Wire,
    build_netlist,
    wire_name,
)

CONFIG_H = os.path.join(REPO_ROOT, "firmware", "RobotNavigasiOtonom", "Config.h")
WIRING_DOC = os.path.join(REPO_ROOT, "docs", "hardware", "wiring-schematic.md")

CONFIG_PIN_PARTNERS: Dict[str, PinRef] = {
    "PIN_ENA": PinRef("l298n", "ENA"),
    "PIN_IN1": PinRef("l298n", "IN1"),
    "PIN_IN2": PinRef("l298n", "IN2"),
    "PIN_IN3": PinRef("l298n", "IN3"),
    "PIN_IN4": PinRef("l298n", "IN4"),
    "PIN_ENB": PinRef("l298n", "ENB"),
    "PIN_ENC_KIRI": PinRef("enc_l", "D0"),
    "PIN_ENC_KANAN": PinRef("enc_r", "D0"),
    "PIN_TRIG": PinRef("sonar", "Trig"),
    "PIN_ECHO": PinRef("sonar", "Echo"),
    "PIN_BUZZER": PinRef("buzzer", "+"),
    "PIN_BUTTON_START": PinRef("btn", "1"),
    "PIN_IR_KIRI": PinRef("ir_l", "OUT"),
    "PIN_IR_KANAN": PinRef("ir_r", "OUT"),
    "PIN_SDA": PinRef("mpu", "SDA"),
    "PIN_SCL": PinRef("mpu", "SCL"),
}

DOC_HEADINGS: List[Tuple[str, str]] = [
    ("Encoder Kiri", "enc_l"),
    ("Encoder Kanan", "enc_r"),
    ("HC-SR04", "sonar"),
    ("MPU6050", "mpu"),
    ("FC-51 Kiri", "ir_l"),
    ("FC-51 Kanan", "ir_r"),
    ("Push Button", "btn"),
    ("Buzzer", "buzzer"),
]

NET_COLORS: Dict[str, str] = {
    "vbat": "#dc2626",
    "v5": "#ea580c",
    "gnd": "#1e293b",
    "ena": "#2563eb",
    "in1": "#2563eb",
    "in2": "#2563eb",
    "in3": "#2563eb",
    "in4": "#2563eb",
    "enb": "#2563eb",
    "out1": "#b45309",
    "out2": "#b45309",
    "out3": "#b45309",
    "out4": "#b45309",
    "enc_l": "#059669",
    "enc_r": "#059669",
    "trig": "#059669",
    "echo": "#059669",
    "ir_l": "#059669",
    "ir_r": "#059669",
    "sda": "#7c3aed",
    "scl": "#7c3aed",
    "start": "#0891b2",
    "buzzer": "#0891b2",
}


def _uno_pin_name(number: int) -> str:
    if number <= 13:
        return f"D{number}"
    if 14 <= number <= 19:
        return f"A{number - 14}"
    raise ValueError(f"pin {number} outside Uno range")


def _endpoints(w: Wire) -> Set[PinRef]:
    return {w.src, w.dst}


def _connected(wires: List[Wire], a: PinRef, b: PinRef) -> bool:
    return any(_endpoints(w) == {a, b} for w in wires)


def _normalize_target(ref: PinRef) -> str:
    if ref.component == "bb" and ref.pin.startswith("P5V_"):
        return "+5V"
    if ref.component == "bb" and ref.pin.startswith("GND_"):
        return "GND"
    if ref.component == "uno":
        return ref.pin
    raise ValueError(f"unexpected sensor partner {ref}")


def _parse_uno_config_pins() -> Dict[str, int]:
    with open(CONFIG_H, encoding="utf-8") as f:
        text = f.read()
    start = text.index("#if (BOARD_TYPE == BOARD_ARDUINO_UNO)")
    end = text.index("#elif", start)
    block = text[start:end]
    return {m.group(1): int(m.group(2)) for m in re.finditer(r"#define\s+(PIN_\w+)\s+(\d+)", block)}


def _doc_section(text: str, start_marker: str, end_marker: str) -> str:
    start = text.index(start_marker)
    return text[start : text.index(end_marker, start)]


def _parse_doc_stage2_table(text: str) -> Set[Tuple[str, str]]:
    section = _doc_section(text, "### Tahap 2", "### Tahap 3")
    rows = re.findall(r"^\|\s*\*\*`(D\d+)`\*\*\s*\|\s*\*\*`([A-Z0-9]+)`\*\*", section, re.MULTILINE)
    return set(rows)


def _parse_doc_pin_name(left: str) -> str:
    left = left.strip().lstrip("-").strip()
    if left.startswith("Kaki"):
        left = left[len("Kaki") :].strip()
    left = left.replace("`", "")
    return left.split()[0]


def _parse_doc_target(right: str) -> str:
    if "+5V" in right:
        return "+5V"
    if "Star Ground Hub" in right:
        return "GND"
    match = re.search(r"`([AD]\d+)`", right)
    if match is None:
        raise ValueError(f"unparseable target: {right!r}")
    return match.group(1)


def _parse_doc_stage3_to_5(text: str) -> Set[Tuple[str, str, str]]:
    section = _doc_section(text, "### Tahap 3", "## Checklist")
    result: Set[Tuple[str, str, str]] = set()
    current: Optional[str] = None
    for line in section.splitlines():
        if re.match(r"^\d+\.\s+\*\*", line):
            current = None
            for heading, component in DOC_HEADINGS:
                if heading in line:
                    current = component
                    break
            continue
        if current is None or r"$\rightarrow$" not in line:
            continue
        left, right = line.split(r"$\rightarrow$", 1)
        pin = _parse_doc_pin_name(left)
        if pin == "AD0":
            continue
        result.add((current, pin, _parse_doc_target(right)))
    return result


class TestBlenderNetlist(unittest.TestCase):
    def setUp(self) -> None:
        self.wires = build_netlist()

    def test_total_wire_count(self) -> None:
        self.assertEqual(len(self.wires), 41)

    def test_arduino_pins_match_config_h(self) -> None:
        config = _parse_uno_config_pins()
        for define, partner in CONFIG_PIN_PARTNERS.items():
            with self.subTest(define=define):
                self.assertIn(define, config)
                uno_pin = PinRef("uno", _uno_pin_name(config[define]))
                self.assertTrue(
                    _connected(self.wires, uno_pin, partner),
                    f"{define}={config[define]} expects wire {uno_pin} <-> {partner}",
                )

    def test_wiring_doc_matches_netlist(self) -> None:
        with open(WIRING_DOC, encoding="utf-8") as f:
            text = f.read()

        doc_stage2 = _parse_doc_stage2_table(text)
        self.assertEqual(len(doc_stage2), 6)
        net_stage2 = {
            (w.src.pin, w.dst.pin) for w in self.wires if w.src.component == "uno" and w.dst.component == "l298n"
        }
        self.assertEqual(doc_stage2, net_stage2)

        doc_sensors = _parse_doc_stage3_to_5(text)
        net_sensors = {
            (w.src.component, w.src.pin, _normalize_target(w.dst)) for w in self.wires if 3 <= w.stage <= 5
        }
        self.assertEqual(doc_sensors, net_sensors)

    def test_power_stage_wires(self) -> None:
        def key(ref: PinRef) -> Tuple[str, str]:
            if ref.component == "bb":
                return ("bb", ref.pin.split("_")[0] + "_*")
            return (ref.component, ref.pin)

        power = {(key(w.src), key(w.dst)) for w in self.wires if w.stage == 1}
        expected = {
            (("batt", "-"), ("bb", "GND_*")),
            (("batt", "+"), ("switch", "IN")),
            (("switch", "OUT"), ("uno", "VIN")),
            (("switch", "OUT"), ("l298n", "+12V")),
            (("uno", "GND"), ("bb", "GND_*")),
            (("l298n", "GND"), ("bb", "GND_*")),
            (("uno", "5V"), ("bb", "P5V_*")),
        }
        self.assertEqual(len([w for w in self.wires if w.stage == 1]), 7)
        self.assertEqual(power, expected)

    def test_motor_outputs(self) -> None:
        for motor, outs in (("motor_l", ("OUT1", "OUT2")), ("motor_r", ("OUT3", "OUT4"))):
            for terminal, out in zip(("T1", "T2"), outs):
                with self.subTest(motor=motor, terminal=terminal):
                    self.assertTrue(_connected(self.wires, PinRef(motor, terminal), PinRef("l298n", out)))

    def test_rails_only_take_matching_power_pins(self) -> None:
        for w in self.wires:
            for rail_end, other in ((w.src, w.dst), (w.dst, w.src)):
                if rail_end.component != "bb":
                    continue
                expected_kind = "power5v" if rail_end.pin.startswith("P5V_") else "gnd"
                with self.subTest(wire=wire_name(w)):
                    self.assertEqual(COMPONENTS[other.component][other.pin], expected_kind)

    def test_no_pin_used_twice(self) -> None:
        counts = Counter(ref for w in self.wires for ref in (w.src, w.dst))
        reused = {ref: n for ref, n in counts.items() if n > 1 and ref not in MULTI_USE_PINS}
        self.assertEqual(reused, {})

    def test_unconnected_pins(self) -> None:
        used = {ref for w in self.wires for ref in (w.src, w.dst)}
        self.assertNotIn(PinRef("l298n", "+5V"), used)
        self.assertNotIn(PinRef("mpu", "AD0"), used)

    def test_all_pinrefs_exist(self) -> None:
        for w in self.wires:
            for ref in (w.src, w.dst):
                with self.subTest(ref=ref):
                    self.assertIn(ref.component, COMPONENTS)
                    self.assertIn(ref.pin, COMPONENTS[ref.component])

    def test_wire_colors(self) -> None:
        for w in self.wires:
            with self.subTest(wire=wire_name(w)):
                self.assertIn(w.net, NET_COLORS)
                self.assertEqual(w.color, NET_COLORS[w.net])

    def test_wire_names_unique(self) -> None:
        names = [wire_name(w) for w in self.wires]
        self.assertEqual(len(names), len(set(names)))
        self.assertTrue(all(n.startswith("wire_") for n in names))

    def test_end_types(self) -> None:
        # btn: the 12 mm tactile switch has bent PCB legs, not header pins, so its wires are soldered.
        bare_components = {"batt", "switch", "motor_l", "motor_r", "btn"}
        bare_l298n = {"+12V", "GND", "OUT1", "OUT2", "OUT3", "OUT4"}
        for w in self.wires:
            for ref, end in ((w.src, w.src_end), (w.dst, w.dst_end)):
                expected = (
                    "bare"
                    if ref.component in bare_components or (ref.component == "l298n" and ref.pin in bare_l298n)
                    else "dupont"
                )
                with self.subTest(wire=wire_name(w), ref=ref):
                    self.assertEqual(end, expected)


if __name__ == "__main__":
    unittest.main()
