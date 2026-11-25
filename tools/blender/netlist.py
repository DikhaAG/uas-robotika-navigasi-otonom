#!/usr/bin/env python3
"""
Wiring netlist for the Blender 3D model (Arduino Uno R3 + L298N variant).

This is the single source of truth for the 41 wires in the model. It is
written out explicitly per assembly stage of docs/hardware/wiring-schematic.md
and cross-checked against that document and firmware Config.h by
tests/test_blender_netlist.py. No Blender dependency: importable anywhere.
"""

from dataclasses import dataclass
from typing import Dict, FrozenSet, List, Literal, Tuple

PinKind = Literal["power7v4", "power5v", "gnd", "signal", "motor"]
EndKind = Literal["dupont", "bare"]

RAIL_HOLES = 25


def _rail(prefix: str, kind: PinKind) -> Dict[str, PinKind]:
    return {f"{prefix}_{i}": kind for i in range(1, RAIL_HOLES + 1)}


COMPONENTS: Dict[str, Dict[str, PinKind]] = {
    "uno": {
        **{f"D{i}": "signal" for i in range(2, 14)},
        **{f"A{i}": "signal" for i in range(0, 6)},
        "VIN": "power7v4",
        "5V": "power5v",
        "GND": "gnd",
    },
    "l298n": {
        "+12V": "power7v4",
        "GND": "gnd",
        "+5V": "power5v",
        "ENA": "signal",
        "IN1": "signal",
        "IN2": "signal",
        "IN3": "signal",
        "IN4": "signal",
        "ENB": "signal",
        "OUT1": "motor",
        "OUT2": "motor",
        "OUT3": "motor",
        "OUT4": "motor",
    },
    "batt": {"+": "power7v4", "-": "gnd"},
    "switch": {"IN": "power7v4", "OUT": "power7v4"},
    "motor_l": {"T1": "motor", "T2": "motor"},
    "motor_r": {"T1": "motor", "T2": "motor"},
    "bb": {**_rail("P5V", "power5v"), **_rail("GND", "gnd")},
    "enc_l": {"VCC": "power5v", "GND": "gnd", "D0": "signal", "A0": "signal"},
    "enc_r": {"VCC": "power5v", "GND": "gnd", "D0": "signal", "A0": "signal"},
    "sonar": {"VCC": "power5v", "Trig": "signal", "Echo": "signal", "GND": "gnd"},
    "mpu": {
        "VCC": "power5v",
        "GND": "gnd",
        "SCL": "signal",
        "SDA": "signal",
        "XDA": "signal",
        "XCL": "signal",
        "AD0": "signal",
        "INT": "signal",
    },
    "ir_l": {"OUT": "signal", "GND": "gnd", "VCC": "power5v"},
    "ir_r": {"OUT": "signal", "GND": "gnd", "VCC": "power5v"},
    "btn": {"1": "signal", "2": "gnd"},
    "buzzer": {"+": "signal", "-": "gnd"},
}

NET_COLORS: Dict[str, str] = {
    "vbat": "#dc2626",
    "v5": "#ea580c",
    "gnd": "#1e293b",
    "motor_signal": "#2563eb",
    "motor_out": "#b45309",
    "sensor": "#059669",
    "i2c": "#7c3aed",
    "interface": "#0891b2",
}

# btn: the 12 mm tactile switch has bent PCB legs (no header), so its wires are soldered on.
_BARE_COMPONENTS: FrozenSet[str] = frozenset({"batt", "switch", "motor_l", "motor_r", "btn"})
_BARE_L298N_PINS: FrozenSet[str] = frozenset({"+12V", "GND", "OUT1", "OUT2", "OUT3", "OUT4"})


@dataclass(frozen=True)
class PinRef:
    """One physical pin: component id (key of COMPONENTS) and pin name."""

    component: str
    pin: str


@dataclass(frozen=True)
class Wire:
    """One jumper/lead between two pins, with its net, colour and assembly stage (1..5)."""

    net: str
    src: PinRef
    dst: PinRef
    color: str
    stage: int
    src_end: EndKind
    dst_end: EndKind


MULTI_USE_PINS: FrozenSet[PinRef] = frozenset({PinRef("switch", "OUT")})


def end_kind(ref: PinRef) -> EndKind:
    """Screw terminals, battery leads, switch, button legs and motor tabs take bare ends; headers take dupont."""
    if ref.component in _BARE_COMPONENTS:
        return "bare"
    if ref.component == "l298n" and ref.pin in _BARE_L298N_PINS:
        return "bare"
    return "dupont"


def _color_for(net: str) -> str:
    if net in ("vbat", "v5", "gnd"):
        return NET_COLORS[net]
    if net in ("ena", "enb") or net.startswith("in"):
        return NET_COLORS["motor_signal"]
    if net.startswith("out"):
        return NET_COLORS["motor_out"]
    if net in ("sda", "scl"):
        return NET_COLORS["i2c"]
    if net in ("start", "buzzer"):
        return NET_COLORS["interface"]
    return NET_COLORS["sensor"]


class _RailAllocator:
    """Hands out breadboard rail holes in order, so each rail wire gets its own hole.
    +5V uses the odd holes and GND the even ones: the two rails sit 2.54 mm apart, so no used
    hole faces another across the rails and a wire dropping onto one never passes over the other."""

    def __init__(self) -> None:
        self._next: Dict[str, int] = {"P5V": 1, "GND": 2}

    def take(self, rail: str) -> PinRef:
        index = self._next[rail]
        if index > RAIL_HOLES:
            raise ValueError(f"breadboard rail {rail} has no free hole left")
        self._next[rail] = index + 2
        return PinRef("bb", f"{rail}_{index}")


def build_netlist() -> List[Wire]:
    """Returns all 41 wires, grouped by assembly stage of wiring-schematic.md."""
    rails = _RailAllocator()
    plus5 = lambda: rails.take("P5V")  # noqa: E731
    gnd = lambda: rails.take("GND")  # noqa: E731

    spec: List[Tuple[int, str, PinRef, PinRef]] = [
        # Tahap 1: daya & star ground
        (1, "gnd", PinRef("batt", "-"), gnd()),
        (1, "vbat", PinRef("batt", "+"), PinRef("switch", "IN")),
        (1, "vbat", PinRef("switch", "OUT"), PinRef("uno", "VIN")),
        (1, "vbat", PinRef("switch", "OUT"), PinRef("l298n", "+12V")),
        (1, "gnd", PinRef("uno", "GND"), gnd()),
        (1, "gnd", PinRef("l298n", "GND"), gnd()),
        (1, "v5", PinRef("uno", "5V"), plus5()),
        # Tahap 2: sinyal kontrol L298N
        (2, "ena", PinRef("uno", "D9"), PinRef("l298n", "ENA")),
        (2, "in1", PinRef("uno", "D8"), PinRef("l298n", "IN1")),
        (2, "in2", PinRef("uno", "D7"), PinRef("l298n", "IN2")),
        (2, "enb", PinRef("uno", "D6"), PinRef("l298n", "ENB")),
        (2, "in3", PinRef("uno", "D5"), PinRef("l298n", "IN3")),
        (2, "in4", PinRef("uno", "D4"), PinRef("l298n", "IN4")),
        # Tahap 2: keluaran motor
        (2, "out1", PinRef("motor_l", "T1"), PinRef("l298n", "OUT1")),
        (2, "out2", PinRef("motor_l", "T2"), PinRef("l298n", "OUT2")),
        (2, "out3", PinRef("motor_r", "T1"), PinRef("l298n", "OUT3")),
        (2, "out4", PinRef("motor_r", "T2"), PinRef("l298n", "OUT4")),
        # Tahap 3: encoder
        (3, "v5", PinRef("enc_l", "VCC"), plus5()),
        (3, "gnd", PinRef("enc_l", "GND"), gnd()),
        (3, "enc_l", PinRef("enc_l", "D0"), PinRef("uno", "D2")),
        (3, "v5", PinRef("enc_r", "VCC"), plus5()),
        (3, "gnd", PinRef("enc_r", "GND"), gnd()),
        (3, "enc_r", PinRef("enc_r", "D0"), PinRef("uno", "D3")),
        # Tahap 4: HC-SR04
        (4, "v5", PinRef("sonar", "VCC"), plus5()),
        (4, "trig", PinRef("sonar", "Trig"), PinRef("uno", "A0")),
        (4, "echo", PinRef("sonar", "Echo"), PinRef("uno", "A1")),
        (4, "gnd", PinRef("sonar", "GND"), gnd()),
        # Tahap 4: MPU6050 (AD0 tidak disambung)
        (4, "v5", PinRef("mpu", "VCC"), plus5()),
        (4, "gnd", PinRef("mpu", "GND"), gnd()),
        (4, "sda", PinRef("mpu", "SDA"), PinRef("uno", "A4")),
        (4, "scl", PinRef("mpu", "SCL"), PinRef("uno", "A5")),
        # Tahap 4: FC-51 kiri & kanan
        (4, "v5", PinRef("ir_l", "VCC"), plus5()),
        (4, "ir_l", PinRef("ir_l", "OUT"), PinRef("uno", "A3")),
        (4, "gnd", PinRef("ir_l", "GND"), gnd()),
        (4, "v5", PinRef("ir_r", "VCC"), plus5()),
        (4, "ir_r", PinRef("ir_r", "OUT"), PinRef("uno", "D11")),
        (4, "gnd", PinRef("ir_r", "GND"), gnd()),
        # Tahap 5: tombol START & buzzer
        (5, "start", PinRef("btn", "1"), PinRef("uno", "D12")),
        (5, "gnd", PinRef("btn", "2"), gnd()),
        (5, "buzzer", PinRef("buzzer", "+"), PinRef("uno", "A2")),
        (5, "gnd", PinRef("buzzer", "-"), gnd()),
    ]

    return [
        Wire(
            net=net,
            src=src,
            dst=dst,
            color=_color_for(net),
            stage=stage,
            src_end=end_kind(src),
            dst_end=end_kind(dst),
        )
        for stage, net, src, dst in spec
    ]


def wire_name(w: Wire) -> str:
    """Blender object name for a wire: wire_<net>_<src.component>.<src.pin>_<dst.component>.<dst.pin>."""
    return f"wire_{w.net}_{w.src.component}.{w.src.pin}_{w.dst.component}.{w.dst.pin}"
