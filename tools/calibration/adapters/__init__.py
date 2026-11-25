"""
Adapters package for transport, observation, and reporting seams.
"""

from tools.calibration.adapters.transport import (
    TransportAdapter,
    SerialTransportAdapter,
    MockRobotSerial,
    autodetect_serial_port,
)
from tools.calibration.adapters.observation import (
    ObservationAdapter,
    ConsoleObservationAdapter,
    ScriptedObservationAdapter,
)
from tools.calibration.adapters.reporter import (
    ReporterAdapter,
    AnsiConsoleReporter,
    NullReporter,
    CalibrationLogger,
    CompositeReporter,
    Colors,
)

__all__ = [
    "TransportAdapter",
    "SerialTransportAdapter",
    "MockRobotSerial",
    "autodetect_serial_port",
    "ObservationAdapter",
    "ConsoleObservationAdapter",
    "ScriptedObservationAdapter",
    "ReporterAdapter",
    "AnsiConsoleReporter",
    "NullReporter",
    "CalibrationLogger",
    "CompositeReporter",
    "Colors",
]
