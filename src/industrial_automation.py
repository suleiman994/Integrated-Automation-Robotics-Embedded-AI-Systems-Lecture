"""Industrial-automation structures, event control, and IIoT utilities."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from datetime import datetime, timezone
import json
import numpy as np


AUTOMATION_LEVELS = [
    "Physical process",
    "Field devices",
    "PLC/DCS control",
    "HMI/SCADA supervision",
    "MES production management",
    "ERP enterprise planning",
]


class MachineState(Enum):
    IDLE = auto()
    RUNNING = auto()
    PROCESSING = auto()
    FAULT = auto()


@dataclass
class EventControlledMachine:
    state: MachineState = MachineState.IDLE
    produced_count: int = 0
    events: list[dict] = field(default_factory=list)

    def update(
        self,
        time_s: float,
        start: bool = False,
        stop: bool = False,
        product_detected: bool = False,
        process_complete: bool = False,
        fault: bool = False,
        reset: bool = False,
    ) -> MachineState:
        previous = self.state
        cause = None
        if fault:
            self.state = MachineState.FAULT
            cause = "FAULT"
        elif self.state == MachineState.FAULT:
            if reset:
                self.state = MachineState.IDLE
                cause = "RESET"
        elif stop:
            self.state = MachineState.IDLE
            cause = "STOP"
        elif self.state == MachineState.IDLE and start:
            self.state = MachineState.RUNNING
            cause = "START"
        elif self.state == MachineState.RUNNING and product_detected:
            self.state = MachineState.PROCESSING
            cause = "PRODUCT_DETECTED"
        elif self.state == MachineState.PROCESSING and process_complete:
            self.produced_count += 1
            self.state = MachineState.RUNNING
            cause = "PROCESS_COMPLETE"

        if self.state != previous:
            self.events.append({
                "time_s": float(time_s),
                "from_state": previous.name,
                "to_state": self.state.name,
                "cause": cause,
            })
        return self.state


def scale_4_20_ma(
    current_ma: float,
    engineering_minimum: float,
    engineering_maximum: float,
) -> float:
    fraction = (current_ma - 4.0) / 16.0
    return float(
        engineering_minimum
        + fraction * (engineering_maximum - engineering_minimum)
    )


def dcs_parallel_availability(single_channel_availability: float, channel_count: int = 2) -> float:
    if not 0 <= single_channel_availability <= 1 or channel_count <= 0:
        raise ValueError("invalid availability arguments")
    return float(1.0 - (1.0 - single_channel_availability) ** channel_count)


def overall_equipment_effectiveness(
    availability: float,
    performance: float,
    quality: float,
) -> float:
    values = np.array([availability, performance, quality], dtype=float)
    if np.any((values < 0.0) | (values > 1.0)):
        raise ValueError("OEE factors must lie in [0,1]")
    return float(np.prod(values))


def iiot_payload(
    asset_id: str,
    measurements: dict,
    quality: str = "GOOD",
) -> str:
    payload = {
        "asset_id": asset_id,
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "quality": quality,
        "measurements": measurements,
    }
    return json.dumps(payload, separators=(",", ":"))
