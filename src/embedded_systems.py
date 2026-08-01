"""Embedded-architecture and real-time scheduling utilities."""

from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class PeriodicTask:
    name: str
    execution_time: float
    period: float
    deadline: float | None = None
    priority: int | None = None

    @property
    def utilization(self) -> float:
        return self.execution_time / self.period

    @property
    def relative_deadline(self) -> float:
        return self.period if self.deadline is None else self.deadline


def total_utilization(tasks: list[PeriodicTask]) -> float:
    return float(sum(task.utilization for task in tasks))


def rate_monotonic_utilization_bound(task_count: int) -> float:
    if task_count <= 0:
        raise ValueError("task_count must be positive")
    return float(task_count * (2.0 ** (1.0 / task_count) - 1.0))


def response_time_analysis(tasks: list[PeriodicTask]) -> list[dict]:
    ordered = sorted(tasks, key=lambda task: task.period)
    results = []
    for index, task in enumerate(ordered):
        response = task.execution_time
        for _ in range(1000):
            interference = sum(
                math.ceil(response / hp.period) * hp.execution_time
                for hp in ordered[:index]
            )
            next_response = task.execution_time + interference
            if abs(next_response - response) < 1e-12:
                response = next_response
                break
            response = next_response
            if response > task.relative_deadline:
                break
        results.append({
            "task": task.name,
            "response_time": float(response),
            "deadline": float(task.relative_deadline),
            "schedulable": bool(response <= task.relative_deadline + 1e-12),
        })
    return results


def partition_pipeline(
    stages: list[dict],
    cpu_capacity: float,
    fpga_capacity: float,
    gpu_capacity: float,
) -> dict:
    capacities = {"CPU": cpu_capacity, "FPGA": fpga_capacity, "GPU": gpu_capacity}
    remaining = capacities.copy()
    allocation = []
    for stage in stages:
        candidates = []
        for resource in ["CPU", "FPGA", "GPU"]:
            demand = float(stage[f"{resource.lower()}_cost"])
            if demand <= remaining[resource]:
                candidates.append((demand, resource))
        if not candidates:
            allocation.append({"stage": stage["name"], "resource": "UNASSIGNED", "cost": np.nan})
            continue
        demand, resource = min(candidates)
        remaining[resource] -= demand
        allocation.append({"stage": stage["name"], "resource": resource, "cost": demand})
    return {"allocation": allocation, "remaining_capacity": remaining}


def dma_transfer_time(bytes_count: int, bandwidth_bytes_per_second: float, setup_seconds: float = 0.0) -> float:
    if bytes_count < 0 or bandwidth_bytes_per_second <= 0:
        raise ValueError("invalid DMA parameters")
    return float(setup_seconds + bytes_count / bandwidth_bytes_per_second)


def amdahl_speedup(parallel_fraction: float, processors: int) -> float:
    if not 0.0 <= parallel_fraction <= 1.0 or processors <= 0:
        raise ValueError("invalid Amdahl parameters")
    return float(1.0 / ((1.0 - parallel_fraction) + parallel_fraction / processors))


def quantized_memory_bytes(parameter_count: int, bits_per_parameter: int) -> int:
    if parameter_count < 0 or bits_per_parameter <= 0:
        raise ValueError("invalid quantization parameters")
    return int(math.ceil(parameter_count * bits_per_parameter / 8))
