"""Autonomous-vehicle metrics, planning, and ADAS utilities."""

from __future__ import annotations

import heapq
import numpy as np


SAE_LEVELS = {
    0: "No driving automation",
    1: "Driver assistance",
    2: "Partial driving automation",
    3: "Conditional driving automation",
    4: "High driving automation",
    5: "Full driving automation",
}


def stopping_distance(
    speed_mps: float,
    reaction_time_s: float,
    deceleration_mps2: float,
) -> float:
    if speed_mps < 0 or reaction_time_s < 0 or deceleration_mps2 <= 0:
        raise ValueError("invalid stopping-distance parameters")
    return float(
        speed_mps * reaction_time_s
        + speed_mps ** 2 / (2.0 * deceleration_mps2)
    )


def time_to_collision(
    relative_distance_m: float,
    closing_speed_mps: float,
) -> float:
    if relative_distance_m < 0:
        raise ValueError("distance must be nonnegative")
    if closing_speed_mps <= 0:
        return float("inf")
    return float(relative_distance_m / closing_speed_mps)


def emergency_braking_required(
    distance_m: float,
    ego_speed_mps: float,
    obstacle_speed_mps: float,
    threshold_s: float = 2.0,
) -> bool:
    closing = ego_speed_mps - obstacle_speed_mps
    return time_to_collision(distance_m, closing) <= threshold_s


def pure_pursuit_curvature(
    vehicle_position: np.ndarray,
    heading_rad: float,
    target_point: np.ndarray,
) -> float:
    position = np.asarray(vehicle_position, dtype=float)
    target = np.asarray(target_point, dtype=float)
    difference = target - position
    lookahead = np.linalg.norm(difference)
    if lookahead <= 1e-12:
        return 0.0
    local_y = -np.sin(heading_rad) * difference[0] + np.cos(heading_rad) * difference[1]
    return float(2.0 * local_y / (lookahead ** 2))


def dijkstra_road_graph(
    graph: dict[str, list[tuple[str, float]]],
    start: str,
    goal: str,
) -> dict:
    frontier = [(0.0, start)]
    distance = {node: float("inf") for node in graph}
    predecessor = {node: None for node in graph}
    distance[start] = 0.0

    while frontier:
        cost, node = heapq.heappop(frontier)
        if node == goal:
            break
        if cost > distance[node]:
            continue
        for neighbor, edge_cost in graph[node]:
            candidate = cost + edge_cost
            if candidate < distance[neighbor]:
                distance[neighbor] = candidate
                predecessor[neighbor] = node
                heapq.heappush(frontier, (candidate, neighbor))

    path = []
    if np.isfinite(distance[goal]):
        current = goal
        while current is not None:
            path.append(current)
            current = predecessor[current]
        path.reverse()
    return {"path": path, "cost": float(distance[goal])}


def occupancy_statistics(grid: np.ndarray) -> dict:
    grid = np.asarray(grid, dtype=float)
    return {
        "mean_occupancy": float(np.mean(grid)),
        "occupancy_variance": float(np.var(grid)),
        "occupied_fraction": float(np.mean(grid >= 0.65)),
        "unknown_fraction": float(np.mean((grid > 0.35) & (grid < 0.65))),
    }


def dataset_motion_statistics(
    positions: np.ndarray,
    timestamps: np.ndarray,
) -> dict:
    positions = np.asarray(positions, dtype=float)
    timestamps = np.asarray(timestamps, dtype=float)
    if len(positions) != len(timestamps) or len(positions) < 2:
        raise ValueError("positions and timestamps must match")
    dt = np.diff(timestamps)
    velocity = np.diff(positions, axis=0) / dt[:, None]
    acceleration = np.diff(velocity, axis=0) / dt[1:, None]
    return {
        "mean_speed": float(np.mean(np.linalg.norm(velocity, axis=1))),
        "maximum_speed": float(np.max(np.linalg.norm(velocity, axis=1))),
        "mean_acceleration": float(np.mean(np.linalg.norm(acceleration, axis=1))) if len(acceleration) else 0.0,
        "maximum_acceleration": float(np.max(np.linalg.norm(acceleration, axis=1))) if len(acceleration) else 0.0,
        "trajectory_extent": (np.max(positions, axis=0) - np.min(positions, axis=0)),
    }
