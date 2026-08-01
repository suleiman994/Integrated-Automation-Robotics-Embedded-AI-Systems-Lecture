"""Robotics kinematics, trajectories, mapping, and planning utilities."""

from __future__ import annotations

import heapq
import numpy as np


def dh_transform(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0.0, sa, ca, d],
        [0.0, 0.0, 0.0, 1.0],
    ])


def forward_kinematics(dh_rows: list[tuple[float, float, float, float]]) -> np.ndarray:
    transform = np.eye(4)
    for row in dh_rows:
        transform = transform @ dh_transform(*row)
    return transform


def planar_two_link_fk(q: np.ndarray, lengths: tuple[float, float]) -> np.ndarray:
    q1, q2 = np.asarray(q, dtype=float)
    l1, l2 = lengths
    return np.array([
        l1 * np.cos(q1) + l2 * np.cos(q1 + q2),
        l1 * np.sin(q1) + l2 * np.sin(q1 + q2),
    ])


def planar_two_link_jacobian(q: np.ndarray, lengths: tuple[float, float]) -> np.ndarray:
    q1, q2 = np.asarray(q, dtype=float)
    l1, l2 = lengths
    return np.array([
        [-l1 * np.sin(q1) - l2 * np.sin(q1 + q2), -l2 * np.sin(q1 + q2)],
        [l1 * np.cos(q1) + l2 * np.cos(q1 + q2), l2 * np.cos(q1 + q2)],
    ])


def inverse_kinematics_two_link(
    target: np.ndarray,
    lengths: tuple[float, float],
    elbow: str = "up",
) -> np.ndarray:
    x, y = np.asarray(target, dtype=float)
    l1, l2 = lengths
    cosine_q2 = (x * x + y * y - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
    if abs(cosine_q2) > 1.0 + 1e-12:
        raise ValueError("target is unreachable")
    cosine_q2 = np.clip(cosine_q2, -1.0, 1.0)
    sine_q2 = np.sqrt(max(0.0, 1.0 - cosine_q2 ** 2))
    if elbow.lower() == "down":
        sine_q2 = -sine_q2
    q2 = np.arctan2(sine_q2, cosine_q2)
    q1 = np.arctan2(y, x) - np.arctan2(l2 * sine_q2, l1 + l2 * cosine_q2)
    return np.array([q1, q2])


def cubic_joint_trajectory(
    q0: np.ndarray,
    qf: np.ndarray,
    duration: float,
    times: np.ndarray,
) -> dict:
    q0 = np.asarray(q0, dtype=float)
    qf = np.asarray(qf, dtype=float)
    times = np.asarray(times, dtype=float)
    if duration <= 0:
        raise ValueError("duration must be positive")
    tau = np.clip(times / duration, 0.0, 1.0)
    blend = 3 * tau ** 2 - 2 * tau ** 3
    blend_rate = (6 * tau - 6 * tau ** 2) / duration
    position = q0[None, :] + blend[:, None] * (qf - q0)[None, :]
    velocity = blend_rate[:, None] * (qf - q0)[None, :]
    return {"position": position, "velocity": velocity}


def astar_grid(occupancy: np.ndarray, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]]:
    grid = np.asarray(occupancy, dtype=bool)
    rows, columns = grid.shape
    if grid[start] or grid[goal]:
        raise ValueError("start and goal must be free")

    def heuristic(node):
        return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

    frontier = [(heuristic(start), 0, start)]
    came_from = {start: None}
    cost_so_far = {start: 0}

    while frontier:
        _, cost, current = heapq.heappop(frontier)
        if current == goal:
            break
        for dr, dc in [(1,0),(-1,0),(0,1),(0,-1)]:
            neighbor = (current[0] + dr, current[1] + dc)
            if not (0 <= neighbor[0] < rows and 0 <= neighbor[1] < columns):
                continue
            if grid[neighbor]:
                continue
            new_cost = cost + 1
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                came_from[neighbor] = current
                heapq.heappush(frontier, (new_cost + heuristic(neighbor), new_cost, neighbor))

    if goal not in came_from:
        return []
    path = []
    current = goal
    while current is not None:
        path.append(current)
        current = came_from[current]
    return list(reversed(path))


def log_odds_update(prior_log_odds: np.ndarray, occupied_cells: list[tuple[int, int]], free_cells: list[tuple[int, int]], occupied_increment: float = 0.85, free_increment: float = -0.4) -> np.ndarray:
    updated = np.asarray(prior_log_odds, dtype=float).copy()
    for cell in occupied_cells:
        updated[cell] += occupied_increment
    for cell in free_cells:
        updated[cell] += free_increment
    return updated


def logistic_probability(log_odds: np.ndarray) -> np.ndarray:
    log_odds = np.asarray(log_odds, dtype=float)
    return 1.0 / (1.0 + np.exp(-log_odds))
