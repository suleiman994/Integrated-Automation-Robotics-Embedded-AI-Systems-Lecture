"""Numerical methods, optimization, dynamic programming, and MCDA."""

from __future__ import annotations

import numpy as np


def bisection(function, lower: float, upper: float, tolerance: float = 1e-10, max_iterations: int = 1000) -> dict:
    f_lower = function(lower)
    f_upper = function(upper)
    if f_lower * f_upper > 0:
        raise ValueError("root is not bracketed")
    for iteration in range(max_iterations):
        midpoint = 0.5 * (lower + upper)
        f_midpoint = function(midpoint)
        if abs(f_midpoint) <= tolerance or 0.5 * (upper - lower) <= tolerance:
            return {"root": float(midpoint), "iterations": iteration + 1}
        if f_lower * f_midpoint <= 0:
            upper = midpoint
            f_upper = f_midpoint
        else:
            lower = midpoint
            f_lower = f_midpoint
    return {"root": float(0.5 * (lower + upper)), "iterations": max_iterations}


def central_difference(function, point: float, step: float = 1e-5) -> float:
    return float((function(point + step) - function(point - step)) / (2.0 * step))


def simpson_integral(function, lower: float, upper: float, intervals: int = 100) -> float:
    if intervals <= 0 or intervals % 2 != 0:
        raise ValueError("intervals must be a positive even integer")
    x = np.linspace(lower, upper, intervals + 1)
    y = np.asarray([function(value) for value in x], dtype=float)
    h = (upper - lower) / intervals
    return float(
        h / 3.0
        * (
            y[0]
            + y[-1]
            + 4.0 * np.sum(y[1:-1:2])
            + 2.0 * np.sum(y[2:-2:2])
        )
    )


def gradient_descent(
    function,
    gradient,
    initial: np.ndarray,
    step_size: float = 0.05,
    iterations: int = 500,
) -> dict:
    x = np.asarray(initial, dtype=float).copy()
    trace = []
    for iteration in range(iterations):
        value = float(function(x))
        grad = np.asarray(gradient(x), dtype=float)
        trace.append({"iteration": iteration, "objective": value, "x": x.copy()})
        if np.linalg.norm(grad) <= 1e-9:
            break
        candidate_step = step_size
        while function(x - candidate_step * grad) > value - 1e-4 * candidate_step * np.dot(grad, grad):
            candidate_step *= 0.5
            if candidate_step < 1e-14:
                break
        x = x - candidate_step * grad
    return {"x": x, "objective": float(function(x)), "trace": trace}


def projected_gradient(
    function,
    gradient,
    initial: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    step_size: float = 0.05,
    iterations: int = 500,
) -> dict:
    x = np.clip(np.asarray(initial, dtype=float), lower, upper)
    for _ in range(iterations):
        candidate = np.clip(x - step_size * np.asarray(gradient(x), dtype=float), lower, upper)
        if np.linalg.norm(candidate - x) <= 1e-9:
            break
        x = candidate
    return {"x": x, "objective": float(function(x))}


def knapsack_dynamic_programming(values: np.ndarray, weights: np.ndarray, capacity: int) -> dict:
    values = np.asarray(values, dtype=float)
    weights = np.asarray(weights, dtype=int)
    table = np.zeros((len(values) + 1, capacity + 1), dtype=float)
    for item in range(1, len(values) + 1):
        for current_capacity in range(capacity + 1):
            table[item, current_capacity] = table[item - 1, current_capacity]
            if weights[item - 1] <= current_capacity:
                table[item, current_capacity] = max(
                    table[item, current_capacity],
                    table[item - 1, current_capacity - weights[item - 1]] + values[item - 1],
                )
    selected = np.zeros(len(values), dtype=int)
    remaining = capacity
    for item in range(len(values), 0, -1):
        if table[item, remaining] > table[item - 1, remaining]:
            selected[item - 1] = 1
            remaining -= weights[item - 1]
    return {
        "selected": selected,
        "value": float(values @ selected),
        "weight": int(weights @ selected),
    }


def weighted_sum_mcda(
    decision_matrix: np.ndarray,
    weights: np.ndarray,
    benefit_criteria: np.ndarray,
) -> dict:
    matrix = np.asarray(decision_matrix, dtype=float)
    weights = np.asarray(weights, dtype=float)
    benefit = np.asarray(benefit_criteria, dtype=bool)
    if matrix.shape[1] != len(weights) or len(weights) != len(benefit):
        raise ValueError("criteria dimensions do not match")
    weights = weights / np.sum(weights)
    normalized = np.zeros_like(matrix)
    for column in range(matrix.shape[1]):
        values = matrix[:, column]
        minimum = np.min(values)
        maximum = np.max(values)
        if maximum <= minimum:
            normalized[:, column] = 1.0
        elif benefit[column]:
            normalized[:, column] = (values - minimum) / (maximum - minimum)
        else:
            normalized[:, column] = (maximum - values) / (maximum - minimum)
    scores = normalized @ weights
    ranking = np.argsort(-scores)
    return {"normalized": normalized, "scores": scores, "ranking": ranking}


def preference_substitution_rate(
    weight_first: float,
    weight_second: float,
) -> float:
    if weight_second == 0:
        return float("inf")
    return float(weight_first / weight_second)


def pareto_nondominated(points: np.ndarray, minimize: np.ndarray) -> np.ndarray:
    values = np.asarray(points, dtype=float)
    minimize = np.asarray(minimize, dtype=bool)
    transformed = values.copy()
    transformed[:, ~minimize] *= -1.0
    nondominated = np.ones(len(values), dtype=bool)
    for index in range(len(values)):
        for other in range(len(values)):
            if index == other:
                continue
            if np.all(transformed[other] <= transformed[index]) and np.any(transformed[other] < transformed[index]):
                nondominated[index] = False
                break
    return nondominated
