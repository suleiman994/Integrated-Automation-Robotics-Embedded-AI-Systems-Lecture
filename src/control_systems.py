"""Compact control-systems algorithms used by the integrated lecture."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


Array = np.ndarray


def controllability_matrix(A: Array, B: Array) -> Array:
    A = np.asarray(A, dtype=float)
    B = np.atleast_2d(np.asarray(B, dtype=float))
    if B.shape[0] != A.shape[0]:
        B = B.T
    blocks = [B]
    for power in range(1, A.shape[0]):
        blocks.append(np.linalg.matrix_power(A, power) @ B)
    return np.hstack(blocks)


def observability_matrix(A: Array, C: Array) -> Array:
    A = np.asarray(A, dtype=float)
    C = np.atleast_2d(np.asarray(C, dtype=float))
    blocks = [C]
    for power in range(1, A.shape[0]):
        blocks.append(C @ np.linalg.matrix_power(A, power))
    return np.vstack(blocks)


def is_controllable(A: Array, B: Array, tolerance: float = 1e-10) -> bool:
    matrix = controllability_matrix(A, B)
    return np.linalg.matrix_rank(matrix, tol=tolerance) == A.shape[0]


def is_observable(A: Array, C: Array, tolerance: float = 1e-10) -> bool:
    matrix = observability_matrix(A, C)
    return np.linalg.matrix_rank(matrix, tol=tolerance) == A.shape[0]


def simulate_discrete_linear(
    A: Array,
    B: Array,
    x0: Array,
    inputs: Array,
) -> Array:
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    x = np.asarray(x0, dtype=float).copy()
    inputs = np.atleast_2d(np.asarray(inputs, dtype=float))
    if inputs.shape[0] == 1 and inputs.shape[1] > 1:
        inputs = inputs.T
    states = [x.copy()]
    for control in inputs:
        x = A @ x + B.reshape(A.shape[0], -1) @ np.atleast_1d(control)
        states.append(x.copy())
    return np.asarray(states)


def simulate_nonlinear_rk4(
    dynamics,
    x0: Array,
    times: Array,
    input_function=None,
) -> Array:
    times = np.asarray(times, dtype=float)
    x = np.asarray(x0, dtype=float).copy()
    states = [x.copy()]
    for index in range(len(times) - 1):
        t = times[index]
        dt = times[index + 1] - times[index]
        u1 = 0.0 if input_function is None else input_function(t, x)
        k1 = np.asarray(dynamics(t, x, u1), dtype=float)
        u2 = 0.0 if input_function is None else input_function(t + dt / 2, x + dt * k1 / 2)
        k2 = np.asarray(dynamics(t + dt / 2, x + dt * k1 / 2, u2), dtype=float)
        u3 = 0.0 if input_function is None else input_function(t + dt / 2, x + dt * k2 / 2)
        k3 = np.asarray(dynamics(t + dt / 2, x + dt * k2 / 2, u3), dtype=float)
        u4 = 0.0 if input_function is None else input_function(t + dt, x + dt * k3)
        k4 = np.asarray(dynamics(t + dt, x + dt * k3, u4), dtype=float)
        x = x + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6.0
        states.append(x.copy())
    return np.asarray(states)


@dataclass
class PID:
    kp: float
    ki: float
    kd: float
    dt: float
    lower: float = -np.inf
    upper: float = np.inf
    integral: float = 0.0
    previous_error: float = 0.0
    initialized: bool = False

    def update(self, setpoint: float, measurement: float) -> float:
        error = setpoint - measurement
        derivative = 0.0 if not self.initialized else (error - self.previous_error) / self.dt
        candidate_integral = self.integral + error * self.dt
        raw = self.kp * error + self.ki * candidate_integral + self.kd * derivative
        output = float(np.clip(raw, self.lower, self.upper))
        if (
            self.lower < raw < self.upper
            or (raw >= self.upper and error < 0.0)
            or (raw <= self.lower and error > 0.0)
        ):
            self.integral = candidate_integral
        self.previous_error = error
        self.initialized = True
        return output


def ultimate_gain_pid_tuning(ultimate_gain: float, ultimate_period: float, mode: str = "PID") -> dict:
    mode = mode.upper()
    if ultimate_gain <= 0 or ultimate_period <= 0:
        raise ValueError("ultimate gain and period must be positive")
    if mode == "P":
        return {"kp": 0.5 * ultimate_gain, "ki": 0.0, "kd": 0.0}
    if mode == "PI":
        kp = 0.45 * ultimate_gain
        ti = ultimate_period / 1.2
        return {"kp": kp, "ki": kp / ti, "kd": 0.0}
    if mode == "PID":
        kp = 0.6 * ultimate_gain
        ti = ultimate_period / 2.0
        td = ultimate_period / 8.0
        return {"kp": kp, "ki": kp / ti, "kd": kp * td}
    raise ValueError("mode must be P, PI, or PID")


def solve_discrete_are(
    A: Array,
    B: Array,
    Q: Array,
    R: Array,
    tolerance: float = 1e-10,
    max_iterations: int = 10000,
) -> Array:
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    Q = np.asarray(Q, dtype=float)
    R = np.atleast_2d(np.asarray(R, dtype=float))
    P = Q.copy()
    for _ in range(max_iterations):
        gain_term = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
        next_P = A.T @ P @ A - A.T @ P @ B @ gain_term + Q
        if np.linalg.norm(next_P - P) <= tolerance:
            return next_P
        P = next_P
    raise RuntimeError("Riccati iteration did not converge")


def dlqr(A: Array, B: Array, Q: Array, R: Array) -> tuple[Array, Array]:
    P = solve_discrete_are(A, B, Q, R)
    K = np.linalg.solve(
        np.asarray(R, dtype=float) + np.asarray(B).T @ P @ np.asarray(B),
        np.asarray(B).T @ P @ np.asarray(A),
    )
    return K, P


def least_squares_identification(regressor: Array, targets: Array) -> dict:
    X = np.asarray(regressor, dtype=float)
    y = np.asarray(targets, dtype=float)
    parameters, residuals, rank, singular_values = np.linalg.lstsq(X, y, rcond=None)
    prediction = X @ parameters
    error = y - prediction
    return {
        "parameters": parameters,
        "prediction": prediction,
        "rmse": float(np.sqrt(np.mean(error ** 2))),
        "rank": int(rank),
        "singular_values": singular_values,
    }


def arx_design_matrix(inputs: Array, outputs: Array) -> tuple[Array, Array]:
    u = np.asarray(inputs, dtype=float)
    y = np.asarray(outputs, dtype=float)
    if len(u) != len(y) or len(y) < 3:
        raise ValueError("input and output lengths must match and exceed two")
    X = np.column_stack([y[1:-1], u[1:-1]])
    target = y[2:]
    return X, target
