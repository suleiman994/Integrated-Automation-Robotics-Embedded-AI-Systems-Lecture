"""Perception and multi-domain sensor-fusion utilities."""

from __future__ import annotations

import numpy as np


def normalize_image(image: np.ndarray) -> np.ndarray:
    image = np.asarray(image, dtype=float)
    minimum = np.min(image)
    maximum = np.max(image)
    if maximum <= minimum:
        return np.zeros_like(image)
    return (image - minimum) / (maximum - minimum)


def convolve2d(image: np.ndarray, kernel: np.ndarray) -> np.ndarray:
    image = np.asarray(image, dtype=float)
    kernel = np.asarray(kernel, dtype=float)
    if image.ndim != 2 or kernel.ndim != 2:
        raise ValueError("image and kernel must be two-dimensional")
    pad_r = kernel.shape[0] // 2
    pad_c = kernel.shape[1] // 2
    padded = np.pad(image, ((pad_r, pad_r), (pad_c, pad_c)), mode="edge")
    output = np.zeros_like(image, dtype=float)
    flipped = np.flip(kernel)
    for row in range(image.shape[0]):
        for column in range(image.shape[1]):
            region = padded[row:row + kernel.shape[0], column:column + kernel.shape[1]]
            output[row, column] = np.sum(region * flipped)
    return output


def gaussian_blur(image: np.ndarray) -> np.ndarray:
    kernel = np.array([
        [1, 2, 1],
        [2, 4, 2],
        [1, 2, 1],
    ], dtype=float) / 16.0
    return convolve2d(image, kernel)


def foreground_mask(
    frame: np.ndarray,
    background: np.ndarray,
    threshold: float,
) -> np.ndarray:
    return np.abs(np.asarray(frame, dtype=float) - np.asarray(background, dtype=float)) >= threshold


def horn_schunck_optical_flow(
    first: np.ndarray,
    second: np.ndarray,
    smoothness: float = 1.0,
    iterations: int = 80,
) -> tuple[np.ndarray, np.ndarray]:
    first = np.asarray(first, dtype=float)
    second = np.asarray(second, dtype=float)
    if first.shape != second.shape or first.ndim != 2:
        raise ValueError("frames must be matching grayscale arrays")

    average_kernel = np.array([
        [1/12, 1/6, 1/12],
        [1/6, 0.0, 1/6],
        [1/12, 1/6, 1/12],
    ])
    derivative_x = np.array([[-1, 1], [-1, 1]], dtype=float) / 4.0
    derivative_y = np.array([[-1, -1], [1, 1]], dtype=float) / 4.0
    derivative_t = np.ones((2, 2), dtype=float) / 4.0

    ix = convolve2d(first, derivative_x) + convolve2d(second, derivative_x)
    iy = convolve2d(first, derivative_y) + convolve2d(second, derivative_y)
    it = convolve2d(second, derivative_t) - convolve2d(first, derivative_t)

    u = np.zeros_like(first)
    v = np.zeros_like(first)
    for _ in range(iterations):
        u_bar = convolve2d(u, average_kernel)
        v_bar = convolve2d(v, average_kernel)
        denominator = smoothness ** 2 + ix ** 2 + iy ** 2
        correction = (ix * u_bar + iy * v_bar + it) / np.maximum(denominator, 1e-12)
        u = u_bar - ix * correction
        v = v_bar - iy * correction
    return u, v


def stereo_depth(
    disparity_pixels: np.ndarray,
    focal_length_pixels: float,
    baseline_meters: float,
    minimum_disparity: float = 1e-6,
) -> np.ndarray:
    disparity = np.asarray(disparity_pixels, dtype=float)
    return focal_length_pixels * baseline_meters / np.maximum(disparity, minimum_disparity)


def intersection_over_union(first_mask: np.ndarray, second_mask: np.ndarray) -> float:
    first = np.asarray(first_mask, dtype=bool)
    second = np.asarray(second_mask, dtype=bool)
    intersection = np.sum(first & second)
    union = np.sum(first | second)
    return 1.0 if union == 0 else float(intersection / union)


def precision_recall_f1(predicted: np.ndarray, truth: np.ndarray) -> dict:
    predicted = np.asarray(predicted, dtype=bool)
    truth = np.asarray(truth, dtype=bool)
    tp = np.sum(predicted & truth)
    fp = np.sum(predicted & ~truth)
    fn = np.sum(~predicted & truth)
    precision = tp / max(tp + fp, 1)
    recall = tp / max(tp + fn, 1)
    f1 = 2 * precision * recall / max(precision + recall, 1e-12)
    return {
        "precision": float(precision),
        "recall": float(recall),
        "f1": float(f1),
    }


def linear_kalman_fusion(
    prior_state: np.ndarray,
    prior_covariance: np.ndarray,
    measurements: list[np.ndarray],
    measurement_matrices: list[np.ndarray],
    measurement_covariances: list[np.ndarray],
) -> dict:
    state = np.asarray(prior_state, dtype=float).copy()
    covariance = np.asarray(prior_covariance, dtype=float).copy()
    for measurement, H, R in zip(measurements, measurement_matrices, measurement_covariances):
        measurement = np.asarray(measurement, dtype=float)
        H = np.asarray(H, dtype=float)
        R = np.asarray(R, dtype=float)
        innovation = measurement - H @ state
        innovation_covariance = H @ covariance @ H.T + R
        gain = covariance @ H.T @ np.linalg.inv(innovation_covariance)
        state = state + gain @ innovation
        covariance = (np.eye(len(state)) - gain @ H) @ covariance
    return {"state": state, "covariance": covariance}


def radar_range_resolution(bandwidth_hz: float, speed_of_light: float = 299_792_458.0) -> float:
    if bandwidth_hz <= 0:
        raise ValueError("bandwidth must be positive")
    return float(speed_of_light / (2.0 * bandwidth_hz))


def lidar_cartesian(ranges: np.ndarray, angles: np.ndarray) -> np.ndarray:
    ranges = np.asarray(ranges, dtype=float)
    angles = np.asarray(angles, dtype=float)
    if ranges.shape != angles.shape:
        raise ValueError("ranges and angles must match")
    return np.column_stack([ranges * np.cos(angles), ranges * np.sin(angles)])
