"""Deep-learning architecture and compression utilities."""

from __future__ import annotations

import math
import numpy as np


def dense_parameter_count(input_features: int, output_features: int, bias: bool = True) -> int:
    return input_features * output_features + (output_features if bias else 0)


def convolution_parameter_count(
    input_channels: int,
    output_channels: int,
    kernel_height: int,
    kernel_width: int,
    bias: bool = True,
) -> int:
    return (
        input_channels
        * output_channels
        * kernel_height
        * kernel_width
        + (output_channels if bias else 0)
    )


def lstm_parameter_count(input_size: int, hidden_size: int, layers: int = 1, bidirectional: bool = False) -> int:
    if min(input_size, hidden_size, layers) <= 0:
        raise ValueError("sizes must be positive")
    directions = 2 if bidirectional else 1
    total = 0
    current_input = input_size
    for _ in range(layers):
        per_direction = 4 * hidden_size * (
            current_input + hidden_size + 2
        )
        total += directions * per_direction
        current_input = hidden_size * directions
    return int(total)


def autoencoder_bottleneck_ratio(input_size: int, latent_size: int) -> float:
    if input_size <= 0 or latent_size <= 0:
        raise ValueError("sizes must be positive")
    return float(latent_size / input_size)


def uniform_symmetric_quantize(weights: np.ndarray, bits: int = 8) -> dict:
    weights = np.asarray(weights, dtype=float)
    if bits < 2:
        raise ValueError("bits must be at least two")
    maximum_integer = 2 ** (bits - 1) - 1
    maximum_absolute = np.max(np.abs(weights))
    scale = 1.0 if maximum_absolute <= 1e-12 else maximum_absolute / maximum_integer
    integer = np.clip(
        np.round(weights / scale),
        -maximum_integer,
        maximum_integer,
    ).astype(int)
    dequantized = integer.astype(float) * scale
    return {
        "integer": integer,
        "scale": float(scale),
        "dequantized": dequantized,
        "rmse": float(np.sqrt(np.mean((weights - dequantized) ** 2))),
    }


def magnitude_prune(weights: np.ndarray, sparsity: float) -> dict:
    weights = np.asarray(weights, dtype=float)
    if not 0.0 <= sparsity < 1.0:
        raise ValueError("sparsity must lie in [0,1)")
    threshold = np.quantile(np.abs(weights), sparsity)
    mask = np.abs(weights) > threshold
    pruned = weights * mask
    actual_sparsity = 1.0 - np.count_nonzero(pruned) / pruned.size
    return {
        "weights": pruned,
        "mask": mask,
        "threshold": float(threshold),
        "actual_sparsity": float(actual_sparsity),
    }


def memory_bytes(parameter_count: int, bits_per_parameter: int) -> int:
    if parameter_count < 0 or bits_per_parameter <= 0:
        raise ValueError("invalid memory arguments")
    return int(math.ceil(parameter_count * bits_per_parameter / 8.0))


def compression_report(
    parameter_count: int,
    original_bits: int,
    quantized_bits: int,
    sparsity: float,
) -> dict:
    original = memory_bytes(parameter_count, original_bits)
    effective_parameters = int(round(parameter_count * (1.0 - sparsity)))
    compressed = memory_bytes(effective_parameters, quantized_bits)
    return {
        "original_bytes": original,
        "compressed_bytes": compressed,
        "compression_ratio": float(original / max(compressed, 1)),
        "effective_parameters": effective_parameters,
    }


def saliency_linear_model(weights: np.ndarray, sample: np.ndarray) -> np.ndarray:
    weights = np.asarray(weights, dtype=float)
    sample = np.asarray(sample, dtype=float)
    return weights * sample


def integrated_gradients_linear(
    weights: np.ndarray,
    sample: np.ndarray,
    baseline: np.ndarray | None = None,
) -> np.ndarray:
    weights = np.asarray(weights, dtype=float)
    sample = np.asarray(sample, dtype=float)
    if baseline is None:
        baseline = np.zeros_like(sample)
    return weights * (sample - np.asarray(baseline, dtype=float))
