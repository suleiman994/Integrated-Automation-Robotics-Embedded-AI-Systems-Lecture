"""Run the integrated educational laboratory for the comprehensive lecture."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.autonomous_vehicles import (
    dataset_motion_statistics,
    emergency_braking_required,
    stopping_distance,
    time_to_collision,
)
from src.computational_methods import (
    bisection,
    central_difference,
    gradient_descent,
    knapsack_dynamic_programming,
    pareto_nondominated,
    simpson_integral,
    weighted_sum_mcda,
)
from src.control_systems import (
    PID,
    arx_design_matrix,
    dlqr,
    is_controllable,
    is_observable,
    least_squares_identification,
    simulate_discrete_linear,
)
from src.deep_learning import (
    compression_report,
    convolution_parameter_count,
    lstm_parameter_count,
    magnitude_prune,
    uniform_symmetric_quantize,
)
from src.embedded_systems import (
    PeriodicTask,
    amdahl_speedup,
    dma_transfer_time,
    partition_pipeline,
    response_time_analysis,
    total_utilization,
)
from src.industrial_automation import (
    EventControlledMachine,
    dcs_parallel_availability,
    iiot_payload,
    overall_equipment_effectiveness,
    scale_4_20_ma,
)
from src.machine_learning import (
    gaussian_naive_bayes_fit,
    gaussian_naive_bayes_predict,
    linear_regression_fit,
    pca_fit_transform,
    q_learning_grid,
)
from src.perception import (
    foreground_mask,
    gaussian_blur,
    intersection_over_union,
    linear_kalman_fusion,
    radar_range_resolution,
    stereo_depth,
)
from src.robotics import (
    astar_grid,
    cubic_joint_trajectory,
    inverse_kinematics_two_link,
    planar_two_link_fk,
)


def control_lab() -> dict:
    dt = 0.05
    A = np.array([[1.0, dt], [0.0, 1.0]])
    B = np.array([[0.5 * dt ** 2], [dt]])
    C = np.array([[1.0, 0.0]])
    Q = np.diag([20.0, 2.0])
    R = np.array([[0.5]])
    K, P = dlqr(A, B, Q, R)

    inputs = np.ones((80, 1)) * 0.2
    states = simulate_discrete_linear(A, B, np.zeros(2), inputs)

    rng = np.random.default_rng(10)
    u = rng.normal(size=500)
    y = np.zeros(500)
    for index in range(1, len(y)):
        y[index] = 0.82 * y[index - 1] + 0.35 * u[index - 1] + rng.normal(0.0, 0.015)
    X, target = arx_design_matrix(u, y)
    identified = least_squares_identification(X, target)

    controller = PID(kp=1.5, ki=0.5, kd=0.08, dt=dt, lower=-2.0, upper=2.0)
    pid_outputs = [controller.update(1.0, measurement) for measurement in np.linspace(0.0, 1.0, 20)]

    return {
        "controllable": is_controllable(A, B),
        "observable": is_observable(A, C),
        "lqr_gain": K.tolist(),
        "riccati_trace": float(np.trace(P)),
        "final_open_loop_position": float(states[-1, 0]),
        "identified_parameters": identified["parameters"].tolist(),
        "identification_rmse": identified["rmse"],
        "final_pid_output": float(pid_outputs[-1]),
    }


def embedded_lab() -> dict:
    tasks = [
        PeriodicTask("current_control", 0.15, 1.0),
        PeriodicTask("state_estimator", 0.45, 5.0),
        PeriodicTask("planner", 2.0, 20.0),
        PeriodicTask("logger", 1.0, 50.0),
    ]
    schedulability = response_time_analysis(tasks)
    stages = [
        {"name": "camera_preprocessing", "cpu_cost": 55, "fpga_cost": 18, "gpu_cost": 30},
        {"name": "cnn_inference", "cpu_cost": 90, "fpga_cost": 48, "gpu_cost": 22},
        {"name": "state_estimation", "cpu_cost": 20, "fpga_cost": 32, "gpu_cost": 35},
        {"name": "planning", "cpu_cost": 28, "fpga_cost": 70, "gpu_cost": 45},
    ]
    partition = partition_pipeline(stages, cpu_capacity=60, fpga_capacity=70, gpu_capacity=60)
    return {
        "total_utilization": total_utilization(tasks),
        "all_tasks_schedulable": bool(all(row["schedulable"] for row in schedulability)),
        "response_times": schedulability,
        "pipeline_partition": partition,
        "dma_1mb_ms": 1000.0 * dma_transfer_time(1_000_000, 400_000_000, 20e-6),
        "amdahl_speedup_8cores": amdahl_speedup(0.92, 8),
    }


def robotics_lab() -> dict:
    lengths = (0.8, 0.6)
    target = np.array([0.9, 0.55])
    joints = inverse_kinematics_two_link(target, lengths)
    recovered = planar_two_link_fk(joints, lengths)
    times = np.linspace(0.0, 2.5, 101)
    trajectory = cubic_joint_trajectory(np.zeros(2), joints, 2.5, times)

    grid = np.zeros((18, 24), dtype=bool)
    grid[5:14, 10] = True
    grid[9, 10:19] = True
    grid[9, 14] = False
    path = astar_grid(grid, (1, 1), (16, 22))

    return {
        "ik_solution_rad": joints.tolist(),
        "fk_reconstruction_error": float(np.linalg.norm(recovered - target)),
        "peak_joint_speed": float(np.max(np.abs(trajectory["velocity"]))),
        "path_length_cells": len(path),
        "path_found": bool(path),
    }


def perception_lab() -> dict:
    background = np.zeros((40, 50), dtype=float)
    frame = background.copy()
    frame[12:24, 20:34] = 1.0
    blurred = gaussian_blur(frame)
    predicted = foreground_mask(blurred, background, threshold=0.25)
    truth = frame > 0.5

    disparities = np.array([8.0, 16.0, 32.0])
    depths = stereo_depth(disparities, focal_length_pixels=700.0, baseline_meters=0.22)

    fusion = linear_kalman_fusion(
        prior_state=np.array([10.0, 0.0]),
        prior_covariance=np.diag([4.0, 3.0]),
        measurements=[np.array([9.7]), np.array([10.1])],
        measurement_matrices=[np.array([[1.0, 0.0]]), np.array([[1.0, 0.0]])],
        measurement_covariances=[np.array([[0.8]]), np.array([[0.3]])],
    )

    return {
        "foreground_iou": intersection_over_union(predicted, truth),
        "stereo_depths_m": depths.tolist(),
        "fused_position": float(fusion["state"][0]),
        "fused_position_variance": float(fusion["covariance"][0, 0]),
        "radar_range_resolution_m": radar_range_resolution(1.5e9),
    }


def autonomous_lab() -> dict:
    timestamps = np.linspace(0.0, 12.0, 121)
    positions = np.column_stack([
        1.8 * timestamps,
        0.5 * np.sin(0.35 * timestamps),
    ])
    motion = dataset_motion_statistics(positions, timestamps)
    distance = 28.0
    ego_speed = 18.0
    obstacle_speed = 8.0
    return {
        "sae_levels": 6,
        "stopping_distance_m": stopping_distance(ego_speed, 0.5, 7.0),
        "ttc_s": time_to_collision(distance, ego_speed - obstacle_speed),
        "emergency_braking_required": emergency_braking_required(
            distance,
            ego_speed,
            obstacle_speed,
            threshold_s=3.0,
        ),
        "motion_statistics": {
            key: value.tolist() if isinstance(value, np.ndarray) else value
            for key, value in motion.items()
        },
    }


def industrial_lab() -> dict:
    machine = EventControlledMachine()
    machine.update(0.0, start=True)
    machine.update(1.0, product_detected=True)
    machine.update(2.0, process_complete=True)
    payload = iiot_payload(
        "robot-cell-01",
        {
            "cycle_time_s": 8.2,
            "motor_current_a": 5.4,
            "temperature_c": 48.5,
        },
    )
    return {
        "scaled_pressure_bar": scale_4_20_ma(12.0, 0.0, 10.0),
        "redundant_dcs_availability": dcs_parallel_availability(0.9995, 2),
        "oee": overall_equipment_effectiveness(0.94, 0.91, 0.985),
        "produced_count": machine.produced_count,
        "event_count": len(machine.events),
        "iiot_payload": json.loads(payload),
    }


def machine_learning_lab() -> dict:
    rng = np.random.default_rng(12)
    X = rng.normal(size=(300, 4))
    y_reg = 1.5 + X @ np.array([2.0, -0.8, 0.0, 1.2]) + rng.normal(0.0, 0.08, size=300)
    regression = linear_regression_fit(X, y_reg, ridge=1e-3)
    pca = pca_fit_transform(X, components=2)

    labels = (X[:, 0] + 0.6 * X[:, 1] > 0.0).astype(int)
    nb = gaussian_naive_bayes_fit(X, labels)
    predictions = gaussian_naive_bayes_predict(nb, X)

    rewards = -np.ones((5, 6))
    rewards[4, 5] = 20.0
    q = q_learning_grid(rewards, terminal=(4, 5), episodes=1500, random_state=3)

    return {
        "regression_rmse": regression["rmse"],
        "regression_weights": regression["weights"].tolist(),
        "pca_explained_variance": pca["explained_variance_ratio"].tolist(),
        "naive_bayes_accuracy": float(np.mean(predictions == labels)),
        "q_policy_shape": list(q["policy"].shape),
    }


def deep_learning_lab() -> dict:
    rng = np.random.default_rng(13)
    weights = rng.normal(0.0, 0.25, size=2000)
    quantized = uniform_symmetric_quantize(weights, bits=8)
    pruned = magnitude_prune(weights, sparsity=0.65)
    conv_parameters = convolution_parameter_count(32, 64, 3, 3)
    lstm_parameters = lstm_parameter_count(16, 64, layers=2)
    report = compression_report(
        parameter_count=1_200_000,
        original_bits=32,
        quantized_bits=8,
        sparsity=0.55,
    )
    return {
        "conv_parameter_count": conv_parameters,
        "lstm_parameter_count": lstm_parameters,
        "quantization_rmse": quantized["rmse"],
        "actual_pruning_sparsity": pruned["actual_sparsity"],
        "compression_report": report,
    }


def computational_lab() -> dict:
    root = bisection(lambda x: x ** 3 - 2.0, 0.0, 2.0)
    derivative = central_difference(np.sin, 0.5)
    integral = simpson_integral(np.sin, 0.0, np.pi, intervals=200)

    objective = lambda x: float((x[0] - 2.0) ** 2 + 3.0 * (x[1] + 1.0) ** 2)
    gradient = lambda x: np.array([2.0 * (x[0] - 2.0), 6.0 * (x[1] + 1.0)])
    optimized = gradient_descent(objective, gradient, np.array([-3.0, 4.0]))

    knapsack = knapsack_dynamic_programming(
        np.array([12.0, 7.0, 18.0, 10.0, 14.0]),
        np.array([4, 2, 7, 3, 5]),
        capacity=12,
    )

    alternatives = np.array([
        [92, 28, 14, 0.88],
        [88, 19, 9, 0.82],
        [95, 42, 22, 0.93],
        [84, 15, 6, 0.78],
    ], dtype=float)
    mcda = weighted_sum_mcda(
        alternatives,
        weights=np.array([0.35, 0.25, 0.20, 0.20]),
        benefit_criteria=np.array([True, False, False, True]),
    )
    nondominated = pareto_nondominated(
        alternatives[:, [0, 1, 2]],
        minimize=np.array([False, True, True]),
    )

    return {
        "cube_root_two": root["root"],
        "sin_derivative_at_0_5": derivative,
        "integral_sin_0_pi": integral,
        "optimized_point": optimized["x"].tolist(),
        "optimized_objective": optimized["objective"],
        "knapsack": {
            "selected": knapsack["selected"].tolist(),
            "value": knapsack["value"],
            "weight": knapsack["weight"],
        },
        "mcda_scores": mcda["scores"].tolist(),
        "mcda_ranking": mcda["ranking"].tolist(),
        "pareto_nondominated": nondominated.tolist(),
    }


def flatten_summary(domain_results: dict) -> pd.DataFrame:
    rows = []
    for domain, result in domain_results.items():
        numeric_items = 0
        for value in result.values():
            if isinstance(value, (int, float, bool, np.integer, np.floating)):
                numeric_items += 1
        rows.append({
            "domain": domain,
            "top_level_metrics": len(result),
            "numeric_top_level_metrics": numeric_items,
            "status": "completed",
        })
    return pd.DataFrame(rows)


def main() -> None:
    results_directory = ROOT / "results"
    results_directory.mkdir(parents=True, exist_ok=True)

    domain_results = {
        "control_systems": control_lab(),
        "embedded_systems": embedded_lab(),
        "robotics": robotics_lab(),
        "perception_and_fusion": perception_lab(),
        "autonomous_vehicles": autonomous_lab(),
        "industrial_automation": industrial_lab(),
        "machine_learning": machine_learning_lab(),
        "deep_learning": deep_learning_lab(),
        "computational_methods": computational_lab(),
    }

    def json_default(value):
        if isinstance(value, np.ndarray):
            return value.tolist()
        if isinstance(value, np.generic):
            return value.item()
        raise TypeError(f"Object of type {type(value).__name__} is not JSON serializable")

    (results_directory / "integrated_lab_results.json").write_text(
        json.dumps(domain_results, indent=2, default=json_default),
        encoding="utf-8",
    )

    summary = flatten_summary(domain_results)
    summary.to_csv(
        results_directory / "integrated_domain_summary.csv",
        index=False,
    )

    print(summary.to_string(index=False))
    print("Integrated laboratory completed successfully.")


if __name__ == "__main__":
    main()
