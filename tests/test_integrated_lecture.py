"""Tests for the integrated lecture repository."""

import json
import numpy as np
import pytest

from src.autonomous_vehicles import (
    SAE_LEVELS,
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
    integrated_gradients_linear,
    lstm_parameter_count,
    magnitude_prune,
    uniform_symmetric_quantize,
)
from src.embedded_systems import (
    PeriodicTask,
    amdahl_speedup,
    dma_transfer_time,
    rate_monotonic_utilization_bound,
    response_time_analysis,
    total_utilization,
)
from src.industrial_automation import (
    EventControlledMachine,
    MachineState,
    dcs_parallel_availability,
    iiot_payload,
    overall_equipment_effectiveness,
    scale_4_20_ma,
)
from src.machine_learning import (
    decision_stump_fit,
    decision_stump_predict,
    gaussian_naive_bayes_fit,
    gaussian_naive_bayes_predict,
    linear_regression_fit,
    linear_svm_fit,
    linear_svm_predict,
    pca_fit_transform,
    q_learning_grid,
)
from src.perception import (
    foreground_mask,
    gaussian_blur,
    intersection_over_union,
    lidar_cartesian,
    linear_kalman_fusion,
    radar_range_resolution,
    stereo_depth,
)
from src.robotics import (
    astar_grid,
    cubic_joint_trajectory,
    inverse_kinematics_two_link,
    logistic_probability,
    planar_two_link_fk,
    planar_two_link_jacobian,
)


def test_control_controllability_observability() -> None:
    A = np.array([[1.0, 1.0], [0.0, 1.0]])
    B = np.array([[0.5], [1.0]])
    C = np.array([[1.0, 0.0]])
    assert is_controllable(A, B)
    assert is_observable(A, C)


def test_discrete_simulation_shape() -> None:
    A = np.eye(2)
    B = np.array([[1.0], [0.0]])
    states = simulate_discrete_linear(
        A,
        B,
        np.zeros(2),
        np.ones((5, 1)),
    )
    assert states.shape == (6, 2)
    assert states[-1, 0] == pytest.approx(5.0)


def test_pid_limits() -> None:
    controller = PID(2.0, 1.0, 0.0, 0.1, lower=0.0, upper=1.0)
    assert controller.update(10.0, 0.0) == pytest.approx(1.0)
    assert 0.0 <= controller.update(0.0, 10.0) <= 1.0


def test_lqr_stabilizes_double_integrator() -> None:
    dt = 0.1
    A = np.array([[1.0, dt], [0.0, 1.0]])
    B = np.array([[0.5 * dt ** 2], [dt]])
    K, _ = dlqr(A, B, np.diag([10.0, 1.0]), np.array([[0.5]]))
    eigenvalues = np.linalg.eigvals(A - B @ K)
    assert np.all(np.abs(eigenvalues) < 1.0)


def test_identification_recovers_parameters() -> None:
    rng = np.random.default_rng(1)
    u = rng.normal(size=800)
    y = np.zeros(800)
    for index in range(1, len(y)):
        y[index] = 0.75 * y[index - 1] + 0.4 * u[index - 1]
    X, target = arx_design_matrix(u, y)
    result = least_squares_identification(X, target)
    assert np.allclose(result["parameters"], [0.75, 0.4], atol=1e-6)


def test_realtime_utilization_and_response_time() -> None:
    tasks = [
        PeriodicTask("fast", 1.0, 5.0),
        PeriodicTask("slow", 2.0, 20.0),
    ]
    assert total_utilization(tasks) == pytest.approx(0.3)
    assert rate_monotonic_utilization_bound(2) > 0.8
    assert all(row["schedulable"] for row in response_time_analysis(tasks))


def test_dma_and_amdahl() -> None:
    assert dma_transfer_time(1000, 1000) == pytest.approx(1.0)
    assert amdahl_speedup(0.0, 8) == pytest.approx(1.0)
    assert amdahl_speedup(0.9, 8) > 4.0


def test_robot_inverse_forward_kinematics() -> None:
    target = np.array([0.9, 0.4])
    lengths = (0.7, 0.6)
    q = inverse_kinematics_two_link(target, lengths)
    recovered = planar_two_link_fk(q, lengths)
    assert np.allclose(recovered, target, atol=1e-9)


def test_robot_jacobian_shape() -> None:
    jacobian = planar_two_link_jacobian(np.array([0.2, -0.4]), (0.8, 0.5))
    assert jacobian.shape == (2, 2)


def test_cubic_trajectory_boundary_conditions() -> None:
    times = np.linspace(0.0, 2.0, 21)
    result = cubic_joint_trajectory(
        np.array([0.0, 1.0]),
        np.array([1.0, -1.0]),
        2.0,
        times,
    )
    assert np.allclose(result["position"][0], [0.0, 1.0])
    assert np.allclose(result["position"][-1], [1.0, -1.0])
    assert np.allclose(result["velocity"][[0, -1]], 0.0)


def test_astar_finds_path() -> None:
    grid = np.zeros((10, 10), dtype=bool)
    grid[2:8, 5] = True
    grid[5, 5] = False
    path = astar_grid(grid, (0, 0), (9, 9))
    assert path[0] == (0, 0)
    assert path[-1] == (9, 9)


def test_logistic_probability() -> None:
    probabilities = logistic_probability(np.array([-10.0, 0.0, 10.0]))
    assert probabilities[0] < 0.001
    assert probabilities[1] == pytest.approx(0.5)
    assert probabilities[2] > 0.999


def test_perception_foreground_iou() -> None:
    background = np.zeros((12, 12))
    frame = background.copy()
    frame[3:7, 4:9] = 1.0
    blurred = gaussian_blur(frame)
    mask = foreground_mask(blurred, background, 0.2)
    assert intersection_over_union(mask, frame > 0.5) > 0.55


def test_stereo_depth_relation() -> None:
    depth = stereo_depth(np.array([10.0, 20.0]), 500.0, 0.2)
    assert depth[0] == pytest.approx(10.0)
    assert depth[1] == pytest.approx(5.0)


def test_kalman_fusion_reduces_variance() -> None:
    result = linear_kalman_fusion(
        np.array([0.0]),
        np.array([[10.0]]),
        [np.array([1.0]), np.array([1.2])],
        [np.array([[1.0]]), np.array([[1.0]])],
        [np.array([[1.0]]), np.array([[0.5]])],
    )
    assert result["covariance"][0, 0] < 0.5
    assert 0.8 < result["state"][0] < 1.3


def test_radar_resolution() -> None:
    resolution = radar_range_resolution(1e9)
    assert resolution == pytest.approx(0.149896229, rel=1e-6)


def test_lidar_cartesian() -> None:
    points = lidar_cartesian(
        np.array([1.0, 2.0]),
        np.array([0.0, np.pi / 2.0]),
    )
    assert np.allclose(points, [[1.0, 0.0], [0.0, 2.0]], atol=1e-12)


def test_vehicle_metrics() -> None:
    assert len(SAE_LEVELS) == 6
    assert stopping_distance(10.0, 0.5, 5.0) == pytest.approx(15.0)
    assert time_to_collision(20.0, 5.0) == pytest.approx(4.0)
    assert emergency_braking_required(20.0, 10.0, 5.0, threshold_s=4.0)


def test_industrial_scaling_and_oee() -> None:
    assert scale_4_20_ma(12.0, 0.0, 100.0) == pytest.approx(50.0)
    assert overall_equipment_effectiveness(0.9, 0.8, 0.95) == pytest.approx(0.684)
    assert dcs_parallel_availability(0.99, 2) == pytest.approx(0.9999)


def test_event_machine_sequence() -> None:
    machine = EventControlledMachine()
    machine.update(0.0, start=True)
    assert machine.state == MachineState.RUNNING
    machine.update(1.0, product_detected=True)
    assert machine.state == MachineState.PROCESSING
    machine.update(2.0, process_complete=True)
    assert machine.state == MachineState.RUNNING
    assert machine.produced_count == 1


def test_iiot_payload_valid_json() -> None:
    payload = json.loads(iiot_payload("asset-1", {"temperature": 45.0}))
    assert payload["asset_id"] == "asset-1"
    assert payload["measurements"]["temperature"] == pytest.approx(45.0)


def test_linear_regression() -> None:
    rng = np.random.default_rng(2)
    X = rng.normal(size=(200, 2))
    y = 1.0 + X @ np.array([2.0, -3.0])
    model = linear_regression_fit(X, y)
    assert model["intercept"] == pytest.approx(1.0, abs=1e-9)
    assert np.allclose(model["weights"], [2.0, -3.0], atol=1e-9)


def test_pca_variance_ratio() -> None:
    rng = np.random.default_rng(3)
    x = rng.normal(size=300)
    X = np.column_stack([x, 2.0 * x + 0.01 * rng.normal(size=300), rng.normal(size=300)])
    result = pca_fit_transform(X, 2)
    assert result["transformed"].shape == (300, 2)
    assert np.sum(result["explained_variance_ratio"]) > 0.9


def test_naive_bayes() -> None:
    X = np.array([
        [-2.0, -1.0],
        [-1.5, -1.2],
        [1.5, 1.1],
        [2.0, 0.9],
    ])
    y = np.array([0, 0, 1, 1])
    model = gaussian_naive_bayes_fit(X, y)
    prediction = gaussian_naive_bayes_predict(model, X)
    assert np.array_equal(prediction, y)


def test_linear_svm() -> None:
    X = np.array([
        [-2.0, -1.0],
        [-1.0, -2.0],
        [1.0, 2.0],
        [2.0, 1.0],
    ])
    y = np.array([-1, -1, 1, 1])
    model = linear_svm_fit(X, y, epochs=800)
    assert np.array_equal(linear_svm_predict(model, X), y)


def test_decision_stump() -> None:
    X = np.array([[0.0], [1.0], [2.0], [3.0]])
    y = np.array([0, 0, 1, 1])
    model = decision_stump_fit(X, y)
    assert np.array_equal(decision_stump_predict(model, X), y)


def test_q_learning_shape() -> None:
    rewards = -np.ones((4, 5))
    rewards[3, 4] = 10.0
    result = q_learning_grid(rewards, (3, 4), episodes=500)
    assert result["policy"].shape == (4, 5)
    assert result["q_values"].shape == (4, 5, 4)


def test_deep_parameter_counts() -> None:
    assert convolution_parameter_count(3, 16, 3, 3) == 448
    assert lstm_parameter_count(10, 20) == 2560


def test_quantization_and_pruning() -> None:
    weights = np.linspace(-1.0, 1.0, 1000)
    quantized = uniform_symmetric_quantize(weights, bits=8)
    pruned = magnitude_prune(weights, sparsity=0.5)
    assert quantized["rmse"] < 0.01
    assert 0.45 <= pruned["actual_sparsity"] <= 0.55


def test_compression_and_explanation() -> None:
    report = compression_report(1000, 32, 8, 0.5)
    assert report["compression_ratio"] == pytest.approx(8.0)
    attribution = integrated_gradients_linear(
        np.array([2.0, -1.0]),
        np.array([3.0, 4.0]),
    )
    assert np.allclose(attribution, [6.0, -4.0])


def test_numerical_methods() -> None:
    root = bisection(lambda x: x ** 2 - 2.0, 0.0, 2.0)
    assert root["root"] == pytest.approx(np.sqrt(2.0), abs=1e-8)
    assert central_difference(np.sin, 0.4) == pytest.approx(np.cos(0.4), abs=1e-8)
    assert simpson_integral(np.sin, 0.0, np.pi, 200) == pytest.approx(2.0, abs=1e-8)


def test_gradient_descent() -> None:
    objective = lambda x: float((x[0] - 2.0) ** 2 + (x[1] + 1.0) ** 2)
    gradient = lambda x: np.array([2.0 * (x[0] - 2.0), 2.0 * (x[1] + 1.0)])
    result = gradient_descent(objective, gradient, np.array([-3.0, 5.0]))
    assert np.allclose(result["x"], [2.0, -1.0], atol=1e-6)


def test_knapsack_dynamic_programming() -> None:
    result = knapsack_dynamic_programming(
        np.array([6.0, 10.0, 12.0]),
        np.array([1, 2, 3]),
        5,
    )
    assert result["value"] == pytest.approx(22.0)
    assert result["weight"] == 5


def test_mcda_and_pareto() -> None:
    matrix = np.array([
        [90.0, 20.0],
        [80.0, 10.0],
        [85.0, 15.0],
    ])
    result = weighted_sum_mcda(
        matrix,
        np.array([0.6, 0.4]),
        np.array([True, False]),
    )
    assert set(result["ranking"].tolist()) == {0, 1, 2}

    nondominated = pareto_nondominated(
        matrix,
        minimize=np.array([False, True]),
    )
    assert nondominated.dtype == bool
    assert np.sum(nondominated) >= 2
