"""Educational machine-learning and reinforcement-learning methods."""

from __future__ import annotations

import numpy as np


def train_test_split(
    features: np.ndarray,
    targets: np.ndarray,
    test_fraction: float = 0.2,
    random_state: int = 0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    X = np.asarray(features, dtype=float)
    y = np.asarray(targets)
    if not 0.0 < test_fraction < 1.0:
        raise ValueError("test_fraction must lie in (0,1)")
    rng = np.random.default_rng(random_state)
    order = rng.permutation(len(X))
    test_count = max(1, int(round(test_fraction * len(X))))
    test_index = order[:test_count]
    train_index = order[test_count:]
    return X[train_index], X[test_index], y[train_index], y[test_index]


def standardize_fit(features: np.ndarray) -> dict:
    X = np.asarray(features, dtype=float)
    mean = np.mean(X, axis=0)
    scale = np.std(X, axis=0)
    scale = np.where(scale <= 1e-12, 1.0, scale)
    return {"mean": mean, "scale": scale}


def standardize_transform(features: np.ndarray, parameters: dict) -> np.ndarray:
    return (
        np.asarray(features, dtype=float)
        - np.asarray(parameters["mean"], dtype=float)
    ) / np.asarray(parameters["scale"], dtype=float)


def linear_regression_fit(features: np.ndarray, targets: np.ndarray, ridge: float = 0.0) -> dict:
    X = np.asarray(features, dtype=float)
    y = np.asarray(targets, dtype=float)
    design = np.column_stack([np.ones(len(X)), X])
    regularizer = ridge * np.eye(design.shape[1])
    regularizer[0, 0] = 0.0
    coefficients = np.linalg.solve(
        design.T @ design + regularizer,
        design.T @ y,
    )
    prediction = design @ coefficients
    return {
        "intercept": float(coefficients[0]),
        "weights": coefficients[1:],
        "prediction": prediction,
        "rmse": float(np.sqrt(np.mean((y - prediction) ** 2))),
    }


def pca_fit_transform(features: np.ndarray, components: int) -> dict:
    X = np.asarray(features, dtype=float)
    if not 1 <= components <= X.shape[1]:
        raise ValueError("invalid component count")
    mean = np.mean(X, axis=0)
    centered = X - mean
    _, singular_values, vectors = np.linalg.svd(centered, full_matrices=False)
    basis = vectors[:components]
    transformed = centered @ basis.T
    variances = singular_values ** 2 / max(len(X) - 1, 1)
    explained_ratio = variances[:components] / np.sum(variances)
    return {
        "mean": mean,
        "components": basis,
        "transformed": transformed,
        "explained_variance_ratio": explained_ratio,
    }


def gaussian_naive_bayes_fit(features: np.ndarray, labels: np.ndarray) -> dict:
    X = np.asarray(features, dtype=float)
    y = np.asarray(labels)
    classes = np.unique(y)
    means = {}
    variances = {}
    priors = {}
    for label in classes:
        subset = X[y == label]
        means[label] = np.mean(subset, axis=0)
        variances[label] = np.var(subset, axis=0) + 1e-9
        priors[label] = len(subset) / len(X)
    return {
        "classes": classes,
        "means": means,
        "variances": variances,
        "priors": priors,
    }


def gaussian_naive_bayes_predict(model: dict, features: np.ndarray) -> np.ndarray:
    X = np.atleast_2d(np.asarray(features, dtype=float))
    predictions = []
    for row in X:
        scores = {}
        for label in model["classes"]:
            mean = model["means"][label]
            variance = model["variances"][label]
            log_likelihood = -0.5 * np.sum(
                np.log(2.0 * np.pi * variance)
                + (row - mean) ** 2 / variance
            )
            scores[label] = np.log(model["priors"][label]) + log_likelihood
        predictions.append(max(scores, key=scores.get))
    return np.asarray(predictions)


def linear_svm_fit(
    features: np.ndarray,
    labels: np.ndarray,
    learning_rate: float = 0.01,
    regularization: float = 0.01,
    epochs: int = 500,
) -> dict:
    X = np.asarray(features, dtype=float)
    y = np.asarray(labels, dtype=float)
    if not np.all(np.isin(y, [-1.0, 1.0])):
        raise ValueError("SVM labels must be -1 or +1")
    weights = np.zeros(X.shape[1], dtype=float)
    bias = 0.0
    for epoch in range(epochs):
        eta = learning_rate / np.sqrt(epoch + 1.0)
        for row, label in zip(X, y):
            margin = label * (weights @ row + bias)
            if margin >= 1.0:
                weights -= eta * regularization * weights
            else:
                weights -= eta * (regularization * weights - label * row)
                bias += eta * label
    return {"weights": weights, "bias": float(bias)}


def linear_svm_predict(model: dict, features: np.ndarray) -> np.ndarray:
    scores = np.asarray(features, dtype=float) @ model["weights"] + model["bias"]
    return np.where(scores >= 0.0, 1, -1)


def decision_stump_fit(features: np.ndarray, labels: np.ndarray) -> dict:
    X = np.asarray(features, dtype=float)
    y = np.asarray(labels)
    classes = np.unique(y)
    if len(classes) != 2:
        raise ValueError("decision stump supports binary labels")
    best = None
    for feature in range(X.shape[1]):
        values = np.unique(X[:, feature])
        thresholds = (
            (values[:-1] + values[1:]) / 2.0
            if len(values) > 1
            else values
        )
        for threshold in thresholds:
            for polarity in [1, -1]:
                prediction = np.where(
                    polarity * X[:, feature] < polarity * threshold,
                    classes[0],
                    classes[1],
                )
                error = np.mean(prediction != y)
                if best is None or error < best["error"]:
                    best = {
                        "feature": feature,
                        "threshold": float(threshold),
                        "polarity": polarity,
                        "classes": classes,
                        "error": float(error),
                    }
    return best


def decision_stump_predict(model: dict, features: np.ndarray) -> np.ndarray:
    X = np.asarray(features, dtype=float)
    return np.where(
        model["polarity"] * X[:, model["feature"]]
        < model["polarity"] * model["threshold"],
        model["classes"][0],
        model["classes"][1],
    )


def q_learning_grid(
    rewards: np.ndarray,
    terminal: tuple[int, int],
    episodes: int = 1000,
    learning_rate: float = 0.2,
    discount: float = 0.95,
    epsilon: float = 0.2,
    random_state: int = 0,
) -> dict:
    rewards = np.asarray(rewards, dtype=float)
    rows, columns = rewards.shape
    actions = [(1,0),(-1,0),(0,1),(0,-1)]
    q = np.zeros((rows, columns, len(actions)), dtype=float)
    rng = np.random.default_rng(random_state)

    for episode in range(episodes):
        state = (0, 0)
        for _ in range(rows * columns * 10):
            if state == terminal:
                break
            if rng.random() < epsilon:
                action_index = int(rng.integers(len(actions)))
            else:
                action_index = int(np.argmax(q[state]))
            dr, dc = actions[action_index]
            next_state = (
                int(np.clip(state[0] + dr, 0, rows - 1)),
                int(np.clip(state[1] + dc, 0, columns - 1)),
            )
            target = rewards[next_state]
            if next_state != terminal:
                target += discount * np.max(q[next_state])
            q[state + (action_index,)] += learning_rate * (
                target - q[state + (action_index,)]
            )
            state = next_state

    policy = np.argmax(q, axis=2)
    return {"q_values": q, "policy": policy}
