# Topic Coverage Matrix

This repository contains one comprehensive lecture. The matrix below maps every requested curriculum area to the main lecture section and executable material.

| Curriculum area | Lecture section | Executable module |
|---|---|---|
| Linear and nonlinear dynamic systems | Sections 2.1–2.3 | `src/control_systems.py` |
| Lumped and distributed parameters | Section 2.2 | Conceptual treatment in the lecture |
| Dynamic-system description and properties | Sections 2.1–2.4 | `src/control_systems.py` |
| System identification | Section 2.6 | `least_squares_identification`, `arx_design_matrix` |
| Open- and closed-loop properties | Section 2.5 | PID and simulation utilities |
| PID and tuning | Section 2.7 | `PID`, `ultimate_gain_pid_tuning` |
| Time-optimal, minimum-energy, and LQR control | Section 2.8 | `dlqr`, Riccati solver |
| Intelligent control | Section 2.9 | ML/RL modules and architecture discussion |
| Digital control and real-time implementation | Section 2.10 | Control and embedded modules |
| Hierarchical control | Section 2.11 | Integrated architecture and research workflow |
| Discrete Event Systems | Sections 2.12 and 7.5 | `EventControlledMachine` |
| FPGA systems | Section 3.3 | Heterogeneous partitioning laboratory |
| Zynq and heterogeneous programmable devices | Section 3.4 | `partition_pipeline` |
| ASICs and ASSPs | Section 3.5 | Architecture and application discussion |
| GPUs and embedded GPUs | Section 3.6 | Partitioning, speedup, and DL deployment analysis |
| Microprocessor architectures | Section 3.2 | Architecture discussion |
| Real-time systems | Section 3.7 | `PeriodicTask`, response-time analysis |
| Embedded programming, interrupts, and DMA | Sections 3.8–3.9 | `dma_transfer_time` |
| Printed circuit boards | Section 3.10 | PCB technology and design discussion |
| Industrial robot configurations | Section 4.1 | Robotics module |
| Robot kinematics and dynamics | Sections 4.2–4.3 | FK, IK, Jacobian |
| End-effector trajectory planning | Section 4.4 | `cubic_joint_trajectory` |
| Autonomous-robot trajectory planning | Section 4.5 | `astar_grid` |
| Autonomous vehicles and robot control | Sections 4.7 and 6 | Vehicle and robotics modules |
| Environment identification | Section 4.6 | Occupancy and mapping utilities |
| Vision preprocessing | Section 5.2 | `gaussian_blur`, convolution |
| Foreground segmentation | Section 5.3 | `foreground_mask` |
| Optical flow | Section 5.4 | Horn–Schunck implementation |
| Stereo vision | Section 5.5 | `stereo_depth` |
| Detection and tracking | Section 5.6 | Conceptual tracking architecture and Kalman fusion |
| Perception quality assessment | Section 5.9 | IoU, precision, recall, F1 |
| Radar and LiDAR | Sections 5.7–5.8 | Radar resolution and LiDAR conversion |
| Multi-domain sensor fusion | Sections 5.10–5.11 | `linear_kalman_fusion` |
| SAE automation classification | Section 6.1 | `SAE_LEVELS` |
| Autonomous-vehicle sensors and functions | Sections 6.2–6.4 | Vehicle metrics |
| Static and dynamic dataset parameters | Section 6.3 | `dataset_motion_statistics` |
| ADAS functionalities | Section 6.4 | TTC and stopping-distance utilities |
| Traffic planning | Sections 6.5–6.8 | Road graph planning and tracking control |
| Real industrial control structures | Section 7.1 | Industrial automation module |
| Real automation devices and processes | Section 7.2 | 4–20 mA and event-machine utilities |
| Distributed Control Systems | Section 7.4 | DCS availability model |
| Event control | Section 7.5 | Event-controlled machine |
| IIoT and Industry 4.0 | Sections 7.6–7.10 | IIoT JSON payload and OEE |
| ML methodology | Section 8.1 | ML module |
| Regression | Section 8.3 | `linear_regression_fit` |
| SVM | Section 8.4 | Linear SVM implementation |
| Decision trees | Section 8.5 | Decision stump implementation |
| PCA | Section 8.6 | SVD-based PCA |
| Naive Bayes | Section 8.7 | Gaussian Naive Bayes |
| Reinforcement learning | Section 8.8 | Grid Q-learning |
| CNN, RNN, and autoencoders | Sections 9.2–9.4 | Parameter and compression utilities |
| Video processing and anomaly detection | Sections 9.5–9.6 | System-level discussion |
| Interpretable and explainable AI | Section 9.7 | Saliency and integrated gradients |
| Embedded DL and model-size reduction | Sections 9.8–9.9 | Quantization, pruning, memory analysis |
| Approximation, algebra, and calculus | Sections 10.1–10.3 | Numerical methods module |
| Constrained and unconstrained optimization | Section 10.4 | Gradient and projected optimization |
| Operations research | Section 10.5 | Knapsack and decision examples |
| Continuous and discrete optimization | Section 10.6 | Integrated discussion |
| P, NP, NP-hard, and NP-complete | Section 10.7 | Complexity discussion |
| Exact algorithms and dynamic programming | Sections 10.8–10.9 | Knapsack DP |
| Constraint handling in approximate methods | Section 10.10 | Penalty and metaheuristic discussion |
| Multi-criteria decision analysis | Section 10.11 | Weighted MCDA and Pareto filtering |
| Preference structures and substitution coefficients | Section 10.11 | Weight ratios and ranking sensitivity |
| Reference alternatives and consequences | Section 10.11 | Ideal/reference and normalized decision matrix discussion |
