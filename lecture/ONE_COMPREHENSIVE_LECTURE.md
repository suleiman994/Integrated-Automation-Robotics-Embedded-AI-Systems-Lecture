# One Comprehensive Lecture on Integrated Automation, Robotics, Embedded AI, and Computational Methods

## A Research-Level Systems View

Modern automation systems are not built from a single discipline. A robot, autonomous vehicle, smart production line, or intelligent inspection platform combines:

- A physical process with continuous and discrete dynamics.
- Sensors that observe only part of the state and introduce uncertainty.
- Estimation algorithms that reconstruct state and environment.
- Controllers that generate safe and effective actions.
- Embedded computers that must meet timing, memory, power, and reliability limits.
- Communication networks that distribute information.
- Perception and learning models that interpret high-dimensional data.
- Optimization methods that choose parameters, trajectories, schedules, and design alternatives.
- Supervisory and enterprise systems that connect machine behavior to operational objectives.

The central engineering question is therefore not merely:

> Which algorithm is best?

It is:

> Which complete architecture can sense, estimate, decide, act, communicate, recover, and satisfy timing and safety requirements under uncertainty?

This lecture develops that architecture from first principles.

---

# 1. Integrated Cyber-Physical Perspective

<p align="center"><img src="../assets/figures/01_integrated_cyber_physical_stack.png" width="940"></p>

A useful abstraction is:

```text
Physical World
    ↓
Sensors and Measurement
    ↓
Signal Processing and Perception
    ↓
State and Environment Estimation
    ↓
Planning, Optimization, and Decision
    ↓
Feedback and Event Control
    ↓
Actuators and Power Electronics
    ↓
Physical World
```

Around this loop are four cross-cutting infrastructures:

```text
Embedded Computing
Communication Networks
Safety and Cybersecurity
Data, Learning, and Lifecycle Management
```

A technically strong design must answer six questions:

1. **Model:** What states, inputs, outputs, disturbances, and constraints describe the system?
2. **Observe:** What can be measured directly, and what must be estimated?
3. **Decide:** What controller, planner, optimizer, or policy should generate actions?
4. **Compute:** Which processor, FPGA, GPU, or distributed controller should execute each task?
5. **Guarantee:** What stability, timing, safety, robustness, and quality properties can be demonstrated?
6. **Validate:** How will simulation, software-in-the-loop, hardware-in-the-loop, experiments, and field data confirm the design?

---

# 2. Control Problems and Dynamic Systems

<p align="center"><img src="../assets/figures/02_control_systems_map.png" width="940"></p>

## 2.1 Dynamic-System Models

A dynamic system contains memory: its future behavior depends on its current state.

A general continuous-time nonlinear state-space model is

\[
\dot{x}(t)=f(x(t),u(t),d(t),t),
\]

\[
y(t)=h(x(t),u(t),v(t),t),
\]

where:

- \(x\) is the state vector.
- \(u\) is the control input.
- \(d\) is a process disturbance.
- \(y\) is the measured output.
- \(v\) is measurement uncertainty.

A linear time-invariant approximation is

\[
\dot{x}=Ax+Bu,
\qquad
y=Cx+Du.
\]

The corresponding discrete-time model is

\[
x_{k+1}=A_d x_k+B_d u_k,
\qquad
y_k=C_d x_k+D_d u_k.
\]

State-space models are preferred when:

- Multiple inputs and outputs interact.
- Internal states matter.
- Controllability and observability must be analyzed.
- Optimal control and estimation will be designed.
- Digital implementation is required.

Transfer functions are useful for linear input-output analysis:

\[
G(s)=\frac{Y(s)}{U(s)}.
\]

They expose poles, zeros, static gain, bandwidth, resonance, and frequency response, but hide the internal state realization.

---

## 2.2 Lumped and Distributed Parameters

A **lumped-parameter system** assumes state variables depend only on time. Examples include:

- Motor speed.
- Tank temperature represented by one average value.
- Rigid robot-joint coordinates.
- Electrical RLC circuits.

These are modeled by ordinary differential equations.

A **distributed-parameter system** has states that depend on time and spatial coordinates:

\[
\frac{\partial T(z,t)}{\partial t}
=
\alpha
\frac{\partial^2 T(z,t)}{\partial z^2}
+q(z,t).
\]

Examples include:

- Temperature along a furnace.
- Flexible beams and robot links.
- Fluid flow in a long pipeline.
- Diffusion and reaction processes.
- Electromagnetic fields.

Distributed systems are modeled by partial differential equations. Practical controllers often use:

- Finite-difference discretization.
- Finite-element models.
- Modal truncation.
- Reduced-order models.
- Boundary control.
- Observer designs for spatial fields.

The reduction step must preserve dominant dynamics. An aggressive reduction may produce a controller that is numerically convenient but physically misleading.

---

## 2.3 Linear and Nonlinear Systems

A linear system satisfies superposition. Most physical systems are nonlinear because of:

- Trigonometric robot kinematics.
- Friction and backlash.
- Saturation.
- Aerodynamic drag.
- Valve nonlinearities.
- Contact and collision.
- Product terms between states.
- Switching logic.

A nonlinear model may be linearized around an operating point \((x^\star,u^\star)\):

\[
\delta \dot{x}=A\delta x+B\delta u,
\]

where

\[
A=
\left.
\frac{\partial f}{\partial x}
\right|_{x^\star,u^\star},
\qquad
B=
\left.
\frac{\partial f}{\partial u}
\right|_{x^\star,u^\star}.
\]

Linearization is local. A controller designed at one operating point may fail when:

- The operating range is wide.
- Actuators saturate.
- Contacts appear.
- Vehicle speed changes substantially.
- Robot configuration approaches singularity.
- Process gains vary with production conditions.

Nonlinear-control tools include:

- Lyapunov analysis.
- Feedback linearization.
- Sliding-mode control.
- Backstepping.
- Gain scheduling.
- Model predictive control.
- Adaptive control.
- Hybrid control.

---

## 2.4 Fundamental Properties

### Stability

For an autonomous system

\[
\dot{x}=f(x),
\]

an equilibrium \(x^\star\) satisfies \(f(x^\star)=0\).

Important notions include:

- Lyapunov stability.
- Asymptotic stability.
- Exponential stability.
- Input-to-state stability.
- Bounded-input bounded-output stability.
- Robust stability.

For an LTI continuous system, asymptotic stability requires all eigenvalues of \(A\) to have negative real parts.

For a discrete system, all eigenvalues must lie inside the unit circle.

### Controllability

The pair \((A,B)\) is controllable if an input can move the state from any initial state to any final state in finite time.

For an \(n\)-state LTI system:

\[
\mathcal{C}
=
\begin{bmatrix}
B & AB & A^2B & \cdots & A^{n-1}B
\end{bmatrix}.
\]

Controllability requires

\[
\operatorname{rank}(\mathcal{C})=n.
\]

### Observability

The pair \((A,C)\) is observable if the initial state can be reconstructed from input-output data.

\[
\mathcal{O}
=
\begin{bmatrix}
C\\
CA\\
CA^2\\
\vdots\\
CA^{n-1}
\end{bmatrix}.
\]

Observability requires

\[
\operatorname{rank}(\mathcal{O})=n.
\]

These tests do more than classify a matrix. They determine whether:

- State feedback can place all relevant modes.
- An observer can reconstruct all states.
- Sensor or actuator placement is structurally adequate.
- Redundant sensing adds genuinely new information.

Executable implementations are in `src/control_systems.py`.

---

## 2.5 Open-Loop and Closed-Loop Properties

An open-loop system applies commands without using the measured result. It is simple but sensitive to model errors and disturbances.

A closed-loop system uses feedback:

\[
e(t)=r(t)-y(t).
\]

The controller maps error and possibly estimated state to input.

Key static properties include:

- Static gain.
- Steady-state error.
- Sensitivity to parameter changes.
- Disturbance rejection.
- Offset under constant disturbances.

Key dynamic properties include:

- Rise time.
- Settling time.
- Overshoot.
- Damping.
- Bandwidth.
- Phase and gain margins.
- Resonance.
- Control effort.
- Noise amplification.

A faster loop is not automatically better. Increasing bandwidth may:

- Amplify measurement noise.
- Excite neglected flexible modes.
- Increase actuator wear.
- Reduce robustness to delay.
- Violate digital timing constraints.

---

## 2.6 System Identification

System identification constructs a model from measured data.

A general workflow is:

```text
Experiment Design
→ Data Acquisition
→ Preprocessing
→ Model Structure Selection
→ Parameter Estimation
→ Validation
→ Uncertainty Analysis
```

### Static Identification

A static map may be modeled as:

\[
y=\phi(x)^\top \theta+\varepsilon.
\]

Examples:

- Sensor calibration.
- Valve flow characteristic.
- Motor torque-current map.
- Camera distortion map.

### Dynamic Identification

An ARX model is

\[
y_k
=
-a_1y_{k-1}
-\cdots
-a_ny_{k-n}
+b_1u_{k-1}
+\cdots
+b_mu_{k-m}
+e_k.
\]

Parameters can be estimated by least squares:

\[
\hat{\theta}
=
(\Phi^\top\Phi)^{-1}\Phi^\top Y,
\]

when the matrix is well conditioned.

Research-quality identification requires attention to:

- Persistent excitation.
- Sampling rate.
- Closed-loop bias.
- Delay.
- Noise color.
- Model order.
- Train-validation separation.
- Residual autocorrelation.
- Parameter uncertainty.
- Physical plausibility.

A low training error does not prove that the model is suitable for control.

---

## 2.7 PID Control and Tuning

The ideal PID law is

\[
u(t)
=
K_p e(t)
+
K_i\int_0^t e(\tau)d\tau
+
K_d\frac{de(t)}{dt}.
\]

Interpretation:

- \(P\): immediate correction.
- \(I\): removal of persistent offset.
- \(D\): prediction from error trend and damping.

Industrial implementation requires:

- Output saturation.
- Anti-windup.
- Derivative filtering.
- Setpoint weighting.
- Bumpless transfer.
- Manual/automatic mode.
- Sensor-failure handling.
- Rate limits.

Common tuning approaches include:

- Manual tuning.
- Ultimate gain methods.
- Step-response reaction-curve rules.
- IMC-based tuning.
- Frequency-response design.
- Optimization-based tuning.
- Gain scheduling.
- Relay auto-tuning.

Tuning must balance:

```text
Tracking
Disturbance Rejection
Noise Sensitivity
Robustness
Actuator Effort
Safety
```

---

## 2.8 Optimal Control

### Linear Quadratic Regulator

For

\[
x_{k+1}=Ax_k+Bu_k,
\]

minimize

\[
J
=
\sum_{k=0}^{\infty}
\left(
x_k^\top Qx_k
+
u_k^\top Ru_k
\right).
\]

The optimal state feedback is

\[
u_k=-Kx_k,
\]

where \(K\) is computed from the discrete algebraic Riccati equation.

Interpretation:

- Large \(Q\): state deviation is expensive.
- Large \(R\): control effort is expensive.
- Cross-coupled states require full matrix reasoning.
- State estimation is needed if all states are not measured.

### Minimum-Energy Control

The objective emphasizes

\[
J=\int_0^T u^\top Ru\,dt.
\]

It is useful when energy, propellant, thermal load, or actuator wear is dominant.

### Time-Optimal Control

The objective is to minimize terminal time. For bounded-input systems, the solution often has bang-bang structure:

\[
u(t)\in\{u_{\min},u_{\max}\}.
\]

Real implementations must account for:

- Switching delay.
- Actuator rate limits.
- Flexible dynamics.
- State and obstacle constraints.
- Chattering avoidance.

### Model Predictive Control

MPC repeatedly solves a finite-horizon constrained optimization problem:

```text
Measure or Estimate State
→ Predict Future Behavior
→ Optimize Control Sequence
→ Apply First Input
→ Repeat
```

It is powerful because constraints are explicit, but computational feasibility must be demonstrated.

---

## 2.9 Intelligent Control

Intelligent control includes controllers that use:

- Fuzzy logic.
- Neural networks.
- Adaptive models.
- Reinforcement learning.
- Expert rules.
- Hybrid model-learning structures.

A useful research distinction is:

1. **Learning for perception:** classify or estimate environment variables.
2. **Learning for modeling:** identify uncertain dynamics.
3. **Learning for tuning:** adapt controller parameters.
4. **Learning for direct control:** map observations to actions.
5. **Learning for supervision:** choose modes, references, or fallback strategies.

Direct learning-based control is the most difficult to certify. Safer architectures often place learning inside a bounded role:

```text
Learning Module
→ Reference, Model, or Parameter Estimate
→ Verified Controller
→ Safety Filter
→ Actuator
```

---

## 2.10 Digital Control and Real-Time Implementation

A digital controller must account for:

- Sampling.
- Quantization.
- Computation delay.
- Zero-order hold.
- Jitter.
- Packet delay.
- Sensor timestamp alignment.
- Task scheduling.
- Numeric precision.
- Overflow and saturation.

A practical design sequence is:

1. Select the control bandwidth.
2. Choose a sampling period sufficiently faster than dominant dynamics.
3. Discretize the plant and controller.
4. Evaluate delay and jitter.
5. Quantize coefficients.
6. Verify closed-loop poles and margins.
7. Implement anti-windup and fault handling.
8. Test software-in-the-loop.
9. Test processor-in-the-loop.
10. Test hardware-in-the-loop.
11. Commission gradually with safety limits.

---

## 2.11 Hierarchical Control

Hierarchical control separates time scales and responsibilities.

```text
Enterprise and Mission Layer
    ↓
Production or Task Planning
    ↓
Trajectory and Reference Generation
    ↓
Local Feedback Control
    ↓
Actuators
```

Examples:

- A fleet manager assigns jobs.
- A robot planner chooses a path.
- A trajectory generator creates smooth references.
- Joint controllers track the references.
- Drive electronics regulate current.

Higher layers are slower and more abstract. Lower layers are faster and closer to physics.

---

## 2.12 Discrete Event Systems

A discrete event system changes state when events occur.

It can be represented by:

- Finite-state machines.
- Automata.
- Petri nets.
- GRAFCET.
- Sequential Function Charts.
- Supervisory control theory.

Typical events include:

```text
Start
Stop
Object Detected
Timer Expired
Fault
Reset
Resource Acquired
Task Completed
```

Important properties include:

- Reachability.
- Liveness.
- Deadlock freedom.
- Nonblocking behavior.
- Mutual exclusion.
- Event controllability.
- Safety invariants.

Many industrial systems are hybrid:

- Continuous PID controls temperature.
- Event logic opens and closes valves.
- Supervisory logic changes recipes.
- Safety logic forces a trip.

---

# 3. Embedded Systems and Heterogeneous Computing

<p align="center"><img src="../assets/figures/03_embedded_computing_map.png" width="940"></p>

## 3.1 Embedded-System Definition

An embedded system is a computing system designed for a specific physical function under constraints such as:

- Timing.
- Power.
- Memory.
- Cost.
- Size.
- Reliability.
- Environmental conditions.
- Certification.
- Long lifecycle.

Unlike a general-purpose computer, its correctness includes physical timing.

---

## 3.2 Microprocessor and Microarchitecture

A processor typically contains:

- Instruction fetch.
- Decode.
- Register file.
- Arithmetic and logic units.
- Load/store units.
- Branch prediction.
- Pipeline control.
- Caches.
- Memory-management units.
- Interrupt controller.
- Timers.
- Debug infrastructure.

Key architecture choices include:

- RISC versus CISC.
- In-order versus out-of-order execution.
- Scalar versus superscalar.
- Single-core versus multicore.
- Shared versus private caches.
- Harvard versus von Neumann memory organization.
- Memory protection and virtualization.

Microarchitecture affects worst-case timing. Features that improve average performance may reduce predictability:

- Deep caches.
- Speculation.
- Out-of-order execution.
- Shared buses.
- Dynamic frequency scaling.

---

## 3.3 FPGA Systems

An FPGA contains programmable:

- Lookup tables.
- Flip-flops.
- Block RAM.
- DSP slices.
- Clock-management resources.
- High-speed transceivers.
- Programmable interconnect.

FPGA logic is spatially parallel. It is well suited to:

- Deterministic pipelines.
- High-rate sensor interfaces.
- Custom protocol handling.
- Image filtering.
- Motor-control PWM.
- Hardware timestamping.
- Low-latency safety monitoring.
- Encryption.
- Neural-network acceleration.

The design flow includes:

```text
Requirements
→ RTL or HLS Description
→ Simulation
→ Synthesis
→ Place and Route
→ Timing Analysis
→ Bitstream
→ Hardware Verification
```

Important metrics:

- Clock frequency.
- Initiation interval.
- Pipeline latency.
- LUT usage.
- Register usage.
- BRAM usage.
- DSP usage.
- Power.
- Timing slack.

---

## 3.4 Zynq and Heterogeneous Programmable Devices

A device such as a Zynq SoC combines:

- Processing system with ARM cores.
- Programmable FPGA fabric.
- Shared memory and interconnect.
- DMA engines.
- Peripheral interfaces.

A common partition is:

```text
FPGA Fabric:
Sensor interfaces
Deterministic filtering
Pixel pipelines
Timestamping
Safety watchdog

ARM Processor:
ROS or middleware
Planning
Supervision
Networking
System management
```

The engineering challenge is not merely moving code to hardware. It is partitioning by:

- Parallelism.
- Latency.
- Data movement.
- Determinism.
- Reconfigurability.
- Development cost.
- Verification burden.

---

## 3.5 ASICs and ASSPs

An ASIC is designed for a specific function and fabricated as fixed silicon.

Advantages:

- High performance.
- Low energy per operation.
- Small area.
- Strong intellectual-property control.

Disadvantages:

- High nonrecurring engineering cost.
- Long development cycle.
- Limited post-fabrication flexibility.
- Expensive verification.

An ASSP is a standardized application-specific product sold for a class of applications. Examples include:

- Motor-control ICs.
- Video encoders.
- Network controllers.
- Automotive radar processors.
- Safety monitoring ICs.
- Sensor hubs.

Use ASIC or ASSP when production volume, power, performance, or certification justifies reduced flexibility.

---

## 3.6 GPUs and Embedded GPUs

GPUs execute many parallel threads efficiently.

Their architecture includes:

- Streaming multiprocessors.
- Warps or wavefronts.
- Vector-like arithmetic units.
- Shared memory.
- Global memory.
- Texture and cache structures.
- Specialized matrix units in modern devices.

They are effective for:

- CNN inference.
- Image processing.
- Point-cloud processing.
- Dense simulation.
- Large matrix operations.
- Parallel optimization.

Performance depends on:

- Memory coalescing.
- Occupancy.
- Branch divergence.
- Arithmetic intensity.
- Data transfer.
- Kernel-launch overhead.
- Precision.

Embedded GPUs add constraints:

- Thermal envelope.
- Shared memory bandwidth.
- Power modes.
- Real-time coexistence with CPU tasks.
- Model-loading latency.

---

## 3.7 Real-Time Systems

A real-time system is correct only if results are produced within required timing.

### Hard Real Time

Missing a deadline is unacceptable.

Examples:

- Airbag deployment.
- Safety shutdown.
- High-speed motor commutation.
- Certain medical and aerospace functions.

### Firm Real Time

A late result has no value, but occasional misses may be tolerated.

### Soft Real Time

Late results reduce quality but are not catastrophic.

Examples:

- Visualization.
- Noncritical analytics.
- Background logging.

A periodic task has:

- Execution time \(C_i\).
- Period \(T_i\).
- Relative deadline \(D_i\).
- Priority.
- Release jitter.
- Blocking time.

Utilization is

\[
U=\sum_i\frac{C_i}{T_i}.
\]

Response-time analysis is stronger than utilization alone because it includes interference from higher-priority tasks.

---

## 3.8 Interrupts and DMA

### Interrupts

An interrupt suspends normal execution to service an event.

Design concerns:

- Interrupt priority.
- Nesting.
- Latency.
- Shared data.
- Critical sections.
- Deferred processing.
- Interrupt storms.

A good pattern is:

```text
Interrupt Service Routine:
Capture data
Clear source
Timestamp
Notify task
Exit quickly
```

### DMA

DMA transfers data without CPU copying every word.

Use cases:

- Camera frames.
- ADC buffers.
- Network packets.
- Audio streams.
- FPGA-CPU data transfer.

DMA introduces:

- Buffer ownership.
- Cache coherency.
- Alignment.
- Descriptor management.
- Completion interrupts.
- Backpressure.

---

## 3.9 Embedded Programming

Languages include:

- C.
- C++.
- Rust in selected systems.
- Assembly for critical low-level code.
- HDL for FPGA.
- Python for prototyping and orchestration.
- Model-based tools for generated control code.

Important programming concerns:

- Fixed-width types.
- Volatile hardware registers.
- Atomic operations.
- Memory-mapped I/O.
- Stack limits.
- Dynamic allocation policy.
- Deterministic execution.
- Watchdogs.
- Defensive programming.
- Unit and integration tests.

---

## 3.10 Printed Circuit Boards

A PCB transforms a schematic into a physical electrical system.

The design process includes:

```text
Requirements
→ Schematic
→ Component Selection
→ Stack-Up
→ Placement
→ Routing
→ Power Integrity
→ Signal Integrity
→ Thermal Design
→ DFM Review
→ Fabrication
→ Assembly
→ Bring-Up
```

Critical topics:

- Layer stack-up.
- Ground planes.
- Return-current paths.
- Controlled impedance.
- Differential pairs.
- Decoupling.
- Power-distribution network.
- EMI/EMC.
- Isolation.
- Creepage and clearance.
- Connector robustness.
- Test points.
- Thermal vias.
- Manufacturing tolerances.

A correct algorithm can fail because of a poor PCB return path, noisy power rail, inadequate decoupling, or thermal throttling.

---

# 4. Robotics

<p align="center"><img src="../assets/figures/04_robotics_pipeline.png" width="940"></p>

## 4.1 Industrial-Robot Configurations

Common configurations include:

- Cartesian.
- Cylindrical.
- SCARA.
- Articulated six-axis.
- Delta/parallel.
- Collaborative manipulators.
- Mobile manipulators.

Selection depends on:

- Workspace.
- Payload.
- Repeatability.
- Stiffness.
- Speed.
- Dexterity.
- Cost.
- Safety.
- Environmental rating.

---

## 4.2 Kinematics

Forward kinematics maps joint variables to end-effector pose:

\[
{}^0T_n
=
{}^0T_1
{}^1T_2
\cdots
{}^{n-1}T_n.
\]

Denavit-Hartenberg parameters provide a systematic representation.

Inverse kinematics solves:

\[
q=f^{-1}(x).
\]

Challenges include:

- Multiple solutions.
- Unreachable targets.
- Joint limits.
- Singularities.
- Collision.
- Continuity between solutions.

The Jacobian relates joint velocity to task-space velocity:

\[
\dot{x}=J(q)\dot{q}.
\]

Near a singularity, some task-space directions require very large joint velocities.

---

## 4.3 Robot Dynamics

Manipulator dynamics are commonly written as:

\[
M(q)\ddot{q}
+
C(q,\dot{q})\dot{q}
+
g(q)
+
f(\dot{q})
=
\tau.
\]

where:

- \(M(q)\) is the inertia matrix.
- \(C(q,\dot{q})\dot{q}\) contains Coriolis and centrifugal terms.
- \(g(q)\) is gravity.
- \(f(\dot{q})\) models friction.
- \(\tau\) is joint torque.

Control methods include:

- Independent joint PID.
- Computed torque.
- Impedance control.
- Admittance control.
- Operational-space control.
- Adaptive control.
- Robust control.
- Model predictive control.

---

## 4.4 End-Effector Trajectory Planning

A trajectory specifies position, velocity, and often acceleration over time.

Common profiles:

- Cubic polynomial.
- Quintic polynomial.
- Trapezoidal velocity.
- S-curve jerk-limited.
- Cartesian interpolation.
- Joint-space interpolation.

A valid trajectory must satisfy:

- Initial and final conditions.
- Joint limits.
- Velocity limits.
- Acceleration limits.
- Jerk limits.
- Collision constraints.
- Singularity avoidance.
- Synchronization.

---

## 4.5 Autonomous-Robot Planning

A mobile robot planning stack includes:

```text
Map
→ Global Planner
→ Path
→ Local Planner
→ Velocity Command
→ Feedback Control
```

Global methods:

- Dijkstra.
- A*.
- D*.
- Sampling-based planning.
- Graph search.
- Optimization-based planning.

Local methods:

- Dynamic Window Approach.
- Timed Elastic Band.
- Model predictive control.
- Velocity obstacles.
- Reactive potential fields.

The local planner must handle:

- Moving obstacles.
- Kinematic constraints.
- Braking distance.
- Sensor uncertainty.
- Limited field of view.
- Replanning latency.

---

## 4.6 Environment Identification

Environment models include:

- Occupancy grids.
- Feature maps.
- Point clouds.
- Signed distance fields.
- Semantic maps.
- Topological graphs.
- Dynamic-object tracks.

Localization and mapping methods include:

- Odometry.
- Scan matching.
- Visual odometry.
- SLAM.
- Graph optimization.
- Kalman filtering.
- Particle filtering.

A map is not simply a picture. It is a probabilistic representation with:

- Resolution.
- Coordinate frame.
- Timestamp.
- Uncertainty.
- Update model.
- Dynamic-object policy.

---

## 4.7 Autonomous-Vehicle and Robot Control

Control layers include:

- Longitudinal speed control.
- Lateral path tracking.
- Trajectory tracking.
- Wheel-speed control.
- Torque allocation.
- Stability control.

Typical methods:

- PID.
- Pure pursuit.
- Stanley control.
- LQR.
- MPC.
- Feedback linearization.

The planner and controller must be compatible. A path with impossible curvature cannot be repaired by a perfect controller.

---

## 4.8 Research Integration

A complete robot pipeline is:

```text
Sensors
→ Calibration and Synchronization
→ Perception
→ Localization and Mapping
→ Global Planning
→ Local Trajectory Optimization
→ Feedback Control
→ Actuation
→ Safety Supervision
```

Executable kinematics, trajectories, occupancy updates, and A* planning are provided in `src/robotics.py`.

---

# 5. Computer Vision, Radar, LiDAR, and Sensor Fusion

<p align="center"><img src="../assets/figures/05_perception_and_sensor_fusion.png" width="940"></p>

## 5.1 Perception as an Estimation Problem

Perception converts raw measurements into useful statements about the environment.

The pipeline is:

```text
Raw Sensor Data
→ Calibration
→ Synchronization
→ Preprocessing
→ Feature or Object Extraction
→ Tracking
→ Fusion
→ World Model
```

Every stage introduces assumptions. A detector does not directly observe an object as a physical truth. It produces an estimate conditioned on:

- Sensor resolution.
- Exposure.
- Weather.
- Training data.
- Calibration.
- Motion.
- Occlusion.
- Thresholds.
- Computational approximation.

Therefore perception quality must be described statistically and operationally.

---

## 5.2 Image Preprocessing

Typical operations include:

- Radiometric normalization.
- Denoising.
- Gaussian or median filtering.
- Contrast enhancement.
- Color-space transformation.
- Lens-distortion correction.
- Geometric rectification.
- Resize and crop.
- Temporal filtering.
- Edge extraction.

Preprocessing should improve the downstream task rather than merely make images visually attractive.

For example, aggressive smoothing may reduce noise but destroy small-object features.

---

## 5.3 Foreground Segmentation

Foreground segmentation separates moving or relevant regions from a background model.

Methods include:

- Frame differencing.
- Running-average background.
- Gaussian mixture models.
- Codebook methods.
- Optical-flow-based motion segmentation.
- Deep semantic segmentation.

Challenges:

- Illumination changes.
- Shadows.
- Camera motion.
- Dynamic background.
- Slow-moving objects.
- Stopped objects.
- Weather.

A binary mask should be evaluated using:

- Precision.
- Recall.
- F1 score.
- Intersection over Union.
- Boundary accuracy.
- Temporal stability.

---

## 5.4 Optical Flow

Optical flow estimates apparent image motion.

The brightness-constancy assumption is

\[
I(x,y,t)
=
I(x+\Delta x,y+\Delta y,t+\Delta t).
\]

Linearization gives

\[
I_xu+I_yv+I_t=0.
\]

This is one equation with two unknown velocity components. Additional assumptions are required.

Lucas-Kanade assumes locally constant motion.

Horn-Schunck imposes global smoothness.

Failure cases include:

- Aperture problem.
- Large displacement.
- Illumination variation.
- Motion blur.
- Low texture.
- Occlusion.
- Nonrigid motion.

Applications:

- Ego-motion.
- Object motion.
- Visual odometry.
- Time-to-contact.
- Video stabilization.
- Tracking.

---

## 5.5 Stereo Vision

For a rectified stereo pair:

\[
Z=\frac{fB}{d},
\]

where:

- \(Z\) is depth.
- \(f\) is focal length in pixels.
- \(B\) is baseline.
- \(d\) is disparity.

Depth uncertainty increases rapidly when disparity is small. Stereo therefore becomes less accurate at long range.

The stereo pipeline includes:

```text
Calibration
→ Rectification
→ Correspondence
→ Disparity
→ Consistency Check
→ Depth
→ Point Cloud
```

Challenges:

- Repetitive texture.
- Textureless surfaces.
- Occlusion.
- Reflective surfaces.
- Exposure mismatch.
- Calibration drift.

---

## 5.6 Detection and Tracking

### Detection

A detector estimates object class and location.

Outputs may include:

- Bounding box.
- Segmentation mask.
- Class probability.
- 3D position.
- Orientation.
- Keypoints.

Metrics include:

- Precision-recall curve.
- Average precision.
- Mean average precision.
- IoU thresholds.
- False positives per image.
- Detection range.
- Latency.

### Tracking

Tracking associates detections across time.

A common architecture is:

```text
Motion Prediction
→ Measurement Association
→ State Update
→ Track Management
```

Track state may include:

\[
x=
\begin{bmatrix}
p_x & p_y & v_x & v_y
\end{bmatrix}^{\top}.
\]

Methods:

- Kalman filter.
- Extended Kalman filter.
- Unscented Kalman filter.
- Particle filter.
- Multiple-hypothesis tracking.
- Joint probabilistic data association.
- Learned appearance association.

Tracking quality metrics include:

- Position error.
- Identity switches.
- Track fragmentation.
- Missed tracks.
- False tracks.
- Multi-object tracking accuracy and precision.

---

## 5.7 Radar

Automotive and industrial radar measures combinations of:

- Range.
- Radial velocity.
- Angle.
- Radar cross section.

Advantages:

- Long range.
- Direct Doppler velocity.
- Strong weather robustness.
- Useful through dust and fog.

Limitations:

- Lower spatial resolution than cameras or high-resolution LiDAR.
- Multipath.
- Ghost targets.
- Ambiguous reflections.
- Sparse detections.

For FMCW radar, range resolution is approximately

\[
\Delta R=\frac{c}{2B},
\]

where \(B\) is sweep bandwidth.

---

## 5.8 LiDAR

LiDAR measures range from emitted light.

Outputs:

- Point clouds.
- Intensity.
- Multiple returns.
- Ring or channel index.
- Timestamp.

Advantages:

- Accurate geometry.
- Direct metric depth.
- Strong object-shape information.

Limitations:

- Cost.
- Weather degradation.
- Sparse vertical resolution in some sensors.
- Reflectivity dependence.
- Motion distortion.
- Mechanical or optical complexity.

Processing includes:

- Ground removal.
- Clustering.
- Surface normals.
- Registration.
- Object detection.
- Occupancy mapping.
- Scan matching.

---

## 5.9 Quality Assessment of Perception Algorithms

Quality must be assessed at several levels.

### Data-Level Quality

- Signal-to-noise ratio.
- Resolution.
- Missing-data rate.
- Timestamp accuracy.
- Calibration error.
- Dynamic range.

### Algorithm-Level Quality

- Detection accuracy.
- Segmentation IoU.
- Depth RMSE.
- Optical-flow endpoint error.
- Tracking identity consistency.
- Calibration residual.

### System-Level Quality

- End-to-end braking success.
- Collision rate.
- Localization availability.
- Planning success.
- False emergency actions.
- Real-time deadline compliance.

### Robustness Slices

Report performance by:

- Day/night.
- Weather.
- Distance.
- Object size.
- Occlusion.
- Speed.
- Sensor degradation.
- Geographic region.
- Rare events.

A single average metric can hide dangerous failure modes.

---

## 5.10 Multi-Domain Sensor Fusion

Fusion can occur at several levels.

### Raw/Data-Level Fusion

Combine measurements before feature extraction.

Advantages:

- Maximum information retention.

Challenges:

- Bandwidth.
- Synchronization.
- Calibration.
- High dimensionality.

### Feature-Level Fusion

Combine extracted features.

Examples:

- Camera features with projected LiDAR points.
- Radar Doppler features with image embeddings.

### Object/Decision-Level Fusion

Combine tracks, object lists, or decisions.

Advantages:

- Modular.
- Lower bandwidth.
- Easier subsystem replacement.

Challenges:

- Information loss.
- Correlated uncertainty.
- Association conflicts.

---

## 5.11 Kalman Fusion

For a linear measurement model

\[
z_k=Hx_k+v_k,
\]

the innovation is

\[
\nu_k=z_k-H\hat{x}_{k|k-1}.
\]

The Kalman gain is

\[
K_k
=
P_{k|k-1}H^\top
\left(
HP_{k|k-1}H^\top+R
\right)^{-1}.
\]

The update is

\[
\hat{x}_{k|k}
=
\hat{x}_{k|k-1}
+
K_k\nu_k.
\]

Fusion requires careful treatment of:

- Coordinate frames.
- Sensor latency.
- Out-of-sequence measurements.
- Correlated noise.
- Data association.
- Faulty sensors.
- Adaptive covariance.
- Track existence probability.

Executable preprocessing, flow, stereo, metrics, radar resolution, LiDAR conversion, and Kalman fusion are in `src/perception.py`.

---

# 6. Autonomous Vehicles

<p align="center"><img src="../assets/figures/06_autonomous_vehicle_stack.png" width="940"></p>

## 6.1 Automation Levels

The common road-vehicle automation taxonomy uses levels from 0 to 5.

- **Level 0:** no sustained driving automation.
- **Level 1:** assistance in one dimension such as speed or steering.
- **Level 2:** combined lateral and longitudinal assistance under continuous driver supervision.
- **Level 3:** conditional automation within a defined operational domain, with fallback requests.
- **Level 4:** high automation within a defined operational domain without expecting human fallback.
- **Level 5:** full automation across all road conditions that a human could manage.

The crucial concept is not merely the level number. It is the **dynamic driving task**, **fallback responsibility**, and **operational design domain**.

---

## 6.2 Autonomous-Vehicle Sensors

A typical sensor suite may include:

- Cameras.
- Radar.
- LiDAR.
- GNSS.
- IMU.
- Wheel encoders.
- Steering-angle sensors.
- Ultrasonic sensors.
- Vehicle-bus signals.
- High-definition maps.

No single sensor dominates all conditions.

| Sensor | Strength | Limitation |
|---|---|---|
| Camera | rich semantics and color | lighting and weather sensitivity |
| Radar | range and radial velocity | lower angular detail |
| LiDAR | accurate 3D geometry | weather, cost, sparsity |
| GNSS | global position | blockage and multipath |
| IMU | high-rate motion | drift |
| Wheel odometry | local motion | slip |

The architecture should exploit complementarity and detect inconsistency.

---

## 6.3 Dataset Description

Large autonomous-driving datasets should be characterized by more than sample count.

### Static Aspects

- Number of frames.
- Number of scenes.
- Sensor modalities.
- Resolution.
- Geographic diversity.
- Weather classes.
- Road classes.
- Object-class distribution.
- Label quality.
- Rare-event count.

### Dynamic Aspects

- Speed distribution.
- Acceleration distribution.
- Turn-rate distribution.
- Time-to-collision distribution.
- Interaction density.
- Track duration.
- Occlusion duration.
- Temporal sampling.
- Motion blur.
- Sensor latency.

Data leakage is a major risk. Random frame splitting may place nearly identical consecutive frames into training and test sets. Scene-level or route-level splitting is usually more meaningful.

---

## 6.4 Driver-Assistance Functions

Selected ADAS functions include:

- Adaptive cruise control.
- Lane-keeping assistance.
- Lane-departure warning.
- Automatic emergency braking.
- Blind-spot monitoring.
- Parking assistance.
- Traffic-sign recognition.
- Driver monitoring.
- Forward-collision warning.

### Time to Collision

For relative distance \(d\) and closing speed \(v_c\):

\[
TTC=\frac{d}{v_c},
\]

when \(v_c>0\).

TTC is useful but incomplete because it ignores:

- Acceleration.
- Road friction.
- Curvature.
- Reaction delay.
- Object uncertainty.
- Lateral escape.

### Stopping Distance

A simple model is

\[
d_{\text{stop}}
=
vT_r+\frac{v^2}{2a},
\]

where \(T_r\) is reaction delay and \(a\) is available deceleration.

---

## 6.5 Functional Architecture

A common autonomous-driving stack is:

```text
Sensor Acquisition
→ Calibration and Time Synchronization
→ Localization
→ Perception and Tracking
→ Prediction
→ Behavior Planning
→ Motion Planning
→ Control
→ Vehicle Actuation
```

Cross-cutting components:

- Map management.
- Health monitoring.
- Safety supervisor.
- Cybersecurity.
- Data logging.
- Simulation.
- Remote operations.

---

## 6.6 Traffic Planning Problem

Given a start state and goal state, planning must produce a collision-free and dynamically feasible trajectory.

The problem contains:

- Road geometry.
- Traffic rules.
- Static obstacles.
- Dynamic agents.
- Vehicle dynamics.
- Comfort constraints.
- Safety margins.
- Prediction uncertainty.
- Real-time computational limits.

Planning is usually hierarchical.

### Route Planning

Chooses roads or lanes over a graph.

Methods:

- Dijkstra.
- A*.
- Contraction hierarchies.
- Multi-criteria route search.

### Behavior Planning

Chooses semantic actions:

- Follow.
- Stop.
- Yield.
- Overtake.
- Change lane.
- Merge.
- Park.

Representations:

- State machines.
- Behavior trees.
- POMDPs.
- Rule systems.
- Learned policies.

### Motion Planning

Produces a geometric or time-parameterized trajectory.

Methods:

- Lattice planning.
- Sampling-based planning.
- Polynomial trajectories.
- Optimization-based planning.
- MPC.
- Hybrid A*.
- Graph search in state-time space.

---

## 6.7 Prediction

Prediction estimates future motion of other agents.

Models range from:

- Constant velocity.
- Constant acceleration.
- Lane-following models.
- Interaction models.
- RNNs and transformers.
- Multi-modal probabilistic predictors.

A good predictor must represent multiple plausible futures. A single mean trajectory can be unsafe when the other vehicle may either turn or continue straight.

---

## 6.8 Localization

Localization fuses:

- GNSS.
- IMU.
- Wheel odometry.
- Camera.
- LiDAR.
- Radar.
- Map landmarks.

Key outputs:

- Position.
- Orientation.
- Velocity.
- Bias estimates.
- Covariance.
- Health flags.

The planner should consume uncertainty, not just the mean pose.

---

## 6.9 Control

Longitudinal control regulates:

- Speed.
- Following distance.
- Acceleration.
- Braking.

Lateral control regulates:

- Heading.
- Cross-track error.
- Curvature tracking.
- Steering.

Methods include:

- PID.
- Pure pursuit.
- Stanley.
- LQR.
- MPC.

The controller must respect:

- Tire friction.
- Steering limits.
- Actuator delay.
- Braking limits.
- Road grade.
- Comfort.
- Stability envelope.

Executable SAE-level descriptions, stopping distance, TTC, road-graph planning, occupancy statistics, motion statistics, and pure-pursuit curvature are in `src/autonomous_vehicles.py`.

---

# 7. Automation of Industrial Processes

<p align="center"><img src="../assets/figures/07_industrial_automation_architecture.png" width="940"></p>

## 7.1 Real Process-Control Structure

A real industrial control loop is:

```text
Setpoint
→ Controller
→ Output Module
→ Actuator
→ Process
→ Sensor
→ Transmitter
→ Input Module
→ Controller
```

A complete industrial system adds:

- Interlocks.
- Permissives.
- Alarms.
- Trips.
- Safety instrumented functions.
- HMI.
- Historian.
- Communication.
- Maintenance diagnostics.
- Production management.

---

## 7.2 Elements of Automation Systems

### Sensors and Transmitters

- Temperature.
- Pressure.
- Flow.
- Level.
- Position.
- Speed.
- Vibration.
- Force.
- Vision.
- Chemical analyzers.

### Industrial Signals

- Digital 24 VDC.
- 4–20 mA.
- 0–10 V.
- Pulse/frequency.
- Encoder signals.
- Fieldbus data.
- Industrial Ethernet.

### Actuators

- Electric motors.
- Variable-frequency drives.
- Servo drives.
- Control valves.
- Solenoid valves.
- Pneumatic cylinders.
- Hydraulic actuators.
- Heaters.
- Pumps.

### Controllers

- PLC.
- PAC.
- DCS controller.
- Safety PLC.
- Motion controller.
- Robot controller.
- Industrial PC.

---

## 7.3 Open and Closed Industrial Loops

An open-loop conveyor may run for a fixed time.

A closed-loop conveyor may use:

- Encoder feedback.
- Position sensor.
- Motor-current monitoring.
- Product-detection sensors.

A process can contain nested loops:

```text
Flow Controller
→ Valve
→ Flow

Level Controller
→ Flow Setpoint
→ Flow Controller
```

This is cascade control.

---

## 7.4 Distributed Control Systems

A DCS distributes process controllers near plant areas while coordinating them through redundant networks.

Components:

- Process controllers.
- Remote I/O.
- Operator stations.
- Engineering stations.
- Alarm servers.
- Historians.
- Asset-management servers.
- Redundant networks.
- Redundant power supplies.
- Time synchronization.

DCS is common in:

- Chemical plants.
- Refineries.
- Power plants.
- Water treatment.
- Pulp and paper.
- Pharmaceutical batch processes.

Redundancy improves availability but does not eliminate:

- Common-cause faults.
- Configuration errors.
- Cyberattacks.
- Shared power failures.
- Incorrect failover logic.
- Maintenance errors.

---

## 7.5 Event Control

Industrial event control uses:

- PLC logic.
- State machines.
- SFC.
- GRAFCET.
- Petri nets.
- Timers.
- Counters.
- Edge detection.
- Interlocks.

A packaging sequence may be:

```text
IDLE
→ CONVEYING
→ POSITIONING
→ FILLING
→ CAPPING
→ EJECTING
→ CONVEYING
```

Every state should define:

- Entry condition.
- Active outputs.
- Exit condition.
- Timeout.
- Fault behavior.
- Reset behavior.

---

## 7.6 IIoT

The Industrial Internet of Things connects industrial assets to data and analytics platforms.

Architecture:

```text
Sensors and Machines
→ PLC/DCS/Embedded Controller
→ Edge Gateway
→ OPC UA or MQTT
→ On-Premise or Cloud Platform
→ Analytics and Digital Twin
→ MES/ERP/Maintenance
```

Fast control should remain local. Cloud services are better suited to:

- Fleet analytics.
- Long-term storage.
- Model training.
- Cross-site comparison.
- Reporting.
- Maintenance planning.

---

## 7.7 Industry 4.0

Important principles:

- Interoperability.
- Information transparency.
- Decentralized decisions.
- Real-time capability.
- Modularity.
- Technical assistance.
- Digital integration.

Technologies:

- Cyber-physical systems.
- Digital twins.
- Edge computing.
- Cloud platforms.
- Machine learning.
- Collaborative robots.
- Additive manufacturing.
- Smart logistics.
- Industrial data spaces.

---

## 7.8 OPC UA and MQTT

### OPC UA

Provides:

- Rich information models.
- Client/server communication.
- Events and methods.
- Security.
- Industrial semantics.
- Pub/sub options.

### MQTT

Provides:

- Lightweight publish/subscribe.
- Brokered messaging.
- Topic hierarchy.
- Quality-of-service levels.
- Efficient telemetry transport.

They can be combined:

```text
OPC UA:
Structured industrial data

MQTT:
Scalable transport
```

---

## 7.9 Industrial Cybersecurity

Controls include:

- Asset inventory.
- Network segmentation.
- Industrial DMZ.
- Role-based access.
- Secure remote access.
- Backup and restore.
- Patch governance.
- Application allow-listing.
- Monitoring.
- Incident response.
- Safety-security co-analysis.

The objective is not only protecting data. It is protecting physical behavior.

---

## 7.10 OEE and Production Integration

Overall Equipment Effectiveness is

\[
OEE
=
Availability
\times
Performance
\times
Quality.
\]

MES connects machine execution to:

- Work orders.
- Recipes.
- Traceability.
- Quality.
- Downtime.
- OEE.
- Material tracking.

ERP connects production to:

- Sales.
- Inventory.
- Purchasing.
- Finance.
- Planning.

Executable event control, 4–20 mA scaling, DCS availability, OEE, and IIoT payload generation are in `src/industrial_automation.py`.

---

# 8. Machine Learning and Artificial Intelligence

<p align="center"><img src="../assets/figures/08_machine_learning_and_deep_learning_lifecycle.png" width="940"></p>

## 8.1 Machine-Learning Methodology

A rigorous workflow is:

```text
Problem Definition
→ Data Collection
→ Data Audit
→ Split Strategy
→ Preprocessing
→ Baseline
→ Model Training
→ Validation
→ Robustness Testing
→ Deployment
→ Monitoring
```

The target should be operationally meaningful.

Examples:

- Predict motor health.
- Estimate object distance.
- Detect anomalous vibration.
- Classify road signs.
- Predict process quality.
- Select robot actions.

---

## 8.2 Data Splitting

Common splits:

- Training.
- Validation.
- Test.

For temporal systems, random sample splitting may leak future information.

Better approaches:

- Time-based split.
- Machine-based split.
- Site-based split.
- Route-based split.
- Operator-based split.
- Leave-one-condition-out.

The test set should represent the deployment question.

---

## 8.3 Regression

Linear regression models:

\[
\hat{y}=w^\top x+b.
\]

The least-squares objective is

\[
J(w,b)
=
\sum_i
(y_i-\hat{y}_i)^2.
\]

Extensions:

- Polynomial regression.
- Ridge.
- Lasso.
- Robust regression.
- Gaussian processes.
- Neural regression.

Metrics:

- MAE.
- RMSE.
- \(R^2\).
- Maximum error.
- Calibration.
- Prediction interval coverage.

---

## 8.4 Support Vector Machines

A linear SVM searches for a separating hyperplane with maximum margin.

For labels \(y_i\in\{-1,+1\}\), the soft-margin objective is

\[
\min_{w,b}
\frac{1}{2}\|w\|^2
+
C\sum_i \max(0,1-y_i(w^\top x_i+b)).
\]

Strengths:

- Strong performance with medium-sized structured data.
- Convex training problem.
- Kernel extensions.

Limitations:

- Scaling to very large datasets.
- Probability calibration.
- Kernel and hyperparameter selection.
- Interpretability in nonlinear kernels.

---

## 8.5 Decision Trees

A decision tree recursively splits features to reduce impurity.

Advantages:

- Interpretable rules.
- Mixed nonlinear interactions.
- Little preprocessing.
- Fast inference.

Limitations:

- Instability.
- Overfitting.
- Axis-aligned partitions.
- Poor extrapolation.

Ensembles improve accuracy:

- Random forests.
- Gradient boosting.
- Extra trees.

---

## 8.6 PCA

Principal Component Analysis finds orthogonal directions of maximum variance.

For centered data \(X\), SVD gives

\[
X=U\Sigma V^\top.
\]

Columns of \(V\) define principal directions.

PCA is used for:

- Visualization.
- Compression.
- Noise reduction.
- Correlation analysis.
- Feature decorrelation.
- Fault detection.

Limitations:

- Linear.
- Variance is not always task relevance.
- Sensitive to scaling.
- Components may be hard to interpret physically.

---

## 8.7 Naive Bayes

Gaussian Naive Bayes assumes features are conditionally independent given the class.

\[
p(y|x)
\propto
p(y)
\prod_j p(x_j|y).
\]

Despite the strong assumption, it can be effective for:

- Fast baselines.
- Small datasets.
- Embedded classification.
- Diagnostic features.

---

## 8.8 Reinforcement Learning

An RL problem contains:

- State \(s\).
- Action \(a\).
- Reward \(r\).
- Transition dynamics.
- Policy \(\pi(a|s)\).
- Return.

The objective is to maximize expected discounted return:

\[
G_t
=
\sum_{k=0}^{\infty}
\gamma^k r_{t+k+1}.
\]

Q-learning updates:

\[
Q(s,a)
\leftarrow
Q(s,a)
+
\alpha
\left[
r+\gamma\max_{a'}Q(s',a')
-Q(s,a)
\right].
\]

Applications:

- Robot navigation.
- Task scheduling.
- Manipulation.
- Energy management.
- Adaptive control.
- Multi-agent coordination.

Challenges:

- Sample inefficiency.
- Unsafe exploration.
- Reward design.
- Sim-to-real transfer.
- Partial observability.
- Nonstationarity.
- Verification.

Safe use often requires:

- Simulation.
- Constrained action space.
- Safety shield.
- Baseline controller.
- Offline evaluation.
- Human oversight.
- Fallback mode.

Executable regression, PCA, Naive Bayes, linear SVM, decision stump, and Q-learning are in `src/machine_learning.py`.

---

# 9. Deep Learning, Explainability, and Embedded Deployment

## 9.1 Deep Neural Networks

A neural network composes affine transformations and nonlinear activations.

\[
h^{(\ell)}
=
\sigma
\left(
W^{(\ell)}h^{(\ell-1)}
+b^{(\ell)}
\right).
\]

Training minimizes a loss through gradient-based optimization and backpropagation.

Key design choices:

- Depth.
- Width.
- Activation.
- Normalization.
- Skip connections.
- Regularization.
- Optimizer.
- Learning-rate schedule.
- Batch size.
- Data augmentation.

---

## 9.2 Convolutional Neural Networks

CNNs exploit local connectivity and weight sharing.

Applications:

- Image classification.
- Object detection.
- Semantic segmentation.
- Optical flow.
- Depth estimation.
- Defect inspection.
- Video understanding.

A convolution layer is characterized by:

- Input channels.
- Output channels.
- Kernel size.
- Stride.
- Padding.
- Dilation.
- Groups.

CNN deployment cost includes:

- Parameter memory.
- Activation memory.
- Multiply-accumulate operations.
- Memory bandwidth.
- Input preprocessing.
- Postprocessing.

---

## 9.3 Recurrent Neural Networks

RNNs model sequential dependence.

A basic recurrent update is

\[
h_t=\phi(W_xx_t+W_hh_{t-1}+b).
\]

LSTM and GRU architectures address long-term gradient problems.

Applications:

- Time-series prediction.
- State estimation.
- Fault diagnosis.
- Video sequence modeling.
- Language-conditioned robotics.
- Adaptive noise estimation.
- Predictive maintenance.

Challenges:

- Sequential execution.
- Long inference latency.
- Hidden-state initialization.
- Distribution shifts.
- Stability when used in feedback.

---

## 9.4 Autoencoders

An autoencoder contains:

```text
Input
→ Encoder
→ Latent Representation
→ Decoder
→ Reconstruction
```

Uses:

- Dimensionality reduction.
- Feature learning.
- Denoising.
- Compression.
- Anomaly detection.

For anomaly detection:

1. Train on healthy data.
2. Compute reconstruction error.
3. Set a threshold.
4. Monitor drift and false alarms.

A low reconstruction error does not guarantee that every anomaly will be detected.

---

## 9.5 Video Processing

Video models must combine spatial and temporal information.

Approaches:

- Framewise CNN plus temporal filter.
- CNN-RNN.
- 3D convolution.
- Two-stream RGB and optical flow.
- Transformer-based video models.
- Tracking-by-detection.
- Event-camera networks.

Deployment constraints include:

- Frame rate.
- Pipeline latency.
- Buffering.
- Camera synchronization.
- GPU memory.
- Thermal limits.
- Batch size of one.
- Preprocessing cost.

---

## 9.6 Diagnostic Anomaly Detection

Inputs may include:

- Vibration spectra.
- Motor current.
- Temperature.
- Acoustic emission.
- Pressure.
- Images.
- Process residuals.

Models:

- Autoencoder.
- One-class classifier.
- Sequence model.
- Forecasting residual model.
- Contrastive representation.
- Hybrid physics-learning model.

Research evaluation should include:

- Detection delay.
- False-alarm rate.
- Fault severity.
- Unseen fault types.
- Operating-condition variation.
- Sensor failure.
- Calibration drift.

---

## 9.7 Explainable and Interpretable AI

Interpretability asks whether a human can understand the model mechanism.

Explainability asks for post-hoc reasons for a prediction.

Methods:

- Feature importance.
- Saliency.
- Integrated gradients.
- SHAP-like attribution.
- Counterfactual explanations.
- Prototype examples.
- Surrogate models.
- Concept activation.
- Attention visualization.

Limitations:

- Explanations can be unstable.
- Saliency may not be causal.
- Different methods can disagree.
- Plausible explanations can be wrong.
- Explanations do not replace validation.

For automation, explanation should support an action:

```text
What evidence triggered the alarm?
Which sensor dominated?
How uncertain is the result?
What operating condition is unusual?
What fallback is recommended?
```

---

## 9.8 Model Compression

### Quantization

Reduce numeric precision:

- FP32 to FP16.
- INT8.
- Mixed precision.
- Lower-bit specialized formats.

Benefits:

- Lower memory.
- Faster arithmetic.
- Lower energy.
- Higher accelerator throughput.

Risks:

- Accuracy loss.
- Overflow.
- Calibration error.
- Sensitivity in recurrent states.
- Unsupported operators.

### Pruning

Remove low-importance weights, channels, filters, or blocks.

Types:

- Unstructured pruning.
- Structured channel pruning.
- Filter pruning.
- Block sparsity.

Structured pruning is often easier to accelerate on real hardware.

### Distillation

A smaller student model learns from a larger teacher.

### Architecture Search

Search for models under constraints:

\[
\min
\quad
\text{error}
+
\lambda_1\text{latency}
+
\lambda_2\text{memory}
+
\lambda_3\text{energy}.
\]

---

## 9.9 Real-Time Embedded Deployment

A deployment study should measure:

- End-to-end latency.
- Worst-case or high-percentile latency.
- Throughput.
- Memory.
- Power.
- Temperature.
- Startup time.
- Accuracy.
- Deadline misses.
- Degradation under load.
- Recovery after faults.

Benchmarking only model inference is insufficient. The complete pipeline includes:

```text
Sensor Read
→ Decode
→ Resize/Normalize
→ Model
→ Postprocess
→ Fusion
→ Decision
→ Communication
```

Executable parameter counting, quantization, pruning, compression, saliency, and integrated gradients are in `src/deep_learning.py`.

---

# 10. Computational Methods in Automation

<p align="center"><img src="../assets/figures/09_computational_methods_map.png" width="940"></p>

## 10.1 Approximation

Approximation replaces a difficult function or dataset with a tractable representation.

Methods:

- Polynomial approximation.
- Interpolation.
- Splines.
- Fourier approximation.
- Basis functions.
- Least squares.
- Reduced-order models.

Important issues:

- Approximation error.
- Conditioning.
- Extrapolation.
- Overfitting.
- Node placement.
- Regularization.

---

## 10.2 Numerical Linear Algebra

Automation algorithms repeatedly solve:

\[
Ax=b.
\]

Methods:

- Gaussian elimination.
- LU.
- QR.
- Cholesky.
- SVD.
- Iterative methods.
- Sparse solvers.

The condition number indicates sensitivity:

\[
\kappa(A)
=
\|A\|\|A^{-1}\|.
\]

Ill-conditioning can make a mathematically valid problem numerically unreliable.

Applications:

- State estimation.
- MPC.
- Least-squares identification.
- Robot kinematics.
- Calibration.
- Finite-element models.
- Sensor fusion.

---

## 10.3 Numerical Calculus

### Differentiation

Central difference:

\[
f'(x)
\approx
\frac{f(x+h)-f(x-h)}{2h}.
\]

Very large \(h\) produces truncation error. Very small \(h\) produces cancellation and roundoff.

### Integration

Methods:

- Rectangle.
- Trapezoidal.
- Simpson.
- Gaussian quadrature.
- Adaptive integration.

### Differential Equations

Methods:

- Euler.
- Heun.
- Runge-Kutta.
- Implicit methods.
- Multistep methods.
- Boundary-value solvers.

Stiff systems require methods whose numerical stability matches the dynamics.

---

## 10.4 Static Optimization

### Unconstrained

\[
\min_x f(x).
\]

Methods:

- Gradient descent.
- Newton.
- BFGS.
- Conjugate gradient.
- Coordinate descent.
- Trust region.
- Derivative-free search.

### Constrained

\[
\min_x f(x)
\]

subject to

\[
h(x)=0,
\qquad
g(x)\le 0.
\]

Methods:

- Projection.
- Penalty.
- Barrier.
- Augmented Lagrangian.
- Sequential quadratic programming.
- Interior point.
- Active set.

KKT conditions combine:

- Stationarity.
- Primal feasibility.
- Dual feasibility.
- Complementarity.

---

## 10.5 Operations Research

Operations research addresses:

- Production planning.
- Transportation.
- Assignment.
- Scheduling.
- Inventory.
- Queueing.
- Network flow.
- Resource allocation.

Linear programming:

\[
\max c^\top x
\]

subject to

\[
Ax\le b,
\qquad
x\ge 0.
\]

Dual variables represent marginal resource values.

---

## 10.6 Continuous and Discrete Optimization

Continuous variables can take real values.

Discrete variables represent:

- On/off decisions.
- Assignments.
- Routes.
- Sequences.
- Selected sensors.
- Selected actuators.
- Task allocations.

Mixed-integer models combine both.

Examples:

- Continuous speed and binary machine state.
- Continuous trajectory and discrete lane choice.
- Continuous controller gains and discrete sensor selection.
- Continuous energy and integer production quantities.

---

## 10.7 Complexity Classes

### P

Decision problems solvable in polynomial time.

### NP

Decision problems whose proposed yes-solutions can be verified in polynomial time.

### NP-Hard

At least as difficult as every problem in NP under polynomial-time reductions.

### NP-Complete

Both in NP and NP-hard.

Examples of difficult combinatorial problems include variants of:

- Traveling salesperson.
- Scheduling.
- Set cover.
- Graph coloring.
- Boolean satisfiability.
- Knapsack.

NP-hardness does not mean every practical instance is impossible. Structure, bounds, decomposition, approximation, and hardware can make many instances manageable.

---

## 10.8 Exact Algorithms

- Enumeration.
- Branch and bound.
- Dynamic programming.
- Cutting planes.
- Branch and cut.
- A*.
- Integer programming.
- Constraint programming.

Exact methods provide certificates when completed.

---

## 10.9 Dynamic Programming

Dynamic programming relies on:

- Optimal substructure.
- Overlapping subproblems.
- State definition.
- Recurrence.
- Boundary conditions.

A finite-horizon control recurrence is

\[
V_k(x)
=
\min_u
\left[
\ell(x,u)
+
V_{k+1}(f(x,u))
\right].
\]

The curse of dimensionality limits direct DP for large continuous state spaces.

---

## 10.10 Approximate Optimization and Constraint Handling

Metaheuristics include:

- Genetic algorithms.
- Particle swarm.
- Differential evolution.
- Simulated annealing.
- Ant colony.
- Tabu search.

Constraint handling methods include:

- Penalties.
- Repair.
- Feasibility rules.
- Decoder representations.
- Barrier-like terms.
- Multi-objective treatment of violation.
- Rejection.

Stochastic optimization should report:

- Multiple random seeds.
- Median.
- Spread.
- Success rate.
- Evaluation count.
- Feasibility rate.
- Best-known gap.
- Runtime.
- Constraint violation.

---

## 10.11 Multi-Criteria Decision Analysis

Real engineering decisions rarely have one objective.

Example criteria:

- Accuracy.
- Latency.
- Cost.
- Power.
- Safety.
- Maintainability.
- Reliability.
- Weight.
- Interpretability.

A weighted-sum model is

\[
S_i
=
\sum_j w_j r_{ij},
\]

where \(r_{ij}\) is normalized performance.

Important concepts:

- Preference structure.
- Benefit and cost criteria.
- Normalization.
- Substitution coefficients.
- Reference alternatives.
- Ideal and anti-ideal points.
- Pareto dominance.
- Sensitivity to weights.
- Ranking stability.

A substitution coefficient such as

\[
\frac{w_1}{w_2}
\]

expresses how much score in criterion 2 compensates for one unit in criterion 1 under the model.

MCDA does not remove judgment. It makes assumptions explicit.

Executable root finding, differentiation, integration, gradient optimization, projected optimization, knapsack DP, Pareto filtering, and weighted MCDA are in `src/computational_methods.py`.

---

# 11. Integrated Research Workflow

<p align="center"><img src="../assets/figures/10_integrated_research_workflow.png" width="940"></p>

A rigorous project can follow this sequence.

## Step 1 — Define the Operational Scenario

Specify:

- Physical environment.
- Mission.
- Users.
- Hazards.
- Disturbances.
- Required autonomy.
- Failure consequences.

## Step 2 — Build the System Model

Define:

- Continuous states.
- Discrete modes.
- Inputs.
- Outputs.
- Disturbances.
- Constraints.
- Uncertainty.
- Communication.

## Step 3 — Design the Sensing Architecture

Choose sensors based on:

- Observability.
- Range.
- Accuracy.
- Failure diversity.
- Timing.
- Environmental robustness.
- Cost.
- Calibration.
- Redundancy.

## Step 4 — Design Estimation and Perception

Specify:

- Preprocessing.
- Detection.
- Tracking.
- Localization.
- Fusion.
- Uncertainty representation.
- Health monitoring.

## Step 5 — Design Planning and Control

Specify:

- Mission planning.
- Path planning.
- Trajectory generation.
- Feedback control.
- Event control.
- Safety supervisor.
- Fallback.

## Step 6 — Partition Computation

Assign functions to:

- Microcontroller.
- CPU.
- FPGA.
- GPU.
- DCS.
- Edge server.
- Cloud.

Evaluate:

- Latency.
- Jitter.
- Bandwidth.
- Memory.
- Power.
- Thermal behavior.
- Availability.

## Step 7 — Optimize

Optimize:

- Controller gains.
- Sensor placement.
- Trajectory.
- Scheduling.
- Network use.
- Model size.
- Hardware allocation.
- Maintenance policy.

## Step 8 — Verify

Use:

- Stability analysis.
- Reachability.
- Timing analysis.
- Constraint checks.
- Fault injection.
- Formal methods where possible.
- Numerical sensitivity.
- Monte Carlo studies.

## Step 9 — Validate

Progress through:

```text
Unit Tests
→ Model Tests
→ Simulation
→ Software-in-the-Loop
→ Processor-in-the-Loop
→ Hardware-in-the-Loop
→ Controlled Experiments
→ Pilot Deployment
```

## Step 10 — Monitor the Lifecycle

After deployment monitor:

- Data drift.
- Sensor drift.
- Model accuracy.
- Deadline misses.
- Resource usage.
- Safety events.
- Cybersecurity events.
- Maintenance.
- Configuration changes.

---

# 12. Integrated Example: Autonomous Industrial Mobile Manipulator

Consider a mobile robot that transports parts, detects obstacles, docks at workstations, and manipulates objects.

## Physical System

- Differential or omnidirectional mobile base.
- Six-axis manipulator.
- Gripper.
- Battery.
- Motors and drives.

## Sensors

- Cameras.
- LiDAR.
- IMU.
- Wheel encoders.
- Joint encoders.
- Force-torque sensor.
- Safety scanner.

## Estimation

- Wheel-IMU localization.
- LiDAR or visual SLAM.
- Object detection.
- Multi-object tracking.
- Manipulator state estimation.
- Battery-health estimate.

## Planning

- Factory-level task assignment.
- Global route planning.
- Local collision avoidance.
- Arm motion planning.
- Grasp planning.
- Discrete task sequence.

## Control

- Base velocity control.
- Path tracking.
- Joint trajectory control.
- Impedance control during contact.
- Emergency stop and safety speed limits.

## Embedded Partition

- FPGA: timestamping and high-rate sensor preprocessing.
- Embedded GPU: vision and point-cloud inference.
- CPU: planning, ROS middleware, fusion.
- Microcontrollers: motor current and safety-related local loops.
- Edge server: fleet coordination and model updates.

## Industrial Integration

- PLC handshake at workstation.
- OPC UA data model.
- MES work orders.
- MQTT telemetry.
- Historian.
- Maintenance dashboard.

## Optimization

- Multi-robot scheduling.
- Battery-aware routing.
- Trajectory time-energy tradeoff.
- Sensor-selection tradeoff.
- Neural-network compression.
- MCDA for hardware selection.

This example demonstrates why the disciplines in this lecture cannot be separated in a real system.

---

# 13. Research-Level Questions

1. Why can a controllable linearization still fail to represent global nonlinear controllability?
2. How does sensor latency affect observability in a digital implementation?
3. When is a distributed-parameter model necessary rather than a lumped approximation?
4. Why can a faster PID loop reduce robustness?
5. How should persistent excitation be designed without violating process constraints?
6. What is the difference between time-optimal and minimum-energy control?
7. Why does LQR require meaningful state and input scaling?
8. How can an intelligent controller be placed inside a verifiable safety architecture?
9. What is the difference between schedulability and average processor utilization?
10. Why can cache and GPU behavior undermine worst-case timing analysis?
11. Which operations should be assigned to FPGA fabric in a heterogeneous SoC?
12. When is DMA slower than direct CPU copying for very small transfers?
13. How do robot singularities affect control and trajectory planning?
14. Why must global and local planners use compatible vehicle models?
15. How should occupancy-map uncertainty influence path planning?
16. What does optical-flow endpoint error fail to capture at the system level?
17. Why does stereo depth uncertainty increase with distance?
18. How can correlated camera and LiDAR errors invalidate naive fusion?
19. Which conditions make radar superior to vision?
20. How should tracking performance be linked to collision-avoidance performance?
21. Why is frame-level random splitting dangerous for autonomous-driving datasets?
22. What is the relation between operational design domain and automation level?
23. Why is TTC alone insufficient for emergency braking?
24. How should multi-modal trajectory prediction influence planning?
25. What should remain local when an industrial system uses cloud analytics?
26. Why can redundant DCS controllers still fail simultaneously?
27. How do event-control deadlocks differ from continuous-control instability?
28. What is the difference between a permissive, interlock, alarm, and trip?
29. Why is model validation more important than training loss?
30. How should PCA components be interpreted in diagnostic systems?
31. When can Naive Bayes outperform a more complex model?
32. What makes reinforcement-learning exploration unsafe in robotics?
33. How can a safety shield constrain a learned policy?
34. Why is a neural explanation not automatically causal?
35. When does INT8 quantization damage recurrent models more than CNNs?
36. Why does unstructured pruning often fail to accelerate general hardware?
37. What is the difference between model latency and end-to-end latency?
38. How does matrix conditioning affect identification and control?
39. Why can a very small finite-difference step produce a worse derivative?
40. What makes an optimization algorithm pseudo-polynomial?
41. How do branch-and-bound bounds influence practical complexity?
42. Why does NP-hardness not imply that useful solutions are unavailable?
43. How should stochastic optimizers be compared fairly?
44. Why can a weighted-sum MCDA model miss nonconvex Pareto alternatives?
45. How should decision rankings be tested against uncertain weights?
46. What evidence is required before deploying a learned anomaly detector?
47. How should timestamp quality be included in sensor-health monitoring?
48. What failure occurs when planner assumptions differ from controller limits?
49. How can a digital twin become misleading?
50. What architecture allows graceful degradation after sensor failure?

---

# 14. Final Synthesis

The disciplines in this lecture form one design chain:

```text
Numerical Models
→ Identification
→ Estimation
→ Perception
→ Planning
→ Optimization
→ Feedback and Event Control
→ Embedded Execution
→ Industrial Integration
→ Validation and Lifecycle Monitoring
```

A strong researcher should be able to move in both directions.

From physics to computation:

```text
Process
→ Model
→ Algorithm
→ Hardware
```

From requirements back to design:

```text
Safety, Timing, and Quality Requirements
→ Architecture
→ Algorithms
→ Sensors and Actuators
→ Physical Implementation
```

The main lesson is:

> Intelligent automation is not obtained by adding artificial intelligence to a machine. It is obtained by engineering a complete cyber-physical system in which models, data, learning, control, computation, communication, safety, and validation remain consistent.

---

# 15. Suggested Foundational References

The repository intentionally presents the lecture as an integrated technical overview. For deeper study, consult authoritative textbooks and official documentation in the following categories:

- Linear and nonlinear systems.
- Optimal control and model predictive control.
- System identification.
- Real-time systems.
- Computer architecture and FPGA design.
- Robot modeling and control.
- Probabilistic robotics.
- Multiple-view geometry and computer vision.
- Sensor fusion and estimation.
- Autonomous-driving architecture.
- Industrial automation, PLC, DCS, and functional safety.
- Machine learning and deep learning.
- Numerical analysis and mathematical optimization.
- Operations research and computational complexity.

The executable laboratory provides a compact starting point, not a replacement for specialized theory, standards, or verified industrial tools.
