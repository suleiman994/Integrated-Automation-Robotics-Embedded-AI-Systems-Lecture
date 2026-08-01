"""Generate ten integrated lecture figures."""

from pathlib import Path
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "assets/figures"


def box(ax, x, y, width, height, text, fontsize=9):
    ax.add_patch(
        plt.Rectangle(
            (x, y),
            width,
            height,
            fill=False,
            linewidth=1.8,
        )
    )
    ax.text(
        x + width / 2,
        y + height / 2,
        text,
        ha="center",
        va="center",
        fontsize=fontsize,
    )


def arrow(ax, x1, y1, x2, y2):
    ax.annotate(
        "",
        xy=(x2, y2),
        xytext=(x1, y1),
        arrowprops={"arrowstyle": "->", "linewidth": 1.5},
    )


def horizontal_map(filename, title, labels, footer):
    fig, ax = plt.subplots(figsize=(15, 5.5))
    ax.axis("off")
    width = 1.65
    gap = 0.38
    start = 0.25
    y = 2.45
    positions = []
    for index, label in enumerate(labels):
        x = start + index * (width + gap)
        positions.append(x)
        box(ax, x, y, width, 0.95, label, 8.6)
        if index > 0:
            arrow(ax, positions[index - 1] + width, y + 0.475, x, y + 0.475)
    ax.text(
        (positions[0] + positions[-1] + width) / 2,
        4.35,
        title,
        ha="center",
        fontsize=18,
    )
    ax.text(
        (positions[0] + positions[-1] + width) / 2,
        1.15,
        footer,
        ha="center",
        fontsize=10,
    )
    ax.set_xlim(0, positions[-1] + width + 0.3)
    ax.set_ylim(0.6, 4.8)
    fig.tight_layout()
    fig.savefig(OUT / filename, dpi=190)
    plt.close(fig)


def integrated_stack():
    horizontal_map(
        "01_integrated_cyber_physical_stack.png",
        "Integrated Cyber-Physical System",
        [
            "Physical\nProcess",
            "Sensors and\nMeasurement",
            "Perception and\nEstimation",
            "Planning and\nOptimization",
            "Feedback and\nEvent Control",
            "Actuators and\nPower",
        ],
        "Embedded computing, communication, safety, cybersecurity, and lifecycle management surround the complete loop.",
    )


def control_map():
    horizontal_map(
        "02_control_systems_map.png",
        "Control Problems and Dynamic Systems",
        [
            "Physical\nModel",
            "Identification",
            "Controllability and\nObservability",
            "PID / Nonlinear /\nOptimal Control",
            "Digital Real-Time\nImplementation",
            "Hierarchical and\nEvent Control",
        ],
        "A controller is valid only when the model, measurements, timing, constraints, and implementation are consistent.",
    )


def embedded_map():
    horizontal_map(
        "03_embedded_computing_map.png",
        "Embedded and Heterogeneous Computing",
        [
            "Sensors and\nI/O",
            "Microcontroller /\nCPU",
            "FPGA /\nZynq",
            "GPU /\nAccelerator",
            "ASIC /\nASSP",
            "PCB, Power,\nand Real Time",
        ],
        "Partition functions according to latency, parallelism, predictability, power, data movement, and verification cost.",
    )


def robotics_map():
    horizontal_map(
        "04_robotics_pipeline.png",
        "Robotics Pipeline",
        [
            "Robot\nConfiguration",
            "Kinematics and\nDynamics",
            "Environment\nModel",
            "Path and\nTrajectory",
            "Feedback\nControl",
            "Safety and\nTask Execution",
        ],
        "Planning must generate references that respect geometry, dynamics, actuator limits, uncertainty, and collision constraints.",
    )


def perception_map():
    horizontal_map(
        "05_perception_and_sensor_fusion.png",
        "Perception and Multi-Domain Sensor Fusion",
        [
            "Camera / Radar /\nLiDAR",
            "Calibration and\nSynchronization",
            "Preprocessing and\nFeatures",
            "Detection and\nTracking",
            "Probabilistic\nFusion",
            "World Model and\nQuality Metrics",
        ],
        "Perception quality must be evaluated at data, algorithm, and end-to-end system levels.",
    )


def vehicle_map():
    horizontal_map(
        "06_autonomous_vehicle_stack.png",
        "Autonomous Vehicle Stack",
        [
            "Sensor\nAcquisition",
            "Localization and\nPerception",
            "Prediction",
            "Behavior\nPlanning",
            "Motion\nPlanning",
            "Vehicle\nControl",
        ],
        "The operational design domain, fallback strategy, and safety supervisor define the practical meaning of autonomy.",
    )


def industrial_map():
    horizontal_map(
        "07_industrial_automation_architecture.png",
        "Industrial Automation Architecture",
        [
            "Physical\nProcess",
            "Field Devices",
            "PLC / DCS /\nSafety Control",
            "HMI / SCADA /\nHistorian",
            "MES",
            "ERP / IIoT /\nIndustry 4.0",
        ],
        "Fast control remains local while supervision, production management, and enterprise systems operate at slower layers.",
    )


def learning_map():
    horizontal_map(
        "08_machine_learning_and_deep_learning_lifecycle.png",
        "Machine Learning and Deep Learning Lifecycle",
        [
            "Problem and\nData",
            "Split and\nPreprocess",
            "Baseline and\nModel",
            "Validation and\nExplainability",
            "Compression and\nDeployment",
            "Monitoring and\nFallback",
        ],
        "Training accuracy is only one part of a trustworthy embedded learning system.",
    )


def computational_map():
    horizontal_map(
        "09_computational_methods_map.png",
        "Computational Methods in Automation",
        [
            "Approximation and\nLinear Algebra",
            "Numerical\nCalculus",
            "Static\nOptimization",
            "Operations\nResearch",
            "Discrete and\nNP-Hard Problems",
            "MCDA and\nDecision Analysis",
        ],
        "Algorithm selection depends on structure, conditioning, constraints, complexity, guarantees, and available computation.",
    )


def research_workflow():
    horizontal_map(
        "10_integrated_research_workflow.png",
        "Integrated Research and Validation Workflow",
        [
            "Requirements and\nHazards",
            "Model and\nArchitecture",
            "Algorithms and\nPartitioning",
            "Verification and\nSimulation",
            "HIL and\nExperiments",
            "Deployment and\nLifecycle Monitoring",
        ],
        "Evidence should progress from equations and unit tests to hardware-in-the-loop, controlled experiments, and monitored deployment.",
    )


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    integrated_stack()
    control_map()
    embedded_map()
    robotics_map()
    perception_map()
    vehicle_map()
    industrial_map()
    learning_map()
    computational_map()
    research_workflow()
    print("Generated ten figures.")


if __name__ == "__main__":
    main()
