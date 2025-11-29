"""
Advanced example with complex scenarios and analysis.
"""

import os
import sys

from matplotlib import pyplot as plt

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import numpy as np

from src.controller import DifferentialDriveController, RobotState
from src.path_smoother import PathSmoother, Point2D
from src.simulator import RobotSimulator
from src.trajectory_generator import TrajectoryGenerator

try:
    import matplotlib.pyplot as plt

    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False


def scenario_figure_eight():
    """Simulate figure-eight path"""
    print("\n" + "=" * 70)
    print("Scenario: Figure-Eight Path")
    print("=" * 70)

    # Create figure-eight waypoints
    t = np.linspace(0, 2 * np.pi, 16)
    waypoints = [Point2D(4 * np.sin(ti), 2 * np.sin(2 * ti)) for ti in t]

    return run_scenario(
        waypoints, "Figure Eight", max_velocity=1.5, max_acceleration=0.8
    )


def scenario_sharp_turns():
    """Simulate path with sharp turns"""
    print("\n" + "=" * 70)
    print("Scenario: Sharp Turns")
    print("=" * 70)

    waypoints = [
        Point2D(0, 0),
        Point2D(3, 0),
        Point2D(3, 3),
        Point2D(0, 3),
        Point2D(0, 6),
        Point2D(3, 6),
    ]

    return run_scenario(
        waypoints, "Sharp Turns", max_velocity=0.8, max_acceleration=0.4
    )


def scenario_high_speed():
    """Simulate high-speed trajectory"""
    print("\n" + "=" * 70)
    print("Scenario: High-Speed Trajectory")
    print("=" * 70)

    waypoints = [
        Point2D(0, 0),
        Point2D(5, 2),
        Point2D(10, 1),
        Point2D(15, 3),
        Point2D(20, 2),
    ]

    return run_scenario(waypoints, "High Speed", max_velocity=3.0, max_acceleration=1.5)


def run_scenario(waypoints, name, max_velocity=1.0, max_acceleration=0.5):
    """Run a navigation scenario"""

    # Smooth path
    smooth_path = PathSmoother.smooth_path(waypoints, samples_per_segment=25)

    # Generate trajectory
    trajectory = TrajectoryGenerator.generate_trajectory(
        smooth_path, max_velocity=max_velocity, max_acceleration=max_acceleration
    )

    # Simulate
    robot = RobotSimulator(
        initial_state=RobotState(waypoints[0].x, waypoints[0].y, 0.0)
    )

    controller = DifferentialDriveController(
        look_ahead_distance=0.6, k_p=2.5, k_i=0.0, k_d=0.6
    )

    dt = 0.02  # 50Hz control
    max_steps = 5000

    for step in range(max_steps):
        robot_state = robot.get_state()

        # Check goal
        goal = trajectory[-1]
        dx = goal.x - robot_state.x
        dy = goal.y - robot_state.y
        if np.sqrt(dx**2 + dy**2) < 0.2:
            print(f"  Goal reached at step {step}")
            break

        control = controller.compute_control(robot_state, trajectory, dt)
        robot.update(control, dt)

    # Analysis
    errors = []
    for state in robot.history:
        min_dist = min(
            np.sqrt((state.x - p.x) ** 2 + (state.y - p.y) ** 2) for p in smooth_path
        )
        errors.append(min_dist)

    print(f"  Waypoints: {len(waypoints)}")
    print(f"  Trajectory time: {trajectory[-1].t:.2f}s")
    print(f"  Avg tracking error: {np.mean(errors):.3f}m")
    print(f"  Max tracking error: {np.max(errors):.3f}m")

    return {
        "name": name,
        "waypoints": waypoints,
        "smooth_path": smooth_path,
        "trajectory": trajectory,
        "robot_history": robot.history,
        "errors": errors,
    }


def compare_scenarios():
    """Run and compare multiple scenarios"""
    print("=" * 70)
    print("Advanced Navigation Scenarios Comparison")
    print("=" * 70)

    scenarios = [scenario_figure_eight(), scenario_sharp_turns(), scenario_high_speed()]

    if MATPLOTLIB_AVAILABLE:
        visualize_comparison(scenarios)


def visualize_comparison(scenarios):
    """Visualize multiple scenarios for comparison"""

    n_scenarios = len(scenarios)
    fig, axes = plt.subplots(2, n_scenarios, figsize=(6 * n_scenarios, 10))

    for idx, scenario in enumerate(scenarios):
        # Path plot
        ax = axes[0, idx] if n_scenarios > 1 else axes[0]
        ax.set_title(f"{scenario['name']}", fontsize=12, fontweight="bold")
        ax.set_xlabel("X (meters)")
        ax.set_ylabel("Y (meters)")
        ax.grid(True, alpha=0.3)
        ax.set_aspect("equal")

        # Waypoints
        wp_x = [wp.x for wp in scenario["waypoints"]]
        wp_y = [wp.y for wp in scenario["waypoints"]]
        ax.plot(wp_x, wp_y, "ro", markersize=8, label="Waypoints")

        # Smooth path
        smooth_x = [p.x for p in scenario["smooth_path"]]
        smooth_y = [p.y for p in scenario["smooth_path"]]
        ax.plot(smooth_x, smooth_y, "b-", linewidth=1.5, label="Smooth Path", alpha=0.6)

        # Robot trajectory
        robot_x = [state.x for state in scenario["robot_history"]]
        robot_y = [state.y for state in scenario["robot_history"]]
        ax.plot(robot_x, robot_y, "g-", linewidth=2, label="Robot Path")

        ax.legend(loc="best", fontsize=8)

        # Error plot
        ax2 = axes[1, idx] if n_scenarios > 1 else axes[1]
        ax2.set_title(f"Tracking Error - {scenario['name']}", fontsize=12)
        ax2.set_xlabel("Sample")
        ax2.set_ylabel("Error (meters)")
        ax2.grid(True, alpha=0.3)

        ax2.plot(scenario["errors"], "r-", linewidth=1.5)
        ax2.axhline(
            y=np.mean(scenario["errors"]),
            color="b",
            linestyle="--",
            label=f"Mean: {np.mean(scenario['errors']):.3f}m",
        )
        ax2.legend(loc="best", fontsize=8)

    plt.tight_layout()
    plt.savefig("scenarios_comparison.png", dpi=150, bbox_inches="tight")
    print("\nSaved comparison to: scenarios_comparison.png")
    plt.show()


if __name__ == "__main__":
    compare_scenarios()
