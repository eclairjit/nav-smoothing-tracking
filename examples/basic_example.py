"""
Basic example demonstrating the robot navigation system.
"""

import os
import sys

import numpy as np
from matplotlib import pyplot as plt

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from src.controller import DifferentialDriveController, RobotState
from src.path_smoother import PathSmoother, Point2D
from src.simulator import RobotSimulator
from src.trajectory_generator import TrajectoryGenerator

try:
    import matplotlib.pyplot as plt

    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("Matplotlib not available. Visualization disabled.")


def main():
    print("=" * 70)
    print("Basic Robot Navigation Example")
    print("=" * 70)
    print()

    # Step 1: Define waypoints
    print("Step 1: Defining waypoints")
    waypoints = [
        Point2D(0.0, 0.0),
        Point2D(2.0, 1.0),
        Point2D(4.0, 3.0),
        Point2D(6.0, 2.5),
        Point2D(8.0, 4.0),
    ]

    for i, wp in enumerate(waypoints):
        print(f"  Waypoint {i + 1}: ({wp.x:.2f}, {wp.y:.2f})")
    print()

    # Step 2: Smooth path
    print("Step 2: Smoothing path using cubic splines")
    smooth_path = PathSmoother.smooth_path(waypoints, samples_per_segment=20)
    print(f"  Generated {len(smooth_path)} smooth path points")
    print()

    # Step 3: Generate trajectory
    print("Step 3: Generating time-parameterized trajectory")
    trajectory = TrajectoryGenerator.generate_trajectory(
        smooth_path, max_velocity=1.0, max_acceleration=0.5
    )
    print(f"  Generated {len(trajectory)} trajectory points")
    print(f"  Total trajectory time: {trajectory[-1].t:.2f} seconds")
    print()

    # Step 4: Simulate robot
    print("Step 4: Simulating robot motion")
    robot = RobotSimulator(
        initial_state=RobotState(waypoints[0].x, waypoints[0].y, 0.0)
    )

    controller = DifferentialDriveController(
        look_ahead_distance=0.5, k_p=2.0, k_i=0.0, k_d=0.5
    )

    dt = 0.05  # 50ms time step
    current_time = 0.0
    max_steps = 2000

    print(f"  Time step: {dt}s")
    print("  Running simulation...")

    for step in range(max_steps):
        robot_state = robot.get_state()

        # Check if reached goal
        goal = trajectory[-1]
        dx = goal.x - robot_state.x
        dy = goal.y - robot_state.y
        distance_to_goal = np.sqrt(dx**2 + dy**2)

        if distance_to_goal < 0.15:
            print(f"  Goal reached at t={current_time:.2f}s (step {step})")
            break

        # Compute control
        control = controller.compute_control(robot_state, trajectory, dt)

        # Update robot
        robot.update(control, dt)
        current_time += dt

        # Progress update every 100 steps
        if step % 100 == 0 and step > 0:
            print(
                f"    Step {step}: Position ({robot_state.x:.2f}, {robot_state.y:.2f}), "
                f"Distance to goal: {distance_to_goal:.2f}m"
            )

    print()

    # Results
    print("=" * 70)
    print("Simulation Results:")
    print("=" * 70)
    final_state = robot.get_state()
    print(f"Final position: ({final_state.x:.2f}, {final_state.y:.2f})")
    print(f"Final heading: {np.degrees(final_state.theta):.1f}°")
    print(f"Total simulation time: {current_time:.2f}s")
    print(f"Path points tracked: {len(robot.history)}")

    # Calculate path following error
    errors = []
    for state in robot.history:
        min_dist = min(
            np.sqrt((state.x - p.x) ** 2 + (state.y - p.y) ** 2) for p in smooth_path
        )
        errors.append(min_dist)

    print(f"Average path tracking error: {np.mean(errors):.3f}m")
    print(f"Max path tracking error: {np.max(errors):.3f}m")
    print("=" * 70)

    # Visualization
    if MATPLOTLIB_AVAILABLE:
        print("\nGenerating visualization...")
        visualize_results(waypoints, smooth_path, trajectory, robot.history)
        print("Visualization complete. Close the window to exit.")
    else:
        print("\nVisualization skipped (matplotlib not available)")


def visualize_results(waypoints, smooth_path, trajectory, robot_history):
    """Create visualization of the navigation system"""

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

    # Plot 1: Path and trajectory
    ax1.set_title("Path Smoothing and Trajectory", fontsize=14, fontweight="bold")
    ax1.set_xlabel("X (meters)")
    ax1.set_ylabel("Y (meters)")
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect("equal")

    # Waypoints
    wp_x = [wp.x for wp in waypoints]
    wp_y = [wp.y for wp in waypoints]
    ax1.plot(wp_x, wp_y, "ro", markersize=10, label="Waypoints", zorder=5)

    # Smooth path
    smooth_x = [p.x for p in smooth_path]
    smooth_y = [p.y for p in smooth_path]
    ax1.plot(smooth_x, smooth_y, "b-", linewidth=2, label="Smooth Path", alpha=0.7)

    # Robot trajectory
    robot_x = [state.x for state in robot_history]
    robot_y = [state.y for state in robot_history]
    ax1.plot(robot_x, robot_y, "g-", linewidth=2, label="Robot Path", alpha=0.8)

    # Start and end markers
    ax1.plot(
        waypoints[0].x, waypoints[0].y, "g^", markersize=15, label="Start", zorder=6
    )
    ax1.plot(
        waypoints[-1].x, waypoints[-1].y, "r*", markersize=18, label="Goal", zorder=6
    )

    ax1.legend(loc="best")

    # Plot 2: Velocity profile
    ax2.set_title("Velocity Profile", fontsize=14, fontweight="bold")
    ax2.set_xlabel("Time (seconds)")
    ax2.set_ylabel("Velocity (m/s)")
    ax2.grid(True, alpha=0.3)

    times = [p.t for p in trajectory]
    velocities = [p.v for p in trajectory]
    ax2.plot(times, velocities, "b-", linewidth=2, label="Linear Velocity")
    ax2.axhline(y=1.0, color="r", linestyle="--", alpha=0.5, label="Max Velocity")
    ax2.legend(loc="best")
    ax2.set_ylim(bottom=0)

    plt.tight_layout()
    plt.savefig("navigation_results.png", dpi=150, bbox_inches="tight")
    print("Saved visualization to: navigation_results.png")
    plt.show()


if __name__ == "__main__":
    main()
