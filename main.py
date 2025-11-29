import numpy as np

from src.controller import DifferentialDriveController, RobotState
from src.path_smoother import PathSmoother, Point2D
from src.simulator import RobotSimulator
from src.trajectory_generator import TrajectoryGenerator


def main():
    """Main execution function"""
    print("=" * 70)
    print("Robot Path Smoothing & Trajectory Control System")
    print("=" * 70)
    print()

    # Define waypoints
    waypoints = [
        Point2D(0.0, 0.0),
        Point2D(2.0, 1.0),
        Point2D(4.0, 3.0),
        Point2D(6.0, 2.5),
        Point2D(8.0, 4.0),
    ]

    print(f"Waypoints: {len(waypoints)}")
    for i, wp in enumerate(waypoints):
        print(f"  {i + 1}. ({wp.x:.2f}, {wp.y:.2f})")
    print()

    # Step 1: Path Smoothing
    print("Step 1: Path Smoothing (Cubic Spline)")
    smooth_path = PathSmoother.smooth_path(waypoints, samples_per_segment=20)
    print(f"  Smooth path points: {len(smooth_path)}")
    print()

    # Step 2: Trajectory Generation
    print("Step 2: Trajectory Generation (Trapezoidal Profile)")
    trajectory = TrajectoryGenerator.generate_trajectory(
        smooth_path, max_velocity=1.0, max_acceleration=0.5
    )
    print(f"  Trajectory points: {len(trajectory)}")
    print(f"  Total time: {trajectory[-1].t:.2f} seconds")
    print()

    # Step 3: Simulation
    print("Step 3: Robot Simulation")
    robot = RobotSimulator(
        initial_state=RobotState(waypoints[0].x, waypoints[0].y, 0.0)
    )
    controller = DifferentialDriveController(
        look_ahead_distance=0.5, k_p=2.0, k_i=0.0, k_d=0.5
    )

    dt = 0.05  # 50ms time step
    current_time = 0.0
    max_time = trajectory[-1].t + 5.0

    print(f"  Running simulation (dt={dt}s)...")

    while current_time < max_time:
        robot_state = robot.get_state()

        # Check if reached goal
        goal = trajectory[-1]
        dx = goal.x - robot_state.x
        dy = goal.y - robot_state.y
        distance_to_goal = np.sqrt(dx**2 + dy**2)

        if distance_to_goal < 0.1:
            print(f"  Goal reached at t={current_time:.2f}s")
            break

        # Compute control
        control = controller.compute_control(robot_state, trajectory, dt)

        # Update robot
        robot.update(control, dt)
        current_time += dt

    print()
    print("=" * 70)
    print("Simulation Complete!")
    print(f"Final position: ({robot.state.x:.2f}, {robot.state.y:.2f})")
    print(f"Final heading: {np.degrees(robot.state.theta):.1f}°")
    print(f"Path tracking points: {len(robot.history)}")
    print("=" * 70)


if __name__ == "__main__":
    main()
