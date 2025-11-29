from dataclasses import dataclass
from typing import List, Tuple

import numpy as np

from .trajectory_generator import TrajectoryPoint


@dataclass
class RobotState:
    """Robot state representation"""

    x: float
    y: float
    theta: float  # Heading angle in radians


@dataclass
class ControlCommand:
    """Control command for differential drive"""

    linear: float  # Linear velocity (m/s)
    angular: float  # Angular velocity (rad/s)
    left_wheel: float  # Left wheel velocity
    right_wheel: float  # Right wheel velocity


class DifferentialDriveController:
    """
    Trajectory tracking controller for differential drive robots.
    Combines Pure Pursuit for path following with PID for angular control.
    """

    def __init__(
        self,
        look_ahead_distance: float = 0.5,
        k_p: float = 2.0,
        k_i: float = 0.0,
        k_d: float = 0.5,
        wheel_base: float = 0.3,
    ):
        """
        Initialize controller.

        Args:
            look_ahead_distance: Distance to look ahead on path (m)
            k_p: Proportional gain
            k_i: Integral gain
            k_d: Derivative gain
            wheel_base: Distance between wheels (m)
        """
        self.look_ahead_distance = look_ahead_distance
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.wheel_base = wheel_base

        # PID state
        self.prev_error = 0.0
        self.integral_error = 0.0
        self.max_integral = 1.0  # Anti-windup

    def find_look_ahead_point(
        self, robot_state: RobotState, trajectory: List[TrajectoryPoint]
    ) -> Tuple[TrajectoryPoint, int]:
        """
        Find look-ahead point on trajectory using Pure Pursuit algorithm.

        Args:
            robot_state: Current robot state
            trajectory: Trajectory to follow

        Returns:
            Tuple of (look_ahead_point, trajectory_index)
        """
        # Find closest point on trajectory
        min_distance = float("inf")
        closest_index = 0

        for i, point in enumerate(trajectory):
            dx = point.x - robot_state.x
            dy = point.y - robot_state.y
            distance = np.sqrt(dx**2 + dy**2)

            if distance < min_distance:
                min_distance = distance
                closest_index = i

        # Find look-ahead point from closest point
        for i in range(closest_index, len(trajectory)):
            dx = trajectory[i].x - robot_state.x
            dy = trajectory[i].y - robot_state.y
            distance = np.sqrt(dx**2 + dy**2)

            if distance >= self.look_ahead_distance:
                return trajectory[i], i

        # Return last point if look-ahead distance not reached
        return trajectory[-1], len(trajectory) - 1

    def compute_control(
        self, robot_state: RobotState, trajectory: List[TrajectoryPoint], dt: float
    ) -> ControlCommand:
        """
        Compute control command using Pure Pursuit + PID.

        Args:
            robot_state: Current robot state
            trajectory: Trajectory to follow
            dt: Time step (seconds)

        Returns:
            Control command for robot
        """
        # Find look-ahead point
        look_ahead_point, traj_index = self.find_look_ahead_point(
            robot_state, trajectory
        )

        # Calculate angle error
        dx = look_ahead_point.x - robot_state.x
        dy = look_ahead_point.y - robot_state.y
        target_angle = np.arctan2(dy, dx)

        # Normalize angle error to [-π, π]
        angle_error = target_angle - robot_state.theta
        angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))

        # PID control for angular velocity
        self.integral_error += angle_error * dt
        self.integral_error = np.clip(
            self.integral_error, -self.max_integral, self.max_integral
        )

        derivative_error = (angle_error - self.prev_error) / dt if dt > 0 else 0

        angular_velocity = (
            self.k_p * angle_error
            + self.k_i * self.integral_error
            + self.k_d * derivative_error
        )

        self.prev_error = angle_error

        # Linear velocity (reduce speed when turning)
        target_velocity = trajectory[traj_index].v

        # Slow down when turning sharply
        turn_factor = 1.0 - min(abs(angle_error) / np.pi, 1.0)
        linear_velocity = target_velocity * (0.3 + 0.7 * turn_factor)

        # Convert to differential drive wheel velocities
        left_wheel = linear_velocity - (angular_velocity * self.wheel_base) / 2
        right_wheel = linear_velocity + (angular_velocity * self.wheel_base) / 2

        return ControlCommand(
            linear=linear_velocity,
            angular=angular_velocity,
            left_wheel=left_wheel,
            right_wheel=right_wheel,
        )

    def reset(self):
        """Reset controller state"""
        self.prev_error = 0.0
        self.integral_error = 0.0
