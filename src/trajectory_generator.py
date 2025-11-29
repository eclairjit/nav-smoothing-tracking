from dataclasses import dataclass
from typing import List

import numpy as np

from .path_smoother import Point2D


@dataclass
class TrajectoryPoint:
    """Point on time-parameterized trajectory"""

    x: float
    y: float
    t: float
    v: float  # Linear velocity
    theta: float  # Heading angle


class TrajectoryGenerator:
    """
    Generates time-parameterized trajectories with velocity profiles.
    Implements trapezoidal velocity profile for smooth acceleration/deceleration.
    """

    @staticmethod
    def generate_trajectory(
        smooth_path: List[Point2D],
        max_velocity: float = 1.0,
        max_acceleration: float = 0.5,
    ) -> List[TrajectoryPoint]:
        """
        Generate time-parameterized trajectory with trapezoidal velocity profile.

        Args:
            smooth_path: Smoothed path points
            max_velocity: Maximum linear velocity (m/s)
            max_acceleration: Maximum acceleration (m/s^2)

        Returns:
            List of trajectory points with time stamps and velocities
        """
        if len(smooth_path) < 2:
            return []

        trajectory = []
        current_time = 0.0
        current_velocity = 0.0

        # Calculate segment distances
        distances = []
        for i in range(len(smooth_path) - 1):
            dist = smooth_path[i].distance_to(smooth_path[i + 1])
            distances.append(dist)

        total_distance = sum(distances)

        # Calculate acceleration distance (time to reach max velocity)
        accel_distance = (max_velocity**2) / (2 * max_acceleration)

        # Determine if we have cruise phase
        if 2 * accel_distance < total_distance:
            # Trapezoidal profile: accel -> cruise -> decel
            decel_start = total_distance - accel_distance
        else:
            # Triangular profile: accel -> decel (no cruise)
            accel_distance = total_distance / 2
            decel_start = accel_distance

        accumulated_distance = 0.0

        # Add first point
        theta = np.arctan2(
            smooth_path[1].y - smooth_path[0].y, smooth_path[1].x - smooth_path[0].x
        )
        trajectory.append(
            TrajectoryPoint(
                x=smooth_path[0].x,
                y=smooth_path[0].y,
                t=current_time,
                v=current_velocity,
                theta=theta,
            )
        )

        # Generate trajectory points
        for i in range(len(smooth_path) - 1):
            segment_distance = distances[i]

            # Calculate heading angle
            dx = smooth_path[i + 1].x - smooth_path[i].x
            dy = smooth_path[i + 1].y - smooth_path[i].y
            theta = np.arctan2(dy, dx)

            # Determine velocity based on position in profile
            if accumulated_distance < accel_distance:
                # Acceleration phase
                current_velocity = np.sqrt(2 * max_acceleration * accumulated_distance)
                current_velocity = min(current_velocity, max_velocity)
            elif accumulated_distance > decel_start:
                # Deceleration phase
                distance_to_end = total_distance - accumulated_distance
                current_velocity = np.sqrt(2 * max_acceleration * distance_to_end)
                current_velocity = min(current_velocity, max_velocity)
            else:
                # Cruise phase
                current_velocity = max_velocity

            # Ensure minimum velocity
            current_velocity = max(0.1, current_velocity)

            # Calculate time for this segment
            avg_velocity = (trajectory[-1].v + current_velocity) / 2
            avg_velocity = max(0.1, avg_velocity)
            time_segment = segment_distance / avg_velocity
            current_time += time_segment

            accumulated_distance += segment_distance

            trajectory.append(
                TrajectoryPoint(
                    x=smooth_path[i + 1].x,
                    y=smooth_path[i + 1].y,
                    t=current_time,
                    v=current_velocity,
                    theta=theta,
                )
            )

        return trajectory
