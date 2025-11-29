from typing import List, Optional

import numpy as np

from .controller import ControlCommand, RobotState


class RobotSimulator:
    """
    Simulates differential drive robot dynamics.
    Uses kinematic model for simulation.
    """

    def __init__(
        self, initial_state: Optional[RobotState] = None, wheel_base: float = 0.3
    ):
        """
        Initialize simulator.

        Args:
            initial_state: Initial robot state
            wheel_base: Distance between wheels (m)
        """
        self.wheel_base = wheel_base

        if initial_state is None:
            self.state = RobotState(x=0.0, y=0.0, theta=0.0)
        else:
            self.state = RobotState(
                x=initial_state.x, y=initial_state.y, theta=initial_state.theta
            )

        self.history: List[RobotState] = [
            RobotState(self.state.x, self.state.y, self.state.theta)
        ]

    def update(self, control: ControlCommand, dt: float) -> RobotState:
        """
        Update robot state using differential drive kinematics.

        Args:
            control: Control command
            dt: Time step (seconds)

        Returns:
            Updated robot state
        """
        # Differential drive kinematics
        # dx/dt = v * cos(θ)
        # dy/dt = v * sin(θ)
        # dθ/dt = ω

        self.state.x += control.linear * np.cos(self.state.theta) * dt
        self.state.y += control.linear * np.sin(self.state.theta) * dt
        self.state.theta += control.angular * dt

        # Normalize theta to [-π, π]
        self.state.theta = np.arctan2(
            np.sin(self.state.theta), np.cos(self.state.theta)
        )

        # Store history
        self.history.append(RobotState(self.state.x, self.state.y, self.state.theta))

        return self.state

    def reset(self, initial_state: RobotState):
        """Reset simulator to initial state"""
        self.state = RobotState(
            x=initial_state.x, y=initial_state.y, theta=initial_state.theta
        )
        self.history = [RobotState(self.state.x, self.state.y, self.state.theta)]

    def get_state(self) -> RobotState:
        """Get current robot state"""
        return RobotState(self.state.x, self.state.y, self.state.theta)
