from dataclasses import dataclass
from typing import List

import numpy as np


@dataclass
class Point2D:
    """2D point representation"""

    x: float
    y: float

    def __iter__(self):
        return iter((self.x, self.y))

    def distance_to(self, other: "Point2D") -> float:
        """Calculate Euclidean distance to another point"""
        return np.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)


class CubicSpline:
    """
    Cubic spline interpolation implementation from scratch.
    Uses natural boundary conditions (second derivative = 0 at endpoints).
    """

    def __init__(self, x: np.ndarray, y: np.ndarray):
        """
        Initialize cubic spline with data points.

        Args:
            x: Independent variable (parameter)
            y: Dependent variable (coordinate)
        """
        if len(x) != len(y):
            raise ValueError("x and y must have the same length")
        if len(x) < 2:
            raise ValueError("Need at least 2 points for interpolation")

        self.x = np.array(x, dtype=float)
        self.y = np.array(y, dtype=float)
        self.n = len(x)

        # Coefficients for cubic polynomials: S_i(x) = a_i + b_i*dx + c_i*dx^2 + d_i*dx^3
        self.a = np.copy(self.y)
        self.b = np.zeros(self.n - 1)
        self.c = np.zeros(self.n)
        self.d = np.zeros(self.n - 1)

        self._compute_coefficients()

    def _compute_coefficients(self):
        """Compute spline coefficients using tridiagonal matrix algorithm"""
        n = self.n
        h = np.diff(self.x)  # h[i] = x[i+1] - x[i]

        # Build tridiagonal system for c coefficients
        # A * c = b, where A is tridiagonal
        A = np.zeros((n, n))
        b = np.zeros(n)

        # Natural boundary conditions: c[0] = c[n-1] = 0
        A[0, 0] = 1.0
        A[n - 1, n - 1] = 1.0

        # Interior equations
        for i in range(1, n - 1):
            A[i, i - 1] = h[i - 1]
            A[i, i] = 2.0 * (h[i - 1] + h[i])
            A[i, i + 1] = h[i]

            b[i] = 3.0 * (
                (self.a[i + 1] - self.a[i]) / h[i]
                - (self.a[i] - self.a[i - 1]) / h[i - 1]
            )

        # Solve for c coefficients
        self.c = np.linalg.solve(A, b)

        # Compute b and d coefficients
        for i in range(n - 1):
            self.b[i] = (self.a[i + 1] - self.a[i]) / h[i] - h[i] * (
                2.0 * self.c[i] + self.c[i + 1]
            ) / 3.0
            self.d[i] = (self.c[i + 1] - self.c[i]) / (3.0 * h[i])

    def interpolate(self, t: float) -> float:
        """
        Interpolate value at parameter t.

        Args:
            t: Parameter value

        Returns:
            Interpolated value
        """
        # Find the correct interval
        if t <= self.x[0]:
            return self.y[0]
        if t >= self.x[-1]:
            return self.y[-1]

        # Binary search for efficiency
        i = np.searchsorted(self.x, t) - 1
        i = max(0, min(i, self.n - 2))

        # Evaluate cubic polynomial
        dx = t - self.x[i]
        return self.a[i] + self.b[i] * dx + self.c[i] * dx**2 + self.d[i] * dx**3

    def interpolate_derivative(self, t: float) -> float:
        """
        Compute first derivative at parameter t.

        Args:
            t: Parameter value

        Returns:
            First derivative value
        """
        if t <= self.x[0]:
            i = 0
        elif t >= self.x[-1]:
            i = self.n - 2
        else:
            i = np.searchsorted(self.x, t) - 1
            i = max(0, min(i, self.n - 2))

        dx = t - self.x[i]
        return self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx**2


class PathSmoother:
    """
    Path smoothing using cubic spline interpolation.
    Generates smooth, continuous paths from discrete waypoints.
    """

    @staticmethod
    def smooth_path(
        waypoints: List[Point2D], samples_per_segment: int = 20
    ) -> List[Point2D]:
        """
        Generate smooth path through waypoints using cubic splines.

        Args:
            waypoints: List of 2D waypoints
            samples_per_segment: Number of samples per waypoint segment

        Returns:
            List of smoothed path points
        """
        if len(waypoints) < 2:
            return waypoints

        # Extract coordinates
        x_coords = np.array([wp.x for wp in waypoints])
        y_coords = np.array([wp.y for wp in waypoints])

        # Create arc-length parameterization
        t = [0.0]
        for i in range(1, len(waypoints)):
            dist = waypoints[i].distance_to(waypoints[i - 1])
            t.append(t[-1] + dist)
        t = np.array(t)

        # Create splines for x and y coordinates
        spline_x = CubicSpline(t, x_coords)
        spline_y = CubicSpline(t, y_coords)

        # Generate smooth path
        total_samples = (len(waypoints) - 1) * samples_per_segment + 1
        t_samples = np.linspace(t[0], t[-1], total_samples)

        smooth_path = []
        for t_val in t_samples:
            x_val = spline_x.interpolate(t_val)
            y_val = spline_y.interpolate(t_val)
            smooth_path.append(Point2D(x_val, y_val))

        return smooth_path
