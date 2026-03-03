# Robot Path Smoothing & Trajectory Control System

A fully vibe-coded implementation of path smoothing, trajectory generation, and control algorithms for differential drive robots, built entirely from scratch without external robotics libraries.

## 🎯 Features

- **Custom Cubic Spline Implementation**: From-scratch path smoothing using natural cubic splines
- **Trapezoidal Velocity Profile**: Smooth acceleration/deceleration trajectory generation
- **Pure Pursuit + PID Control**: Robust trajectory tracking controller
- **Differential Drive Kinematics**: Accurate robot physics simulation
- **Well-Documented**: Extensive inline documentation and architectural notes

## 📁 Project Structure

```
robot_navigation/
├── src/
│   ├── __init__.py
│   ├── path_smoother.py          # Cubic spline path smoothing
│   ├── trajectory_generator.py   # Time-parameterized trajectories
│   ├── controller.py              # Pure Pursuit + PID controller
│   ├── simulator.py               # Robot physics simulation
│   └── visualizer.py              # Matplotlib visualization
├── examples/
│   ├── basic_example.py           # Simple usage example
│   └── advanced_example.py        # Complex scenarios
├── main.py                        # Main execution script
├── requirements.txt               # Python dependencies
├── setup.py                       # Package setup
├── README.md                      # This file
└── .gitignore
```

## 🚀 Quick Start

### Prerequisites

- Python 3.8 or higher
- pip package manager

### Installation

1. **Clone or download the repository**

```bash
git clone github.com/eclairjit/nav-smoothing-tracking
cd nav-smoothing-tracking
```

2. **Create virtual environment (recommended)**

```bash
# On Mac/Linux
python3 -m venv venv
source venv/bin/activate

# On Windows
python -m venv venv
venv\Scripts\activate
```

3. **Install dependencies**

```bash
pip install -r requirements.txt
```

### Running the System

**Basic simulation with visualization:**

```bash
python main.py
```

**Run with custom parameters:**

```bash
python main.py --max-velocity 1.5 --look-ahead 0.8
```

## 📊 Algorithm Details

### 1. Path Smoothing (Cubic Spline)

**Algorithm**: Natural Cubic Spline Interpolation

**Implementation Highlights**:

- Solves tridiagonal system using Thomas algorithm
- Natural boundary conditions (zero second derivative at endpoints)
- Arc-length parameterization for uniform sampling
- O(n) time complexity after coefficient computation

**Mathematical Foundation**:

For each segment i, the spline S_i(x) is defined as:
$$S_i(x) = a_i + b_i(x - x_i) + c_i(x - x_i)² + d_i(x - x_i)³$$

where coefficients are computed to ensure:

- C² continuity at all interior points
- Natural boundary conditions at endpoints

**Key Files**: `src/path_smoother.py`

### 2. Trajectory Generation

**Algorithm**: Trapezoidal Velocity Profile

**Phases**:

1. **Acceleration**: $$v(t) = a\*t$$ (until v_max or midpoint)
2. **Cruise**: $$v(t) = v_max$$ (if path is long enough)
3. **Deceleration**: $$v(t) = v_max - a*t$$ (until stop)

**Features**:

- Time-optimal trajectory generation
- Respects velocity and acceleration constraints
- Smooth transitions between phases
- Arc-length based parameterization

**Key Files**: `src/trajectory_generator.py`

### 3. Trajectory Tracking Controller

**Algorithm**: Pure Pursuit + PID

**Control Strategy**:

1. Pure Pursuit: Find look-ahead point on trajectory
2. Calculate angular error: $$θ_error = atan2(Δy, Δx) - θ_robot$$
3. PID Control: $$ω = Kp*e + Ki*∫e + Kd\*de/dt$$
4. Speed adaptation: v = $$v_target * (1 - |θ_error|/π)$$
5. Differential drive: $$v_L = v - ω*L/2, v_R = v + ω*L/2$$

**Tuning Parameters**:

- `look_ahead_distance`: 0.3-1.0m (larger = smoother, smaller = more accurate)
- `k_p`: 1.5-3.0 (proportional gain)
- `k_d`: 0.3-1.0 (derivative gain for damping)
- `k_i`: 0.0-0.1 (integral gain, usually small to avoid windup)

**Key Files**: `src/controller.py`

### 4. Robot Simulation

**Model**: Differential Drive Kinematics

**State Update Equations**:

$$x(t+1) = x(t) + v*cos(θ)*dt$$
$$y(t+1) = y(t) + v*sin(θ)*dt$$
$$θ(t+1) = θ(t) + ω*dt$$

**Key Files**: `src/simulator.py`

## 🎮 Usage Examples

### Basic Usage

```python
from src.path_smoother import PathSmoother, Point2D
from src.trajectory_generator import TrajectoryGenerator
from src.controller import DifferentialDriveController, RobotState
from src.simulator import RobotSimulator

# Define waypoints
waypoints = [
    Point2D(0.0, 0.0),
    Point2D(2.0, 1.0),
    Point2D(4.0, 3.0),
    Point2D(6.0, 2.5)
]

# Smooth path
smooth_path = PathSmoother.smooth_path(waypoints, samples_per_segment=20)

# Generate trajectory
trajectory = TrajectoryGenerator.generate_trajectory(
    smooth_path,
    max_velocity=1.0,
    max_acceleration=0.5
)

# Create robot and controller
robot = RobotSimulator(initial_state=RobotState(0, 0, 0))
controller = DifferentialDriveController(look_ahead_distance=0.5)

# Simulation loop
dt = 0.05
for _ in range(1000):
    state = robot.get_state()
    control = controller.compute_control(state, trajectory, dt)
    robot.update(control, dt)
```

### Advanced: Custom Waypoint Following

```python
# Load waypoints from file
waypoints = load_waypoints_from_csv("path.csv")

# High-speed trajectory
trajectory = TrajectoryGenerator.generate_trajectory(
    smooth_path,
    max_velocity=2.0,      # 2 m/s
    max_acceleration=1.0   # 1 m/s²
)

# Aggressive controller tuning
controller = DifferentialDriveController(
    look_ahead_distance=0.8,
    k_p=3.0,
    k_d=0.8,
    wheel_base=0.35
)
```

## 🏗️ Architecture & Design Decisions

### Modular Design

**Separation of Concerns**:

- **Path Smoother**: Pure mathematical interpolation (no dependencies)
- **Trajectory Generator**: Kinematic planning (depends only on geometry)
- **Controller**: Control logic (no simulation coupling)
- **Simulator**: Physics engine (independent of control)

**Benefits**:

- Easy to test each component independently
- Simple to swap implementations
- Clear interfaces between modules
- Facilitates parallel development

### Design Patterns Used

1. **Strategy Pattern**: Different controllers can be swapped
2. **Observer Pattern**: State history tracking in simulator
3. **Factory Pattern**: Trajectory generator creates various profiles
4. **Dependency Injection**: Controllers receive dependencies via constructor

### Error Handling

- Input validation at all public interfaces
- Graceful degradation for edge cases
- Comprehensive error messages with context
- Exception hierarchy for different error types

## 🤖 Extending to Real Robot

### Hardware Interface Layer

```python
class RobotHardwareInterface:
    """Abstract interface for real robot hardware"""

    def get_odometry(self) -> RobotState:
        """Read current robot state from sensors"""
        pass

    def send_velocity_command(self, control: ControlCommand):
        """Send velocity commands to motor controllers"""
        pass

    def emergency_stop(self):
        """Emergency stop command"""
        pass
```

### ROS 2 Integration

```python
# ros2_node.py
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class TrajectoryFollowerNode(Node):
    def __init__(self):
        super().__init__('trajectory_follower')

        # Create controller
        self.controller = DifferentialDriveController()

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # Timer for control loop
        self.create_timer(0.05, self.control_loop)

    def control_loop(self):
        control = self.controller.compute_control(
            self.robot_state,
            self.trajectory,
            0.05
        )
        self.publish_cmd_vel(control)
```

### Sensor Integration

**Required Sensors**:

1. **Odometry**: Wheel encoders or IMU + wheel encoders
2. **Localization**: LIDAR, cameras, or GPS for absolute positioning
3. **Safety**: Bumpers, cliff sensors, emergency stop button

**Sensor Fusion**:

```python
from filterpy.kalman import ExtendedKalmanFilter

class StateEstimator:
    """Fuses multiple sensor readings for robust state estimation"""

    def __init__(self):
        self.ekf = ExtendedKalmanFilter(dim_x=3, dim_z=2)
        # Configure EKF for robot state estimation

    def update(self, odom, lidar_pose):
        """Fuse odometry and LIDAR measurements"""
        self.ekf.predict()
        self.ekf.update([lidar_pose.x, lidar_pose.y])
        return self.ekf.x  # Estimated state
```

### Safety Considerations

```python
class SafetyMonitor:
    """Monitors robot state and environment for safety"""

    def check_velocity_limits(self, control: ControlCommand) -> bool:
        """Ensure velocity commands are within safe limits"""
        return (abs(control.linear) <= self.max_linear and
                abs(control.angular) <= self.max_angular)

    def check_trajectory_collision(self, trajectory, obstacles) -> bool:
        """Check if trajectory collides with obstacles"""
        # Implement collision checking
        pass

    def emergency_stop_conditions(self) -> bool:
        """Check if emergency stop is needed"""
        return (self.battery_low or
                self.obstacle_too_close or
                self.communication_lost)
```

## 🛡️ Obstacle Avoidance Extension (Extra Credit)

### Dynamic Window Approach (DWA)

```python
class DynamicWindowApproach:
    """
    Real-time obstacle avoidance using Dynamic Window Approach.
    Evaluates possible velocity commands and selects the best one.
    """

    def __init__(self, max_vel, max_accel):
        self.max_vel = max_vel
        self.max_accel = max_accel

    def compute_dynamic_window(self, current_vel, dt):
        """Compute set of reachable velocities"""
        v_min = max(0, current_vel - self.max_accel * dt)
        v_max = min(self.max_vel, current_vel + self.max_accel * dt)
        return (v_min, v_max)

    def evaluate_trajectory(self, vel, obstacles, target):
        """
        Score a candidate velocity based on:
        - Distance to obstacles (clearance)
        - Distance to target (goal alignment)
        - Current velocity (prefer forward motion)
        """
        clearance_score = self.compute_clearance(vel, obstacles)
        goal_score = self.compute_goal_alignment(vel, target)
        velocity_score = vel / self.max_vel

        return (self.w_clearance * clearance_score +
                self.w_goal * goal_score +
                self.w_velocity * velocity_score)

    def select_best_velocity(self, current_vel, obstacles, target, dt):
        """Select optimal velocity command"""
        v_min, v_max = self.compute_dynamic_window(current_vel, dt)

        best_score = -float('inf')
        best_vel = current_vel

        # Sample velocity space
        for v in np.linspace(v_min, v_max, 20):
            for w in np.linspace(-self.max_angular, self.max_angular, 20):
                score = self.evaluate_trajectory((v, w), obstacles, target)
                if score > best_score:
                    best_score = score
                    best_vel = (v, w)

        return best_vel
```

### Artificial Potential Fields

```python
class PotentialFieldPlanner:
    """
    Obstacle avoidance using artificial potential fields.
    Attractive force to goal, repulsive force from obstacles.
    """

    def __init__(self, k_att=1.0, k_rep=10.0, influence_distance=2.0):
        self.k_att = k_att  # Attractive gain
        self.k_rep = k_rep  # Repulsive gain
        self.d_influence = influence_distance

    def attractive_force(self, robot_pos, goal_pos):
        """Compute attractive force toward goal"""
        diff = goal_pos - robot_pos
        distance = np.linalg.norm(diff)
        if distance < 0.01:
            return np.zeros(2)
        return self.k_att * diff / distance

    def repulsive_force(self, robot_pos, obstacle_pos):
        """Compute repulsive force from obstacle"""
        diff = robot_pos - obstacle_pos
        distance = np.linalg.norm(diff)

        if distance > self.d_influence:
            return np.zeros(2)

        if distance < 0.01:
            distance = 0.01  # Avoid division by zero

        magnitude = self.k_rep * (1/distance - 1/self.d_influence) / (distance**2)
        return magnitude * diff / distance

    def compute_velocity(self, robot_state, goal, obstacles):
        """Compute velocity command using potential fields"""
        robot_pos = np.array([robot_state.x, robot_state.y])
        goal_pos = np.array([goal.x, goal.y])

        # Attractive force
        f_att = self.attractive_force(robot_pos, goal_pos)

        # Repulsive forces
        f_rep = np.zeros(2)
        for obs in obstacles:
            obs_pos = np.array([obs.x, obs.y])
            f_rep += self.repulsive_force(robot_pos, obs_pos)

        # Total force
        f_total = f_att + f_rep

        # Convert to velocity command
        desired_heading = np.arctan2(f_total[1], f_total[0])
        heading_error = desired_heading - robot_state.theta

        # Normalize to [-π, π]
        heading_error = np.arctan2(np.sin(heading_error),
                                  np.cos(heading_error))

        return ControlCommand(
            linear=min(np.linalg.norm(f_total), self.max_vel),
            angular=2.0 * heading_error,
            left_wheel=0,  # Will be computed
            right_wheel=0
        )
```

### Integration with Main Controller

```python
class HybridController:
    """
    Combines trajectory tracking with obstacle avoidance.
    Uses trajectory controller normally, switches to avoidance when needed.
    """

    def __init__(self, trajectory_controller, avoidance_controller):
        self.traj_ctrl = trajectory_controller
        self.avoid_ctrl = avoidance_controller
        self.mode = "TRACKING"

    def detect_obstacles(self, robot_state, sensor_data):
        """Detect obstacles from sensor data"""
        obstacles = []
        # Process LIDAR/camera data to extract obstacle positions
        return obstacles

    def compute_control(self, robot_state, trajectory, obstacles, dt):
        """Compute control with obstacle avoidance"""

        # Check for nearby obstacles
        min_obstacle_dist = self.get_min_obstacle_distance(
            robot_state, obstacles
        )

        if min_obstacle_dist < self.danger_threshold:
            # Switch to avoidance mode
            self.mode = "AVOIDING"
            goal = self.find_look_ahead_point(robot_state, trajectory)
            return self.avoid_ctrl.compute_velocity(
                robot_state, goal, obstacles
            )
        else:
            # Normal trajectory tracking
            self.mode = "TRACKING"
            return self.traj_ctrl.compute_control(
                robot_state, trajectory, dt
            )
```

## 🧪 Testing

### Test Categories

1. **Unit Tests**: Individual component testing
2. **Integration Tests**: Multi-component interaction
3. **Validation Tests**: Algorithm correctness verification
4. **Performance Tests**: Speed and resource usage

## 🛠️ AI Tools Used

### Development Workflow

1. **Claude AI**: Documentation and research for smoothing algorithm
2. **GitHub Copilot**: Code completion and boilerplate generation
