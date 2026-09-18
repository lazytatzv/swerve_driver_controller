# Swerve Drive Controller for ROS 2

A robust, smooth, and full-featured 4-wheel independent steering & drive (**Swerve Drive**) controller implementation for `ros2_control`.

Includes an interactive simulation environment with **URDF model**, **Foxglove Studio 3D visualization**, and **PS5 DualSense gamepad teleoperation**.

---

## Features

- **ros2_control Chainable Controller:**
  - Implemented using modern `chainable_controller_interface`.
  - Subscribes to `/cmd_vel` (`geometry_msgs/msg/Twist` or `geometry_msgs/msg/TwistStamped`).
- **Dual-Mode Steering Support:**
  - **Continuous Mode (Default):** Infinite rotation without angle bounds for modules equipped with slip rings.
  - **Bounded Mode:** Mechanical hardstop / cable-protection mode (e.g., $\pm 180^\circ$ / $[-\pi, \pi]$) with soft-boundary avoidance.
- **Chattering Prevention (Hysteresis):**
  - $\pm 20^\circ$ hysteresis band around $90^\circ$ reversal boundaries to eliminate jitter during diagonal motion.
- **Smooth Continuous Velocity Scaling:**
  - Dynamic drive wheel velocity scaling based on $\cos(\text{steering\_error})$: automatically slows down wheel drive when steering is aligning and ramps up smoothly when aligned.
- **Deadband & Singularity Handling:**
  - Holds module angle when linear/angular speed is below threshold to prevent erratic wheel twitching when stopped or pivoting near modules.
- **Thoroughly Tested:**
  - Comprehensive GTest suite covering kinematics, continuous rotation, small-radius stability, and bounded limits (22 unit tests passing).

---

## Packages

- **`swerve_drive_controller`**: The core `ros2_control` controller plugin and kinematics library.
- **`swerve_drive_bringup`**: URDF robot model, mock hardware configuration, Foxglove bridge, and joystick teleop launch files.

---

## Getting Started

### 1. Prerequisites

ROS 2 (Lyrical, Jazzy, Iron, or Humble) with `ros2_control` stack installed:

```bash
./install_deps.sh
```

Or manually install:
```bash
sudo apt install -y \
  ros-$ROS_DISTRO-ros2-control \
  ros-$ROS_DISTRO-ros2-controllers \
  ros-$ROS_DISTRO-joint-state-broadcaster \
  ros-$ROS_DISTRO-generate-parameter-library \
  ros-$ROS_DISTRO-control-toolbox \
  ros-$ROS_DISTRO-realtime-tools \
  ros-$ROS_DISTRO-foxglove-bridge \
  ros-$ROS_DISTRO-teleop-twist-joy \
  ros-$ROS_DISTRO-joy \
  ros-$ROS_DISTRO-xacro
```

### 2. Build

```bash
colcon build --symlink-install
source install/setup.bash
```

### 3. Run Unit Tests

```bash
colcon test --packages-select swerve_drive_controller
colcon test-result --all --verbose
```

### 4. Launch Simulation with Foxglove Bridge

```bash
ros2 launch swerve_drive_bringup foxglove_sim.launch.py
```

1. Open **[Foxglove Studio](https://foxglove.dev/)** (desktop app or web app).
2. Connect to `ws://localhost:8765`.
3. Add a **3D Panel** and subscribe to `/robot_description` and `/tf`.
4. Connect a gamepad (PS5 DualSense / standard joystick) to `/dev/input/js0` and drive!

---

## Controller Configuration

Configuration file: `src/swerve_drive_bringup/config/swerve_controllers.yaml`

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `wheelbase` | `double` | `0.50` | Distance between front and rear axles (m) |
| `trackwidth` | `double` | `0.40` | Distance between left and right wheels (m) |
| `wheel_radius` | `double` | `0.05` | Wheel radius (m) |
| `enable_steering_angle_limits` | `bool` | `false` | Enable physical steering limit enforcement |
| `min_steering_angle` | `double` | `-3.14159` | Minimum steering angle in radians (when limits enabled) |
| `max_steering_angle` | `double` | `3.14159` | Maximum steering angle in radians (when limits enabled) |
| `use_stamped_vel` | `bool` | `false` | Accept `TwistStamped` instead of unstamped `Twist` |
| `cmd_vel_timeout` | `double` | `0.5` | Timeout before stopping robot if no commands received (s) |

---

## Teleoperation Controls (PS5 DualSense)

- **Left Stick Vertical (Axis 1):** Forward / Backward ($v_x$)
- **Left Stick Horizontal (Axis 0):** Left / Right strafe ($v_y$)
- **Right Stick Horizontal (Axis 3):** Yaw rotation ($\omega_z$)
- **R1 Button (Button 5):** Turbo speed boost

---

## License

This project is licensed under the [Apache-2.0 License](src/swerve_drive_controller/LICENSE).
