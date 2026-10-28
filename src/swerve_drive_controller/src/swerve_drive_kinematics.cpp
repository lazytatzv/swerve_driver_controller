// Copyright 2025 ros2_control development team
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "swerve_drive_controller/swerve_drive_kinematics.hpp"
#include <algorithm>

namespace swerve_drive_controller
{

SwerveDriveKinematics::SwerveDriveKinematics()
: odometry_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
  previous_inversion_{{false, false, false, false}}
{}

void SwerveDriveKinematics::reset_inversion_state()
{
  previous_inversion_.fill(false);
  previous_global_inversion_ = false;
}

void SwerveDriveKinematics::calculate_wheel_position(
  double wheel_base, double track_width, double x_offset, double y_offset)
{
  double half_length = wheel_base / 2.0;
  double half_width = track_width / 2.0;

  wheel_positions_[0] = {half_length - x_offset, half_width - y_offset};    // Front Left  (+x, +y)
  wheel_positions_[1] = {half_length - x_offset, -half_width - y_offset};   // Front Right (+x, -y)
  wheel_positions_[2] = {-half_length - x_offset, half_width - y_offset};   // Rear Left   (-x, +y)
  wheel_positions_[3] = {-half_length - x_offset, -half_width - y_offset};  // Rear Right  (-x, -y)
}

std::array<WheelCommand, 4> SwerveDriveKinematics::compute_wheel_commands(
  double linear_velocity_x, double linear_velocity_y, double angular_velocity_z,
  double wheel_radius)
{
  std::array<WheelCommand, 4> wheel_commands;

  if (wheel_radius <= 0.0)
  {
    std::cerr << "invalid wheel_radius <= 0.0\n";
    // fallthrough: compute but set angular velocities to 0 to avoid div-by-zero
  }

  for (std::size_t i = 0; i < 4; i++)
  {
    const auto & [wx, wy] = wheel_positions_[i];

    double vx = linear_velocity_x - angular_velocity_z * wy;
    double vy = linear_velocity_y + angular_velocity_z * wx;

    double linear_speed = std::hypot(vx, vy);
    double steering = std::atan2(vy, vx);

    wheel_commands[i].drive_velocity = linear_speed;

    if (wheel_radius > 0.0)
    {
      wheel_commands[i].drive_angular_velocity = linear_speed / wheel_radius;  // rad/s
    }
    else
    {
      wheel_commands[i].drive_angular_velocity = 0.0;  // safe fallback
    }

    wheel_commands[i].steering_angle = steering;
  }

  return wheel_commands;
}

std::array<WheelCommand, 4> SwerveDriveKinematics::optimize_wheel_commands(
  const std::array<WheelCommand, 4> & wheel_commands,
  const std::array<double, 4> & current_steering_angles,
  double min_steering_angle,
  double max_steering_angle,
  bool enable_steering_limits)
{
  if (min_steering_angle > max_steering_angle)
  {
    std::swap(min_steering_angle, max_steering_angle);
  }

  std::array<WheelCommand, 4> optimized_commands = wheel_commands;

  constexpr double hysteresis = 0.15;           // ~8.6 deg margin to prevent thrashing near 90 deg
  constexpr double min_speed_threshold = 0.05;   // 0.05 m/s deadband against joystick noise
  constexpr double pi_2 = M_PI_2;

  // Check if all wheels have identical desired steering angle (pure translation)
  bool is_uniform_translation = true;
  double ref_angle = wheel_commands[0].steering_angle;
  for (std::size_t i = 1; i < 4; i++)
  {
    if (std::abs(angles::shortest_angular_distance(ref_angle, wheel_commands[i].steering_angle)) > 1e-3)
    {
      is_uniform_translation = false;
      break;
    }
  }

  // If pure translation, determine coordinated inversion decision across all 4 wheels
  bool global_choose_B = false;
  if (is_uniform_translation)
  {
    double max_rot_A = 0.0;
    for (std::size_t i = 0; i < 4; i++)
    {
      double curr = current_steering_angles[i];
      double diff_A = angles::shortest_angular_distance(curr, ref_angle);
      max_rot_A = std::max(max_rot_A, std::abs(diff_A));
    }

    global_choose_B = previous_global_inversion_ ?
      (max_rot_A > (pi_2 - hysteresis)) : (max_rot_A > (pi_2 + hysteresis));
    previous_global_inversion_ = global_choose_B;
  }

  for (std::size_t i = 0; i < 4; i++)
  {
    double curr = current_steering_angles[i];
    double desired = wheel_commands[i].steering_angle;
    double speed = wheel_commands[i].drive_velocity;
    double angular_speed = wheel_commands[i].drive_angular_velocity;

    // If commanded speed is within deadband, keep current continuous steering angle
    if (std::abs(speed) < min_speed_threshold)
    {
      optimized_commands[i].steering_angle = curr;
      optimized_commands[i].drive_velocity = 0.0;
      optimized_commands[i].drive_angular_velocity = 0.0;
      continue;
    }

    // Direct heading candidate (Option A)
    double diff_A = angles::shortest_angular_distance(curr, desired);
    double target_A = curr + diff_A;
    double rot_A = std::abs(diff_A);

    // Inverted heading candidate (Option B: steer to 180 deg opposite, reverse drive)
    double heading_B = angles::normalize_angle(desired + M_PI);
    double diff_B = angles::shortest_angular_distance(curr, heading_B);
    double target_B = curr + diff_B;

    bool choose_B = false;
    if (is_uniform_translation)
    {
      choose_B = global_choose_B;
    }
    else
    {
      choose_B = previous_inversion_[i] ?
        (rot_A > (pi_2 - hysteresis)) : (rot_A > (pi_2 + hysteresis));
    }

    if (enable_steering_limits)
    {
      constexpr double eps = 1e-4;
      bool in_range_A = (target_A >= min_steering_angle - eps) && (target_A <= max_steering_angle + eps);
      bool in_range_B = (target_B >= min_steering_angle - eps) && (target_B <= max_steering_angle + eps);

      if (in_range_A && in_range_B)
      {
        // Both within range, preserve choose_B decision
      }
      else if (in_range_B && !in_range_A)
      {
        choose_B = true;
      }
      else if (in_range_A && !in_range_B)
      {
        choose_B = false;
      }
    }

    previous_inversion_[i] = choose_B;

    double final_steering_angle = choose_B ? target_B : target_A;
    double final_speed = choose_B ? -speed : speed;
    double final_angular_speed = choose_B ? -angular_speed : angular_speed;

    if (enable_steering_limits)
    {
      final_steering_angle = std::clamp(final_steering_angle, min_steering_angle, max_steering_angle);
    }

    optimized_commands[i].steering_angle = final_steering_angle;
    optimized_commands[i].drive_velocity = final_speed;
    optimized_commands[i].drive_angular_velocity = final_angular_speed;
  }

  return optimized_commands;
}

OdometryState SwerveDriveKinematics::update_odometry(
  const std::array<double, 4> & wheel_velocities, const std::array<double, 4> & steering_angles,
  double dt,
  const std::array<bool, 4> & active_wheels)
{
  // Full rigid-body Least-Squares Odometry over active wheels (Fault-Tolerant Degraded Swerve)
  // Solves [1 0 -yi; 0 1 xi] * [vx; vy; wz] = [vx_i; vy_i] via (A^T A)^-1 A^T b
  std::size_t active_count = 0;
  double sum_x = 0.0, sum_y = 0.0, sum_r2 = 0.0;
  double Y0 = 0.0, Y1 = 0.0, Y2 = 0.0;

  for (std::size_t i = 0; i < 4; i++)
  {
    if (!active_wheels[i])
    {
      continue;
    }

    double vx = wheel_velocities[i] * std::cos(steering_angles[i]);
    double vy = wheel_velocities[i] * std::sin(steering_angles[i]);
    double xi = wheel_positions_[i].first;
    double yi = wheel_positions_[i].second;

    sum_x += xi;
    sum_y += yi;
    sum_r2 += (xi * xi + yi * yi);

    Y0 += vx;
    Y1 += vy;
    Y2 += (vy * xi - vx * yi);
    ++active_count;
  }

  if (active_count == 0)
  {
    return odometry_;
  }

  double vx_robot = 0.0;
  double vy_robot = 0.0;
  double wz_robot = 0.0;

  double N = static_cast<double>(active_count);
  double m02 = -sum_y;
  double m12 = sum_x;
  double m22 = sum_r2;

  double det = N * (N * m22 - m12 * m12 - m02 * m02);

  if (active_count >= 2 && std::fabs(det) > 1e-9)
  {
    double c00 = N * m22 - m12 * m12;
    double c01 = m02 * m12;
    double c02 = -N * m02;

    double c10 = m02 * m12;
    double c11 = N * m22 - m02 * m02;
    double c12 = -N * m12;

    double c20 = -N * m02;
    double c21 = -N * m12;
    double c22 = N * N;

    vx_robot = (c00 * Y0 + c01 * Y1 + c02 * Y2) / det;
    vy_robot = (c10 * Y0 + c11 * Y1 + c12 * Y2) / det;
    wz_robot = (c20 * Y0 + c21 * Y1 + c22 * Y2) / det;
  }
  else
  {
    // Fallback for single active wheel
    vx_robot = Y0 / N;
    vy_robot = Y1 / N;
    wz_robot = 0.0;
  }

  // Compute linear displacement in robot frame
  double dx_robot = vx_robot * dt;
  double dy_robot = vy_robot * dt;
  double dtheta = wz_robot * dt;

  // Integrate using Exact integration for rotation, RK2 fallback for near-zero angular velocity
  if (std::fabs(dtheta) < 1e-6)
  {
    // RK2 (Runge-Kutta 2nd order) integration for small angular changes
    // Use midpoint heading for better accuracy
    double mid_theta = odometry_.theta + dtheta * 0.5;
    double cos_mid = std::cos(mid_theta);
    double sin_mid = std::sin(mid_theta);

    odometry_.x += dx_robot * cos_mid - dy_robot * sin_mid;
    odometry_.y += dx_robot * sin_mid + dy_robot * cos_mid;
    odometry_.theta += dtheta;
  }
  else
  {
    // Exact integration for arc motion
    // When the robot rotates, it follows an arc trajectory
    // This computes the exact displacement along that arc
    double theta_old = odometry_.theta;
    double theta_new = theta_old + dtheta;

    double sin_new = std::sin(theta_new);
    double sin_old = std::sin(theta_old);
    double cos_new = std::cos(theta_new);
    double cos_old = std::cos(theta_old);

    // Exact integration formula for simultaneous translation and rotation
    // Derived from integrating velocity over the arc
    odometry_.x += (dx_robot * (sin_new - sin_old) + dy_robot * (cos_new - cos_old)) / dtheta;
    odometry_.y += (dx_robot * (cos_old - cos_new) + dy_robot * (sin_new - sin_old)) / dtheta;
    odometry_.theta = theta_new;
  }

  // Normalize theta to [-pi, pi]
  odometry_.theta = angles::normalize_angle(odometry_.theta);
  odometry_.vx = vx_robot;
  odometry_.vy = vy_robot;
  odometry_.wz = wz_robot;
  return odometry_;
}
}  // namespace swerve_drive_controller
