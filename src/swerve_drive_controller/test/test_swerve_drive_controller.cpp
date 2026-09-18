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

#include "test_swerve_drive_controller.hpp"

#include <gtest/gtest.h>
#include "controller_interface/controller_interface_base.hpp"
#include "lifecycle_msgs/msg/state.hpp"

using lifecycle_msgs::msg::State;

namespace swerve_drive_controller
{

class SwerveDriveControllerTest : public SwerveDriveControllerFixture<TestableSwerveDriveController>
{
};

// cppcheck-suppress syntaxError
TEST_F(SwerveDriveControllerTest, init_fails_without_parameters)
{
  const auto ret =
    controller_->init(controller_name_, urdf_, 0, "", controller_->define_custom_node_options());
  ASSERT_EQ(ret, controller_interface::return_type::OK);
}

TEST_F(SwerveDriveControllerTest, configure_fails_with_missing_wheels)
{
  std::vector<std::string> wheel_joints = {"front_left_wheel_joint", "front_right_wheel_joint"};
  std::vector<std::string> steering_joints = {"front_left_axle_joint", "front_right_axle_joint"};
  ASSERT_EQ(InitController(wheel_joints, steering_joints), controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(SwerveDriveControllerTest, configure_succeeds_no_namespace)
{
  ASSERT_EQ(InitController(), controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto command_interfaces = controller_->command_interface_configuration();
  ASSERT_EQ(
    command_interfaces.names.size(), wheel_joint_names_.size() + steering_joint_names_.size());
  for (size_t i = 0; i < wheel_joint_names_.size(); ++i)
  {
    EXPECT_EQ(command_interfaces.names[i], wheel_joint_names_[i] + "/" + HW_IF_VELOCITY);
  }
  for (size_t i = 0; i < steering_joint_names_.size(); ++i)
  {
    EXPECT_EQ(
      command_interfaces.names[i + wheel_joint_names_.size()],
      steering_joint_names_[i] + "/" + HW_IF_POSITION);
  }
  EXPECT_EQ(
    command_interfaces.type, controller_interface::interface_configuration_type::INDIVIDUAL);

  auto state_interfaces = controller_->state_interface_configuration();
  ASSERT_EQ(
    state_interfaces.names.size(), wheel_joint_names_.size() + steering_joint_names_.size());
  for (size_t i = 0; i < wheel_joint_names_.size(); ++i)
  {
    EXPECT_EQ(state_interfaces.names[i], wheel_joint_names_[i] + "/" + HW_IF_VELOCITY);
  }
  for (size_t i = 0; i < steering_joint_names_.size(); ++i)
  {
    EXPECT_EQ(
      state_interfaces.names[i + wheel_joint_names_.size()],
      steering_joint_names_[i] + "/" + HW_IF_POSITION);
  }
  EXPECT_EQ(state_interfaces.type, controller_interface::interface_configuration_type::INDIVIDUAL);
}

TEST_F(SwerveDriveControllerTest, configure_succeeds_with_namespace)
{
  std::string test_namespace = "/test_namespace";
  ASSERT_EQ(
    InitController(wheel_joint_names_, steering_joint_names_, {}, test_namespace),
    controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(SwerveDriveControllerTest, configure_succeeds_tf_prefix_false_no_namespace)
{
  std::string odom_id = "odom";
  std::string base_frame_id = "base_footprint";
  std::string frame_prefix = "test_prefix";

  ASSERT_EQ(
    InitController(
      wheel_joint_names_, steering_joint_names_,
      {rclcpp::Parameter("tf_frame_prefix_enable", rclcpp::ParameterValue(false)),
       rclcpp::Parameter("tf_frame_prefix", rclcpp::ParameterValue(frame_prefix)),
       rclcpp::Parameter("odom", rclcpp::ParameterValue(odom_id)),
       rclcpp::Parameter("base_footprint", rclcpp::ParameterValue(base_frame_id))}),
    controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  const auto & odom_msg = controller_->get_odometry_message();
  std::string test_odom_frame_id = odom_msg.header.frame_id;
  std::string test_base_frame_id = odom_msg.child_frame_id;
  // Namespace is "/", so no prefix
  ASSERT_EQ(test_odom_frame_id, odom_id);
  ASSERT_EQ(test_base_frame_id, base_frame_id);
}

TEST_F(SwerveDriveControllerTest, configure_succeeds_tf_prefix_true_no_namespace)
{
  std::string odom_id = "odom";
  std::string base_frame_id = "base_footprint";
  std::string frame_prefix = "test_prefix";

  ASSERT_EQ(
    InitController(
      wheel_joint_names_, steering_joint_names_,
      {rclcpp::Parameter("tf_frame_prefix_enable", rclcpp::ParameterValue(true)),
       rclcpp::Parameter("tf_frame_prefix", rclcpp::ParameterValue(frame_prefix)),
       rclcpp::Parameter("odom", rclcpp::ParameterValue(odom_id)),
       rclcpp::Parameter("base_footprint", rclcpp::ParameterValue(base_frame_id))}),
    controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  const auto & odom_msg = controller_->get_odometry_message();
  std::string test_odom_frame_id = odom_msg.header.frame_id;
  std::string test_base_frame_id = odom_msg.child_frame_id;
  ASSERT_EQ(test_odom_frame_id, odom_id);
  ASSERT_EQ(test_base_frame_id, base_frame_id);
}

TEST_F(SwerveDriveControllerTest, configure_succeeds_tf_blank_prefix_true_no_namespace)
{
  std::string odom_id = "odom";
  std::string base_frame_id = "base_footprint";
  std::string frame_prefix = "";

  ASSERT_EQ(
    InitController(
      wheel_joint_names_, steering_joint_names_,
      {rclcpp::Parameter("tf_frame_prefix_enable", rclcpp::ParameterValue(true)),
       rclcpp::Parameter("tf_frame_prefix", rclcpp::ParameterValue(frame_prefix)),
       rclcpp::Parameter("odom", rclcpp::ParameterValue(odom_id)),
       rclcpp::Parameter("base_footprint", rclcpp::ParameterValue(base_frame_id))}),
    controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  const auto & odom_msg = controller_->get_odometry_message();
  std::string test_odom_frame_id = odom_msg.header.frame_id;
  std::string test_base_frame_id = odom_msg.child_frame_id;
  ASSERT_EQ(test_odom_frame_id, odom_id);
  ASSERT_EQ(test_base_frame_id, base_frame_id);
}

TEST_F(SwerveDriveControllerTest, configure_succeeds_tf_prefix_false_set_namespace)
{
  std::string test_namespace = "/test_namespace";
  std::string odom_id = "odom";
  std::string base_frame_id = "base_footprint";
  std::string frame_prefix = "test_prefix";

  ASSERT_EQ(
    InitController(
      wheel_joint_names_, steering_joint_names_,
      {rclcpp::Parameter("tf_frame_prefix_enable", rclcpp::ParameterValue(false)),
       rclcpp::Parameter("tf_frame_prefix", rclcpp::ParameterValue(frame_prefix)),
       rclcpp::Parameter("odom", rclcpp::ParameterValue(odom_id)),
       rclcpp::Parameter("base_footprint", rclcpp::ParameterValue(base_frame_id))},
      test_namespace),
    controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  const auto & odom_msg = controller_->get_odometry_message();
  std::string test_odom_frame_id = odom_msg.header.frame_id;
  std::string test_base_frame_id = odom_msg.child_frame_id;
  ASSERT_EQ(test_odom_frame_id, "/test_namespace/odom");
  ASSERT_EQ(test_base_frame_id, "/test_namespace/base_footprint");
}

TEST_F(SwerveDriveControllerTest, configure_succeeds_tf_prefix_true_set_namespace)
{
  std::string test_namespace = "/test_namespace";
  std::string odom_id = "odom";
  std::string base_frame_id = "base_footprint";
  std::string frame_prefix = "test_prefix";

  ASSERT_EQ(
    InitController(
      wheel_joint_names_, steering_joint_names_,
      {rclcpp::Parameter("tf_frame_prefix_enable", rclcpp::ParameterValue(true)),
       rclcpp::Parameter("tf_frame_prefix", rclcpp::ParameterValue(frame_prefix)),
       rclcpp::Parameter("odom", rclcpp::ParameterValue(odom_id)),
       rclcpp::Parameter("base_footprint", rclcpp::ParameterValue(base_frame_id))},
      test_namespace),
    controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  const auto & odom_msg = controller_->get_odometry_message();
  std::string test_odom_frame_id = odom_msg.header.frame_id;
  std::string test_base_frame_id = odom_msg.child_frame_id;
  ASSERT_EQ(test_odom_frame_id, "/test_namespace/odom");
  ASSERT_EQ(test_base_frame_id, "/test_namespace/base_footprint");
}

TEST_F(SwerveDriveControllerTest, configure_succeeds_tf_blank_prefix_true_set_namespace)
{
  std::string test_namespace = "/test_namespace";
  std::string odom_id = "odom";
  std::string base_frame_id = "base_footprint";
  std::string frame_prefix = "";

  ASSERT_EQ(
    InitController(
      wheel_joint_names_, steering_joint_names_,
      {rclcpp::Parameter("tf_frame_prefix_enable", rclcpp::ParameterValue(true)),
       rclcpp::Parameter("tf_frame_prefix", rclcpp::ParameterValue(frame_prefix)),
       rclcpp::Parameter("odom", rclcpp::ParameterValue(odom_id)),
       rclcpp::Parameter("base_footprint", rclcpp::ParameterValue(base_frame_id))},
      test_namespace),
    controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  const auto & odom_msg = controller_->get_odometry_message();
  std::string test_odom_frame_id = odom_msg.header.frame_id;
  std::string test_base_frame_id = odom_msg.child_frame_id;
  ASSERT_EQ(test_odom_frame_id, "/test_namespace/odom");
  ASSERT_EQ(test_base_frame_id, "/test_namespace/base_footprint");
}

TEST_F(SwerveDriveControllerTest, activate_fails_without_resources_assigned)
{
  ASSERT_EQ(InitController(), controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(SwerveDriveControllerTest, activate_succeeds_with_resources_assigned)
{
  ASSERT_EQ(InitController(), controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  assignResources();
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(SwerveDriveControllerTest, deactivate_then_activate)
{
  ASSERT_EQ(InitController(), controller_interface::return_type::OK);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  auto state = controller_->get_node()->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  assignResources();

  state = controller_->get_node()->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, state.id());

  waitForSetup();

  publish_twist(1.0, 0.0, 0.0);
  controller_->wait_for_twist(executor);

  ASSERT_EQ(
    controller_->update_reference_from_subscribers(
      rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update_and_write_commands(
      rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  std::vector<double> expected_wheel_vel_cmds = {10.0, 10.0, 10.0, 10.0};
  std::vector<double> expected_steering_pos_cmds = {0.0, 0.0, 0.0, 0.0};
  for (size_t i = 0; i < wheel_vel_cmds_.size(); i++)
  {
    EXPECT_DOUBLE_EQ(command_itfs_[i].get_optional().value(), expected_wheel_vel_cmds[i]);
  }
  for (size_t i = 0; i < steering_pos_cmds_.size(); i++)
  {
    EXPECT_DOUBLE_EQ(
      command_itfs_[i + wheel_vel_cmds_.size()].get_optional().value(),
      expected_steering_pos_cmds[i]);
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  state = controller_->get_node()->deactivate();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(
    controller_->update_reference_from_subscribers(
      rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update_and_write_commands(
      rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  for (size_t i = 0; i < command_itfs_.size(); i++)
  {
    EXPECT_EQ(command_itfs_[i].get_optional().value(), 0.0);
  }

  state = controller_->get_node()->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, state.id());

  waitForSetup();

  for (size_t i = 0; i < command_itfs_.size(); i++)
  {
    EXPECT_EQ(command_itfs_[i].get_optional().value(), 0.0);
  }

  publish_twist(1.0, 0.0, 0.0);  // Forward motion
  controller_->wait_for_twist(executor);

  ASSERT_EQ(
    controller_->update_reference_from_subscribers(
      rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update_and_write_commands(
      rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  for (size_t i = 0; i < wheel_vel_cmds_.size(); i++)
  {
    EXPECT_DOUBLE_EQ(command_itfs_[i].get_optional().value(), expected_wheel_vel_cmds[i]);
  }
  for (size_t i = 0; i < steering_pos_cmds_.size(); i++)
  {
    EXPECT_DOUBLE_EQ(
      command_itfs_[i + wheel_vel_cmds_.size()].get_optional().value(),
      expected_steering_pos_cmds[i]);
  }

  // Deactivate and cleanup
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  state = controller_->get_node()->deactivate();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);
  state = controller_->get_node()->cleanup();
  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, state.id());
  executor.cancel();
}

TEST_F(SwerveDriveControllerTest, command_with_zero_timestamp_is_accepted_with_warning)
{
  ASSERT_EQ(InitController(), controller_interface::return_type::OK);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  auto state = controller_->get_node()->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  assignResources();

  state = controller_->get_node()->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, state.id());

  waitForSetup();

  publish_twist_timestamped(rclcpp::Time(0, 0, RCL_ROS_TIME), 1.0, 0.0, 0.0);
  controller_->wait_for_twist(executor);

  ASSERT_EQ(
    controller_->update_reference_from_subscribers(
      rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update_and_write_commands(
      rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  std::vector<double> expected_wheel_vel_cmds = {10.0, 10.0, 10.0, 10.0};
  std::vector<double> expected_steering_pos_cmds = {0.0, 0.0, 0.0, 0.0};
  for (size_t i = 0; i < wheel_vel_cmds_.size(); i++)
  {
    EXPECT_DOUBLE_EQ(command_itfs_[i].get_optional().value(), expected_wheel_vel_cmds[i]);
  }
  for (size_t i = 0; i < steering_pos_cmds_.size(); i++)
  {
    EXPECT_DOUBLE_EQ(
      command_itfs_[i + wheel_vel_cmds_.size()].get_optional().value(),
      expected_steering_pos_cmds[i]);
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  state = controller_->get_node()->deactivate();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);
  state = controller_->get_node()->cleanup();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_UNCONFIGURED);
  executor.cancel();
}

TEST(SwerveDriveKinematicsTest, test_angle_limits_and_hysteresis)
{
  SwerveDriveKinematics kinematics;
  kinematics.calculate_wheel_position(0.5, 0.4);

  // Wheel commands requesting backwards motion (theta = pi, vel = 2.0 m/s)
  std::array<WheelCommand, 4> cmds;
  for (int i = 0; i < 4; ++i)
  {
    cmds[i] = {M_PI, 2.0, 2.0 / 0.05};
  }

  std::array<double, 4> current_angles = {{0.0, 0.0, 0.0, 0.0}};
  // Limit to +/- 90 degrees (+/- 1.5708 rad)
  double min_limit = -M_PI_2;
  double max_limit = M_PI_2;

  auto opt_cmds = kinematics.optimize_wheel_commands(cmds, current_angles, min_limit, max_limit);

  for (int i = 0; i < 4; ++i)
  {
    // The angle must be strictly within [-pi/2, pi/2]
    EXPECT_GE(opt_cmds[i].steering_angle, min_limit - 1e-4);
    EXPECT_LE(opt_cmds[i].steering_angle, max_limit + 1e-4);
    // Reversed angle for pi should be 0.0
    EXPECT_NEAR(opt_cmds[i].steering_angle, 0.0, 1e-3);
    // Velocity must be inverted (-2.0 m/s)
    EXPECT_NEAR(opt_cmds[i].drive_velocity, -2.0, 1e-3);
  }
}

TEST(SwerveDriveKinematicsTest, test_continuous_rotation_no_flips)
{
  SwerveDriveKinematics kinematics;
  kinematics.calculate_wheel_position(0.5, 0.4);

  std::array<double, 4> current_angles = {{0.0, 0.0, 0.0, 0.0}};

  // Rotate stick continuously for 2 full revolutions (0 to 4*pi) in 0.1 rad increments
  for (int step = 1; step <= 63; ++step)
  {
    double desired_angle = angles::normalize_angle(step * 0.1);
    std::array<WheelCommand, 4> cmds;
    for (int i = 0; i < 4; ++i)
    {
      cmds[i] = {desired_angle, 1.0, 1.0 / 0.05};
    }

    auto opt_cmds = kinematics.optimize_wheel_commands(
      cmds, current_angles, -M_PI, M_PI, false /* enable_steering_limits = false */);

    for (int i = 0; i < 4; ++i)
    {
      // No 180-degree jumps during continuous rotation
      double jump = std::abs(opt_cmds[i].steering_angle - current_angles[i]);
      EXPECT_LT(jump, M_PI_2);
      EXPECT_GT(opt_cmds[i].drive_velocity, 0.0);
      current_angles[i] = opt_cmds[i].steering_angle;
    }
  }

  // After 2 revolutions, current angle should have accumulated past 2*pi smoothly
  for (int i = 0; i < 4; ++i)
  {
    EXPECT_GT(current_angles[i], 6.0);
  }
}

TEST(SwerveDriveKinematicsTest, test_small_radius_circling_stability)
{
  SwerveDriveKinematics kinematics;
  kinematics.calculate_wheel_position(0.5, 0.4);

  std::array<double, 4> current_angles = {{0.0, 0.0, 0.0, 0.0}};

  // Robot driving in a small radius circle: vx=0.1, vy=0.0, wz=1.5
  for (int step = 0; step < 50; ++step)
  {
    auto wheel_cmds = kinematics.compute_wheel_commands(0.1, 0.0, 1.5, 0.05);
    auto opt_cmds = kinematics.optimize_wheel_commands(
      wheel_cmds, current_angles, -M_PI, M_PI, false);

    for (int i = 0; i < 4; ++i)
    {
      EXPECT_TRUE(std::isfinite(opt_cmds[i].steering_angle));
      EXPECT_TRUE(std::isfinite(opt_cmds[i].drive_velocity));
      current_angles[i] = opt_cmds[i].steering_angle;
    }
  }
}

TEST(SwerveDriveKinematicsTest, test_bounded_plus_minus_pi_limits)
{
  SwerveDriveKinematics kinematics;
  kinematics.calculate_wheel_position(0.5, 0.4);

  std::array<double, 4> current_angles = {{0.0, 0.0, 0.0, 0.0}};

  // Rotate stick full 360 degrees in 72 steps with strict [-pi, pi] limits
  for (int step = 0; step <= 72; ++step)
  {
    double theta = step * (2.0 * M_PI / 72.0);
    double desired_angle = std::atan2(std::sin(theta), std::cos(theta));
    std::array<WheelCommand, 4> cmds;
    for (int i = 0; i < 4; ++i)
    {
      cmds[i] = {desired_angle, 1.5, 1.5 / 0.05};
    }

    auto opt_cmds = kinematics.optimize_wheel_commands(
      cmds, current_angles, -M_PI, M_PI, true /* enable_steering_limits = true */);

    for (int i = 0; i < 4; ++i)
    {
      // Angle MUST strictly stay within [-pi, pi]
      EXPECT_GE(opt_cmds[i].steering_angle, -M_PI - 1e-4);
      EXPECT_LE(opt_cmds[i].steering_angle, M_PI + 1e-4);

      // Physical ground velocity vector MUST match desired velocity vector
      double vx_cmd = opt_cmds[i].drive_velocity * std::cos(opt_cmds[i].steering_angle);
      double vy_cmd = opt_cmds[i].drive_velocity * std::sin(opt_cmds[i].steering_angle);
      double vx_des = 1.5 * std::cos(desired_angle);
      double vy_des = 1.5 * std::sin(desired_angle);
      EXPECT_NEAR(vx_cmd, vx_des, 1e-3);
      EXPECT_NEAR(vy_cmd, vy_des, 1e-3);

      current_angles[i] = opt_cmds[i].steering_angle;
    }
  }
}

TEST(SwerveDriveKinematicsTest, test_tight_turn_bounded_smoothness)
{
  SwerveDriveKinematics kinematics;
  kinematics.calculate_wheel_position(0.5, 0.4);

  std::array<double, 4> current_angles = {{0.0, 0.0, 0.0, 0.0}};

  // 1. Driving in tight circle with bounded +/- pi limits (vx=0.1, vy=0.0, wz=2.0)
  for (int step = 0; step < 50; ++step)
  {
    auto raw_cmds = kinematics.compute_wheel_commands(0.1, 0.0, 2.0, 0.05);
    auto opt_cmds = kinematics.optimize_wheel_commands(
      raw_cmds, current_angles, -M_PI, M_PI, true);

    for (int i = 0; i < 4; ++i)
    {
      EXPECT_GE(opt_cmds[i].steering_angle, -M_PI - 1e-4);
      EXPECT_LE(opt_cmds[i].steering_angle, M_PI + 1e-4);

      if (step > 0)
      {
        // No large flips (> 60 deg) during execution of the turn
        double jump = std::abs(angles::shortest_angular_distance(current_angles[i], opt_cmds[i].steering_angle));
        EXPECT_LT(jump, M_PI / 3.0);
      }
      current_angles[i] = opt_cmds[i].steering_angle;
    }
  }

  // 2. Transition: Straight forward (vx=0.3, wz=0.0) -> Tight turn (vx=0.05, wz=2.5) -> Straight forward
  current_angles.fill(0.0);
  kinematics.reset_inversion_state();

  for (int step = 0; step < 60; ++step)
  {
    double phase = std::sin(step * M_PI / 59.0);
    double vx = 0.3 * (1.0 - phase) + 0.05 * phase;
    double wz = 2.5 * phase;

    auto raw_cmds = kinematics.compute_wheel_commands(vx, 0.0, wz, 0.05);
    auto opt_cmds = kinematics.optimize_wheel_commands(
      raw_cmds, current_angles, -M_PI, M_PI, true);

    for (int i = 0; i < 4; ++i)
    {
      EXPECT_GE(opt_cmds[i].steering_angle, -M_PI - 1e-4);
      EXPECT_LE(opt_cmds[i].steering_angle, M_PI + 1e-4);

      if (step > 0)
      {
        double jump = std::abs(angles::shortest_angular_distance(current_angles[i], opt_cmds[i].steering_angle));
        EXPECT_LT(jump, M_PI / 3.0);
      }
      current_angles[i] = opt_cmds[i].steering_angle;
    }
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
}  // namespace swerve_drive_controller
