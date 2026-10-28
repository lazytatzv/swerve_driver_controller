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
#include <random>
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

TEST_F(SwerveDriveControllerTest, stall_detection_and_auto_recovery)
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

  // Command forward motion at 1.0 m/s
  publish_twist(1.0, 0.0, 0.0);
  controller_->wait_for_twist(executor);

  // All wheels active initially
  for (bool active : controller_->get_active_wheels())
  {
    EXPECT_TRUE(active);
  }

  // Simulate Wheel 1 (front-right) locked rotor/stalled (torque applied but actual velocity is 0.0)
  // while other wheels (0, 2, 3) are moving at 10.0 rad/s (1.0 m/s)
  wheel_vel_states_[1] = 0.0;
  wheel_vel_states_[0] = 10.0;
  wheel_vel_states_[2] = 10.0;
  wheel_vel_states_[3] = 10.0;

  // Step 70 cycles (> 60 cycles required for stall cutoff)
  for (int step = 0; step < 70; ++step)
  {
    ASSERT_EQ(
      controller_->update_reference_from_subscribers(
        rclcpp::Time(step / 100, (step % 100) * 10000000, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
      controller_interface::return_type::OK);
    ASSERT_EQ(
      controller_->update_and_write_commands(
        rclcpp::Time(step / 100, (step % 100) * 10000000, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
      controller_interface::return_type::OK);
  }

  // Wheel 1 must be marked inactive (stall protection triggered)
  EXPECT_FALSE(controller_->get_active_wheels()[1]);
  // Other wheels remain active
  EXPECT_TRUE(controller_->get_active_wheels()[0]);
  EXPECT_TRUE(controller_->get_active_wheels()[2]);
  EXPECT_TRUE(controller_->get_active_wheels()[3]);
  // Commanded velocity to wheel 1 must be cut to 0.0 to prevent motor burnout
  EXPECT_DOUBLE_EQ(command_itfs_[1].get_optional().value(), 0.0);

  // Now the robot stops: publish 0 velocity
  publish_twist(0.0, 0.0, 0.0);
  controller_->wait_for_twist(executor);

  // Step 160 cycles of rest (> 150 cycles needed for auto-recovery)
  for (int step = 70; step < 240; ++step)
  {
    ASSERT_EQ(
      controller_->update_reference_from_subscribers(
        rclcpp::Time(step / 100, (step % 100) * 10000000, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
      controller_interface::return_type::OK);
    ASSERT_EQ(
      controller_->update_and_write_commands(
        rclcpp::Time(step / 100, (step % 100) * 10000000, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
      controller_interface::return_type::OK);
  }

  // Wheel 1 must be auto-recovered and active again!
  EXPECT_TRUE(controller_->get_active_wheels()[1]);

  // Deactivate and cleanup
  state = controller_->get_node()->deactivate();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);
  state = controller_->get_node()->cleanup();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_UNCONFIGURED);
  executor.cancel();
}

TEST_F(SwerveDriveControllerTest, reversed_wheel_is_cut_off_not_inverted)
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

  // Wheel 2 spins backwards at full speed while the others follow the command.
  wheel_vel_states_[0] = 10.0;
  wheel_vel_states_[1] = 10.0;
  wheel_vel_states_[2] = -10.0;
  wheel_vel_states_[3] = 10.0;

  // Short glitch (< 30 cycles) must NOT disable the wheel
  for (int step = 0; step < 20; ++step)
  {
    ASSERT_EQ(
      controller_->update_reference_from_subscribers(
        rclcpp::Time(step / 100, (step % 100) * 10000000, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
      controller_interface::return_type::OK);
    ASSERT_EQ(
      controller_->update_and_write_commands(
        rclcpp::Time(step / 100, (step % 100) * 10000000, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
      controller_interface::return_type::OK);
  }
  EXPECT_TRUE(controller_->get_active_wheels()[2]);

  // Sustained reverse rotation must disable the wheel
  for (int step = 20; step < 60; ++step)
  {
    ASSERT_EQ(
      controller_->update_reference_from_subscribers(
        rclcpp::Time(step / 100, (step % 100) * 10000000, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
      controller_interface::return_type::OK);
    ASSERT_EQ(
      controller_->update_and_write_commands(
        rclcpp::Time(step / 100, (step % 100) * 10000000, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
      controller_interface::return_type::OK);
  }
  EXPECT_FALSE(controller_->get_active_wheels()[2]);
  EXPECT_TRUE(controller_->get_active_wheels()[0]);
  EXPECT_TRUE(controller_->get_active_wheels()[1]);
  EXPECT_TRUE(controller_->get_active_wheels()[3]);
  // Drive command is cut to 0, never sign-flipped
  EXPECT_DOUBLE_EQ(command_itfs_[2].get_optional().value(), 0.0);

  state = controller_->get_node()->deactivate();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);
  state = controller_->get_node()->cleanup();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_UNCONFIGURED);
  executor.cancel();
}

TEST_F(SwerveDriveControllerTest, degraded_mode_wheel_commands_and_passive_caster_steering)
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

  // Create publisher to ~/disabled_wheels
  auto disabled_pub = cmd_vel_publisher_node_->create_publisher<std_msgs::msg::Int32MultiArray>(
    "/test_swerve_drive_controller/disabled_wheels", rclcpp::SystemDefaultsQoS());

  // Disable Wheel 2 (rear-left)
  std_msgs::msg::Int32MultiArray dis_msg;
  dis_msg.data = {2};
  disabled_pub->publish(dis_msg);

  // Spin executor to process ~/disabled_wheels
  for (int i = 0; i < 20; ++i)
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  EXPECT_FALSE(controller_->get_active_wheels()[2]);
  EXPECT_TRUE(controller_->get_active_wheels()[0]);
  EXPECT_TRUE(controller_->get_active_wheels()[1]);
  EXPECT_TRUE(controller_->get_active_wheels()[3]);

  // Command forward motion + diagonal translation (vx = 1.0, vy = 0.5)
  publish_twist(1.0, 0.5, 0.0);
  controller_->wait_for_twist(executor);

  // Update
  ASSERT_EQ(
    controller_->update_reference_from_subscribers(
      rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update_and_write_commands(
      rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // 1. Active wheels (0, 1, 3) must be actively commanded to drive
  EXPECT_GT(std::abs(command_itfs_[0].get_optional().value()), 5.0);
  EXPECT_GT(std::abs(command_itfs_[1].get_optional().value()), 5.0);
  EXPECT_GT(std::abs(command_itfs_[3].get_optional().value()), 5.0);

  // 2. Disabled wheel (2) must have DRIVE velocity 0.0 (no torque, motor safe)
  EXPECT_DOUBLE_EQ(command_itfs_[2].get_optional().value(), 0.0);

  // 3. Disabled wheel (2) steering position command must be aligned with velocity vector
  // so it rolls along freely like a passive caster!
  double expected_steer = std::atan2(0.5, 1.0);
  EXPECT_NEAR(command_itfs_[2 + 4].get_optional().value(), expected_steer, 1e-2);

  // Deactivate and cleanup
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

TEST(SwerveDriveKinematicsTest, test_forward_inverse_roundtrip_exhaustive)
{
  SwerveDriveKinematics kinematics;
  const double wb = 0.36;
  const double tw = 0.40;
  const double radius = 0.035;
  kinematics.calculate_wheel_position(wb, tw);

  std::mt19937 gen(42);
  std::uniform_real_distribution<double> dist_v(-3.5, 3.5);
  std::uniform_real_distribution<double> dist_w(-8.0, 8.0);

  for (int iter = 0; iter < 1000; ++iter)
  {
    double vx_in = dist_v(gen);
    double vy_in = dist_v(gen);
    double wz_in = dist_w(gen);

    auto wheel_cmds = kinematics.compute_wheel_commands(vx_in, vy_in, wz_in, radius);

    std::array<double, 4> wheel_vels;
    std::array<double, 4> steer_angles;
    for (size_t i = 0; i < 4; ++i)
    {
      wheel_vels[i] = wheel_cmds[i].drive_velocity;
      steer_angles[i] = wheel_cmds[i].steering_angle;
    }

    auto odom = kinematics.update_odometry(wheel_vels, steer_angles, 0.02);

    EXPECT_NEAR(odom.vx, vx_in, 1e-9);
    EXPECT_NEAR(odom.vy, vy_in, 1e-9);
    EXPECT_NEAR(odom.wz, wz_in, 1e-9);
  }
}

TEST(SwerveDriveKinematicsTest, test_icr_on_each_wheel_module)
{
  SwerveDriveKinematics kinematics;
  const double wb = 0.36;
  const double tw = 0.40;
  const double radius = 0.035;
  kinematics.calculate_wheel_position(wb, tw);

  // Module positions: FL=(+0.18, +0.20), FR=(+0.18, -0.20), RL=(-0.18, +0.20), RR=(-0.18, -0.20)
  const std::array<std::pair<double, double>, 4> mod_pos = {{
    {0.18, 0.20}, {0.18, -0.20}, {-0.18, 0.20}, {-0.18, -0.20}
  }};

  const double omega = 2.5;

  for (size_t target_idx = 0; target_idx < 4; ++target_idx)
  {
    // ICR at target module: v_target = 0 => vx = omega * y_target, vy = -omega * x_target
    double vx = omega * mod_pos[target_idx].second;
    double vy = -omega * mod_pos[target_idx].first;

    auto cmds = kinematics.compute_wheel_commands(vx, vy, omega, radius);

    // Target module must have exactly 0 velocity
    EXPECT_NEAR(cmds[target_idx].drive_velocity, 0.0, 1e-9);

    // Other modules must have speed = omega * distance_to_target
    for (size_t other_idx = 0; other_idx < 4; ++other_idx)
    {
      if (other_idx == target_idx) continue;
      double dx = mod_pos[other_idx].first - mod_pos[target_idx].first;
      double dy = mod_pos[other_idx].second - mod_pos[target_idx].second;
      double dist = std::hypot(dx, dy);
      double expected_speed = std::abs(omega) * dist;

      EXPECT_NEAR(cmds[other_idx].drive_velocity, expected_speed, 1e-6);
    }

    // Odometry reconstruction check
    std::array<double, 4> wheel_vels, steer_angles;
    for (size_t i = 0; i < 4; ++i)
    {
      wheel_vels[i] = cmds[i].drive_velocity;
      steer_angles[i] = cmds[i].steering_angle;
    }
    auto odom = kinematics.update_odometry(wheel_vels, steer_angles, 0.02);
    EXPECT_NEAR(odom.vx, vx, 1e-9);
    EXPECT_NEAR(odom.vy, vy, 1e-9);
    EXPECT_NEAR(odom.wz, omega, 1e-9);
  }
}

TEST(SwerveDriveKinematicsTest, test_pure_translation_compass_sweep)
{
  SwerveDriveKinematics kinematics;
  kinematics.calculate_wheel_position(0.36, 0.40);

  const double speed = 2.0;
  for (int deg = 0; deg < 360; ++deg)
  {
    double rad = deg * M_PI / 180.0;
    double vx = speed * std::cos(rad);
    double vy = speed * std::sin(rad);

    auto cmds = kinematics.compute_wheel_commands(vx, vy, 0.0, 0.035);

    double expected_angle = std::atan2(std::sin(rad), std::cos(rad));

    for (size_t i = 0; i < 4; ++i)
    {
      EXPECT_NEAR(cmds[i].drive_velocity, speed, 1e-6);
      EXPECT_NEAR(cmds[i].steering_angle, expected_angle, 1e-6);
    }
  }
}

TEST(SwerveDriveKinematicsTest, test_pure_rotation_symmetry)
{
  SwerveDriveKinematics kinematics;
  kinematics.calculate_wheel_position(0.36, 0.40);

  double r_module = std::hypot(0.18, 0.20); // ~0.26907 m
  double omega = 3.0;

  auto cmds = kinematics.compute_wheel_commands(0.0, 0.0, omega, 0.035);

  for (size_t i = 0; i < 4; ++i)
  {
    EXPECT_NEAR(cmds[i].drive_velocity, omega * r_module, 1e-6);
  }

  // Check tangent angles for CCW spin (+wz)
  // FL (+0.18, +0.20): vx = -w*y < 0, vy = +w*x > 0 => 2nd quadrant: atan2(0.18, -0.20)
  EXPECT_NEAR(cmds[0].steering_angle, std::atan2(0.18, -0.20), 1e-6);
  // FR (+0.18, -0.20): vx = -w*(-0.2) > 0, vy = +w*0.18 > 0 => 1st quadrant: atan2(0.18, 0.20)
  EXPECT_NEAR(cmds[1].steering_angle, std::atan2(0.18, 0.20), 1e-6);
  // RL (-0.18, +0.20): vx = -w*0.2 < 0, vy = +w*(-0.18) < 0 => 3rd quadrant: atan2(-0.18, -0.20)
  EXPECT_NEAR(cmds[2].steering_angle, std::atan2(-0.18, -0.20), 1e-6);
  // RR (-0.18, -0.20): vx = -w*(-0.2) > 0, vy = +w*(-0.18) < 0 => 4th quadrant: atan2(-0.18, 0.20)
  EXPECT_NEAR(cmds[3].steering_angle, std::atan2(-0.18, 0.20), 1e-6);
}

TEST(SwerveDriveKinematicsTest, test_steering_optimization_shortest_path)
{
  SwerveDriveKinematics kinematics;
  kinematics.calculate_wheel_position(0.36, 0.40);

  // Current angle is 0.0. Command velocity in opposite direction (-X, angle = pi)
  std::array<WheelCommand, 4> cmds;
  for (int i = 0; i < 4; ++i)
  {
    cmds[i] = {M_PI, 1.5, 1.5 / 0.035};
  }

  std::array<double, 4> current_angles = {{0.0, 0.0, 0.0, 0.0}};
  auto opt = kinematics.optimize_wheel_commands(cmds, current_angles, -M_PI, M_PI, false);

  for (int i = 0; i < 4; ++i)
  {
    // Angle rotation must be minimized: instead of turning 180 deg to pi, steer 0 deg and reverse drive
    EXPECT_NEAR(opt[i].steering_angle, 0.0, 1e-3);
    EXPECT_NEAR(opt[i].drive_velocity, -1.5, 1e-3);
  }
}

TEST(SwerveDriveKinematicsTest, test_hysteresis_chattering_prevention)
{
  SwerveDriveKinematics kinematics;
  kinematics.calculate_wheel_position(0.36, 0.40);

  std::array<double, 4> current_angles = {{0.0, 0.0, 0.0, 0.0}};

  // 1. Move to 80 deg (1.396 rad) -> chooses Option A (rot < 90 deg)
  std::array<WheelCommand, 4> cmds_80;
  for (int i = 0; i < 4; ++i) cmds_80[i] = {1.396, 1.0, 1.0 / 0.035};
  auto opt_80 = kinematics.optimize_wheel_commands(cmds_80, current_angles, -M_PI, M_PI, false);
  for (int i = 0; i < 4; ++i)
  {
    EXPECT_GT(opt_80[i].drive_velocity, 0.0);
    EXPECT_NEAR(opt_80[i].steering_angle, 1.396, 1e-3);
    current_angles[i] = opt_80[i].steering_angle;
  }

  // 2. Small step across 90 deg relative to initial (to 95 deg, 1.658 rad).
  // Relative to current_angles (~80 deg), the difference is only 15 deg. No flip should occur.
  std::array<WheelCommand, 4> cmds_95;
  for (int i = 0; i < 4; ++i) cmds_95[i] = {1.658, 1.0, 1.0 / 0.035};
  auto opt_95 = kinematics.optimize_wheel_commands(cmds_95, current_angles, -M_PI, M_PI, false);
  for (int i = 0; i < 4; ++i)
  {
    EXPECT_GT(opt_95[i].drive_velocity, 0.0);
    EXPECT_NEAR(opt_95[i].steering_angle, 1.658, 1e-3);
  }
}

TEST(SwerveDriveKinematicsTest, test_deadband_standstill_preservation)
{
  SwerveDriveKinematics kinematics;
  kinematics.calculate_wheel_position(0.36, 0.40);

  // Command moving diagonally at 45 deg
  auto cmds = kinematics.compute_wheel_commands(1.0, 1.0, 0.0, 0.035);
  std::array<double, 4> current_angles = {{0.0, 0.0, 0.0, 0.0}};
  auto opt_cmds = kinematics.optimize_wheel_commands(cmds, current_angles, -M_PI, M_PI, false);

  for (size_t i = 0; i < 4; ++i)
  {
    EXPECT_NEAR(opt_cmds[i].steering_angle, M_PI_4, 1e-3);
    current_angles[i] = opt_cmds[i].steering_angle;
  }

  // Now command negligible velocity (< 0.02 m/s deadband)
  auto stop_cmds = kinematics.compute_wheel_commands(0.005, 0.0, 0.0, 0.035);
  auto stop_opt = kinematics.optimize_wheel_commands(stop_cmds, current_angles, -M_PI, M_PI, false);

  for (size_t i = 0; i < 4; ++i)
  {
    // Must preserve current angle (~45 deg) instead of jumping to 0.0
    EXPECT_NEAR(stop_opt[i].steering_angle, M_PI_4, 1e-3);
    EXPECT_DOUBLE_EQ(stop_opt[i].drive_velocity, 0.0);
    EXPECT_DOUBLE_EQ(stop_opt[i].drive_angular_velocity, 0.0);
  }
}

TEST(SwerveDriveKinematicsTest, test_closed_loop_odometry_loop_closure)
{
  SwerveDriveKinematics kinematics;
  kinematics.calculate_wheel_position(0.36, 0.40);

  // 1. Square path: 1m forward, 1m left, 1m backward, 1m right
  double dt = 0.01;
  int steps_per_leg = 100; // 100 steps * 0.01s * 1.0 m/s = 1.0 m

  // Leg 1: +X
  for (int s = 0; s < steps_per_leg; ++s) {
    kinematics.update_odometry({{1.0, 1.0, 1.0, 1.0}}, {{0.0, 0.0, 0.0, 0.0}}, dt);
  }
  // Leg 2: +Y
  for (int s = 0; s < steps_per_leg; ++s) {
    kinematics.update_odometry({{1.0, 1.0, 1.0, 1.0}}, {{M_PI_2, M_PI_2, M_PI_2, M_PI_2}}, dt);
  }
  // Leg 3: -X
  for (int s = 0; s < steps_per_leg; ++s) {
    kinematics.update_odometry({{-1.0, -1.0, -1.0, -1.0}}, {{0.0, 0.0, 0.0, 0.0}}, dt);
  }
  // Leg 4: -Y
  for (int s = 0; s < steps_per_leg; ++s) {
    kinematics.update_odometry({{-1.0, -1.0, -1.0, -1.0}}, {{M_PI_2, M_PI_2, M_PI_2, M_PI_2}}, dt);
  }

  // End of square loop: must be at (0, 0, 0)
  auto state = kinematics.update_odometry({{0.0, 0.0, 0.0, 0.0}}, {{0.0, 0.0, 0.0, 0.0}}, 0.0);
  EXPECT_NEAR(state.x, 0.0, 1e-5);
  EXPECT_NEAR(state.y, 0.0, 1e-5);
  EXPECT_NEAR(state.theta, 0.0, 1e-5);

  // 2. Pure in-place spin: 5 full rotations (10*pi rad) at 5.0 rad/s
  SwerveDriveKinematics kin_spin;
  kin_spin.calculate_wheel_position(0.36, 0.40);

  double omega = 5.0;
  auto spin_cmds = kin_spin.compute_wheel_commands(0.0, 0.0, omega, 0.035);
  std::array<double, 4> v_spin, th_spin;
  for (int i = 0; i < 4; ++i) {
    v_spin[i] = spin_cmds[i].drive_velocity;
    th_spin[i] = spin_cmds[i].steering_angle;
  }

  double total_time = 2.0 * M_PI * 5.0 / omega; // 5 full turns
  int spin_steps = 1000;
  double dt_spin = total_time / spin_steps;
  for (int s = 0; s < spin_steps; ++s) {
    kin_spin.update_odometry(v_spin, th_spin, dt_spin);
  }
  auto final_spin = kin_spin.update_odometry({{0,0,0,0}}, th_spin, 0.0);
  EXPECT_NEAR(final_spin.x, 0.0, 1e-4);
  EXPECT_NEAR(final_spin.y, 0.0, 1e-4);
  EXPECT_NEAR(final_spin.theta, 0.0, 1e-6);
}

TEST(SwerveDriveKinematicsTest, test_robot_physical_limits_250_degrees)
{
  SwerveDriveKinematics kinematics;
  kinematics.calculate_wheel_position(0.36, 0.40);

  const double limit = 4.363323129985824; // 250 degrees

  std::array<double, 4> current_angles = {{0.0, 0.0, 0.0, 0.0}};

  // Sweep full 720 degrees (2 full revolutions) in bounded mode
  for (int deg = -360; deg <= 360; deg += 5)
  {
    double rad = deg * M_PI / 180.0;
    double vx = 1.5 * std::cos(rad);
    double vy = 1.5 * std::sin(rad);

    auto raw_cmds = kinematics.compute_wheel_commands(vx, vy, 0.0, 0.035);
    auto opt_cmds = kinematics.optimize_wheel_commands(raw_cmds, current_angles, -limit, limit, true);

    for (int i = 0; i < 4; ++i)
    {
      // Angle must NEVER exceed +/- 250 deg limit
      EXPECT_GE(opt_cmds[i].steering_angle, -limit - 1e-4);
      EXPECT_LE(opt_cmds[i].steering_angle, limit + 1e-4);

      // Force vector must match desired (vx, vy)
      double vx_actual = opt_cmds[i].drive_velocity * std::cos(opt_cmds[i].steering_angle);
      double vy_actual = opt_cmds[i].drive_velocity * std::sin(opt_cmds[i].steering_angle);
      EXPECT_NEAR(vx_actual, vx, 1e-3);
      EXPECT_NEAR(vy_actual, vy, 1e-3);

      current_angles[i] = opt_cmds[i].steering_angle;
    }
  }
}

TEST(SwerveDriveKinematicsTest, test_degraded_3_wheel_fault_tolerant_odometry)
{
  SwerveDriveKinematics kinematics;
  kinematics.calculate_wheel_position(0.36, 0.40);

  const double target_vx = 1.2;
  const double target_vy = 0.8;
  const double target_wz = 0.5;
  const double radius = 0.035;

  auto cmds = kinematics.compute_wheel_commands(target_vx, target_vy, target_wz, radius);
  std::array<double, 4> v{};
  std::array<double, 4> th{};
  for (int i = 0; i < 4; ++i) {
    v[i] = cmds[i].drive_velocity;
    th[i] = cmds[i].steering_angle;
  }

  // 1. Single Wheel Failure test across ALL 4 wheels (FL=0, FR=1, RL=2, RR=3)
  for (int dead_wheel = 0; dead_wheel < 4; ++dead_wheel)
  {
    std::array<bool, 4> active_wheels = {{true, true, true, true}};
    active_wheels[dead_wheel] = false;

    // Simulate dead wheel velocity being 0.0
    auto v_with_dead = v;
    v_with_dead[dead_wheel] = 0.0;

    SwerveDriveKinematics kin_test;
    kin_test.calculate_wheel_position(0.36, 0.40);

    double dt = 0.01;
    for (int step = 0; step < 100; ++step) {
      kin_test.update_odometry(v_with_dead, th, dt, active_wheels);
    }
    auto odom = kin_test.update_odometry(v_with_dead, th, 0.0, active_wheels);

    // After 1.0 second at (1.2, 0.8, 0.5):
    // Odometry with 3 active wheels should track the trajectory with high fidelity!
    EXPECT_NEAR(odom.vx, target_vx, 1e-3) << "Failed for dead wheel: " << dead_wheel;
    EXPECT_NEAR(odom.vy, target_vy, 1e-3) << "Failed for dead wheel: " << dead_wheel;
    EXPECT_NEAR(odom.wz, target_wz, 1e-3) << "Failed for dead wheel: " << dead_wheel;
  }

  // 2. Dual Wheel Failure test (All 6 combinations of 2 active wheels)
  std::vector<std::pair<int, int>> dual_combinations = {
    {0, 1}, // Front 2 alive (FL + FR)
    {0, 2}, // Left 2 alive (FL + RL)
    {0, 3}, // Diagonal alive (FL + RR)
    {1, 2}, // Diagonal alive (FR + RL)
    {1, 3}, // Right 2 alive (FR + RR)
    {2, 3}  // Rear 2 alive (RL + RR)
  };

  for (const auto & [w1, w2] : dual_combinations)
  {
    std::array<bool, 4> active_wheels = {{false, false, false, false}};
    active_wheels[w1] = true;
    active_wheels[w2] = true;

    auto v_2wheel = v;
    for (int k = 0; k < 4; ++k) {
      if (!active_wheels[k]) v_2wheel[k] = 0.0;
    }

    SwerveDriveKinematics kin_test;
    kin_test.calculate_wheel_position(0.36, 0.40);

    for (int step = 0; step < 100; ++step) {
      kin_test.update_odometry(v_2wheel, th, 0.01, active_wheels);
    }
    auto odom = kin_test.update_odometry(v_2wheel, th, 0.0, active_wheels);
    EXPECT_NEAR(odom.vx, target_vx, 1e-3) << "Failed for pair: " << w1 << ", " << w2;
    EXPECT_NEAR(odom.vy, target_vy, 1e-3) << "Failed for pair: " << w1 << ", " << w2;
    EXPECT_NEAR(odom.wz, target_wz, 1e-3) << "Failed for pair: " << w1 << ", " << w2;
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
