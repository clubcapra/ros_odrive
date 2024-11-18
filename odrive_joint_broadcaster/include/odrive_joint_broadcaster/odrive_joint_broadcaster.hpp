// Copyright (c) 2024, capra
// Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
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

//
// Source of this file are templates in
// [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace) repository.
//

#ifndef ODRIVE_JOINT_BROADCASTER__ODRIVE_JOINT_BROADCASTER_HPP_
#define ODRIVE_JOINT_BROADCASTER__ODRIVE_JOINT_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "odrive_joint_broadcaster/visibility_control.h"
// #include "odrive_joint_broadcaster_parameters.hpp"
#include "../../../build/odrive_joint_broadcaster/odrive_joint_broadcaster_parameters/include/odrive_joint_broadcaster_parameters.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_msgs/msg/empty.hpp"

// Messages and services
#include "odrive_can/msg/o_drive_joint_state.hpp"
#include "odrive_can/srv/request_clear_errors.hpp"
#include "odrive_can/srv/request_reboot.hpp"

namespace odrive_joint_broadcaster
{
// name constants for state interfaces
enum class joint_state_itfs : std::uint8_t
{
  BUS_VOLTAGE = 0,
  BUS_CURRENT,
  FET_TEMPERATURE,
  MOTOR_TEMPERATURE,
  AXIS_STATE,
  ACTIVE_ERRORS,
  DISARM_REASON,
  COUNT
};

enum class joint_command_itfs : std::uint8_t
{
  REQUEST_CLEAR_ERRORS = 0,
  REQUEST_REBOOT,
  COUNT
};

const std::vector<std::string> JointStateITFS = {
  "bus_voltage",
  "bus_current",
  "fet_temperature",
  "motor_temperature",
  "axis_state",
  "active_errors",
  "disarm_reason",
};

const std::vector<std::string> StateITFS = {
  "average_voltage",
  "total_current",
};

const std::vector<std::string> JointCommandITFS = {
  "request_clear_errors",
  "request_reboot",
};

class ODriveJointBroadcaster : public controller_interface::ControllerInterface
{
public:
  ODRIVE_JOINT_BROADCASTER__VISIBILITY_PUBLIC
  ODriveJointBroadcaster();

  ODRIVE_JOINT_BROADCASTER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  ODRIVE_JOINT_BROADCASTER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  ODRIVE_JOINT_BROADCASTER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  ODRIVE_JOINT_BROADCASTER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  ODRIVE_JOINT_BROADCASTER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  ODRIVE_JOINT_BROADCASTER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  ODRIVE_JOINT_BROADCASTER__VISIBILITY_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  using ControllerStateMsg = odrive_can::msg::ODriveJointState;
  using RequestClearErrorsSrv = odrive_can::srv::RequestClearErrors;
  using RequestRebootSrv = odrive_can::srv::RequestReboot;

protected:
  std::shared_ptr<odrive_joint_broadcaster::ParamListener> param_listener_;
  odrive_joint_broadcaster::Params params_;

  std::vector<std::string> joints_;

  using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;

  rclcpp::Publisher<ControllerStateMsg>::SharedPtr state_publisher_;
  std::unique_ptr<ControllerStatePublisher> rt_state_publisher_;

};

}  // namespace odrive_joint_broadcaster

#endif  // ODRIVE_JOINT_BROADCASTER__ODRIVE_JOINT_BROADCASTER_HPP_
