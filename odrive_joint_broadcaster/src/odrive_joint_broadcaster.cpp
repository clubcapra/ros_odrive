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

#include "odrive_joint_broadcaster/odrive_joint_broadcaster.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"

namespace
{  // utility

// TODO(destogl): remove this when merged upstream
// Changed services history QoS to keep all so we don't lose any client service calls
static constexpr rmw_qos_profile_t rmw_qos_profile_services_hist_keep_all = {
  RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  1,  // message queue depth
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false};

}  // namespace

namespace odrive_joint_broadcaster
{

ODriveJointBroadcaster::ODriveJointBroadcaster() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn ODriveJointBroadcaster::on_init()
{

  try
  {
    param_listener_ = std::make_shared<odrive_joint_broadcaster::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ODriveJointBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  if (!params_.joints.empty())
  {
    joints_ = params_.joints;
  }

  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  try
  {
    // State publisher
    state_publisher_ =
      get_node()->create_publisher<ControllerStateMsg>("~/odrive_state", rclcpp::SystemDefaultsQoS());
    rt_state_publisher_ = std::make_unique<ControllerStatePublisher>(state_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Reserve memory for the publisher
  rt_state_publisher_->lock();
  rt_state_publisher_->msg_.header.frame_id = params_.joints[0];
  rt_state_publisher_->msg_.name.resize(joints_.size());
  rt_state_publisher_->msg_.bus_voltage.resize(joints_.size());
  rt_state_publisher_->msg_.bus_current.resize(joints_.size());
  rt_state_publisher_->msg_.fet_temperature.resize(joints_.size());
  rt_state_publisher_->msg_.motor_temperature.resize(joints_.size());
  rt_state_publisher_->msg_.active_errors.resize(joints_.size());
  rt_state_publisher_->msg_.disarm_reason.resize(joints_.size());
  rt_state_publisher_->unlock();

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ODriveJointBroadcaster::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // command_interfaces_config.names.reserve(params_.joints.size() * (int)joint_command_itfs::COUNT);
  // for (const auto & joint : params_.joints)
  // {
  //   for (const auto& itf : JointCommandITFS)
  //   {
  //     command_interfaces_config.names.push_back(joint + "/" + itf);
  //   }
  // }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration ODriveJointBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(joints_.size() * (int)joint_state_itfs::COUNT);
  for (const auto& joint : params_.joints)
  {
    for (const auto& itf : JointStateITFS)
    {
      state_interfaces_config.names.push_back(joint + "/" + itf);
    }
  }

  return state_interfaces_config;
}

controller_interface::CallbackReturn ODriveJointBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): if you have to manage multiple interfaces that need to be sorted check
  // `on_activate` method in `JointTrajectoryController` for exemplary use of
  // `controller_interface::get_ordered_interfaces` helper function

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ODriveJointBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
  // instead of a loop
  // for (size_t i = 0; i < command_interfaces_.size(); ++i)
  // {
  //   command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
  // }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ODriveJointBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{

  if (rt_state_publisher_ && rt_state_publisher_->trylock())
  {
    rt_state_publisher_->msg_.header.stamp = time;

    rt_state_publisher_->msg_.name.resize(joints_.size());

    rt_state_publisher_->msg_.bus_voltage.resize(joints_.size());
    rt_state_publisher_->msg_.bus_current.resize(joints_.size());
    rt_state_publisher_->msg_.fet_temperature.resize(joints_.size());
    rt_state_publisher_->msg_.motor_temperature.resize(joints_.size());
    rt_state_publisher_->msg_.active_errors.resize(joints_.size());
    rt_state_publisher_->msg_.disarm_reason.resize(joints_.size());
    for (const auto& itf : state_interfaces_)
    {
      auto match = std::find(joints_.begin(), joints_.end(), itf.get_prefix_name());
      if (match == joints_.end())
      {
        joints_.emplace_back(itf.get_prefix_name());
        rt_state_publisher_->msg_.name.emplace_back(itf.get_prefix_name());

        rt_state_publisher_->msg_.bus_voltage.resize(joints_.size());
        rt_state_publisher_->msg_.bus_current.resize(joints_.size());
        rt_state_publisher_->msg_.fet_temperature.resize(joints_.size());
        rt_state_publisher_->msg_.motor_temperature.resize(joints_.size());
        rt_state_publisher_->msg_.active_errors.resize(joints_.size());
        rt_state_publisher_->msg_.disarm_reason.resize(joints_.size());
      }
      auto index = joints_.begin() - match;


      if (itf.get_interface_name() == "bus_voltage") rt_state_publisher_->msg_.bus_voltage[index] = (float)itf.get_value();
      if (itf.get_interface_name() == "bus_current") rt_state_publisher_->msg_.bus_current[index] = (float)itf.get_value();
      if (itf.get_interface_name() == "fet_temperature") rt_state_publisher_->msg_.fet_temperature[index] = (float)itf.get_value();
      if (itf.get_interface_name() == "motor_temperature") rt_state_publisher_->msg_.motor_temperature[index] = (float)itf.get_value();
      if (itf.get_interface_name() == "active_errors") rt_state_publisher_->msg_.active_errors[index] = (uint32_t)itf.get_value();
      if (itf.get_interface_name() == "disarm_reason") rt_state_publisher_->msg_.disarm_reason[index] = (uint32_t)itf.get_value();
      
    }

    rt_state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace odrive_joint_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  odrive_joint_broadcaster::ODriveJointBroadcaster, controller_interface::ControllerInterface)
