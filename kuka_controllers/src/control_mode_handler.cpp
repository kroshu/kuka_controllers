// Copyright 2022 Áron Svastits
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

#include "kuka_controllers/control_mode_handler.hpp"

namespace kuka_controllers
{
controller_interface::CallbackReturn ControlModeHandler::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ControlModeHandler::command_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.emplace_back("runtime_config/control_mode");
  return config;
}

controller_interface::InterfaceConfiguration ControlModeHandler::state_interface_configuration()
const
{
  return controller_interface::InterfaceConfiguration{controller_interface::
    interface_configuration_type::NONE};
}

controller_interface::CallbackReturn
ControlModeHandler::on_configure(const rclcpp_lifecycle::State &)
{
  control_mode_subscriber_ = get_node()->create_subscription<std_msgs::msg::UInt32>(
    "control_mode", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::UInt32::SharedPtr msg) {
      control_mode_ = kuka_motion_external_ExternalControlMode(msg->data);
      RCLCPP_INFO(get_node()->get_logger(), "Control mode changed to %u", msg->data);
    });
  RCLCPP_INFO(get_node()->get_logger(), "Control mode handler configured");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ControlModeHandler::on_activate(const rclcpp_lifecycle::State &)
{
  if (control_mode_ <= nanopb::kuka::motion::external::ExternalControlMode::kUnspecified) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Control mode unspecified, It is set to position control");
    control_mode_ = nanopb::kuka::motion::external::ExternalControlMode::kPositionControl;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ControlModeHandler::on_deactivate(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ControlModeHandler::update(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  command_interfaces_[0].set_value(static_cast<double>(control_mode_));
  return controller_interface::return_type::OK;
}

}  // namespace kuka_controllers

PLUGINLIB_EXPORT_CLASS(
  kuka_controllers::ControlModeHandler,
  controller_interface::ControllerInterface)
