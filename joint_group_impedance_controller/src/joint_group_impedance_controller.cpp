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

#include "controller_interface/controller_interface.hpp"

#include "joint_group_impedance_controller/joint_group_impedance_controller.hpp"

namespace kuka_controllers
{
controller_interface::CallbackReturn JointGroupImpedanceController::on_init()
{
  auto ret = MultiInterfaceForwardCommandController::on_init();
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }

  try {
    // Explicitly set the interfaces parameter declared by the forward_command_controller
    get_node()->set_parameter(
      rclcpp::Parameter("interface_names", std::vector<std::string>{"stiffness", "damping"}));
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

}  // namespace kuka_controllers

PLUGINLIB_EXPORT_CLASS(
  kuka_controllers::JointGroupImpedanceController,
  controller_interface::ControllerInterface)
