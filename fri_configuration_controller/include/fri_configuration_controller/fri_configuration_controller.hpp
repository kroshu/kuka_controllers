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


#ifndef KUKA_CONTROLLERS__TIMING_CONTROLLER_HPP_
#define KUKA_CONTROLLERS__TIMING_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"
#include "kuka_driver_interfaces/srv/set_int.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace kuka_controllers
{
class TimingController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state)
  override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state)
  override;

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state)
  override;

  controller_interface::CallbackReturn on_init() override;

private:
  rclcpp::Service<kuka_driver_interfaces::srv::SetInt>::SharedPtr receive_multiplier_service_;
  int receive_multiplier_ = 1;
  bool resend_multiplier_ = false;
};
}  // namespace kuka_controllers
#endif  // KUKA_CONTROLLERS__TIMING_CONTROLLER_HPP_
