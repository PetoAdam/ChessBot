#include "chessbot/gripper_controller.hpp"

#include <stddef.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace chessbot
{
GripperController::GripperController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn GripperController::on_init()
{
  joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
  gripper_position_command_ = 0.0;
  command_interface_types_ =
    auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
  state_interface_types_ =
    auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration GripperController::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration conf = {controller_interface::interface_configuration_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size() * command_interface_types_.size());
  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : command_interface_types_)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return conf;
}

controller_interface::InterfaceConfiguration GripperController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf = {controller_interface::interface_configuration_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size() * state_interface_types_.size());
  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : state_interface_types_)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return conf;
}

controller_interface::CallbackReturn GripperController::on_configure(const rclcpp_lifecycle::State&)
{

  auto callback =
    [this](const std::shared_ptr<std_msgs::msg::Float64> gripper_msg) -> void
  {
    gripper_position_command_ = static_cast<double>(gripper_msg->data);
    new_msg_ = true;
  };

  gripper_command_subscriber_ =
    get_node()->create_subscription<std_msgs::msg::Float64>(
      "~/gripper_command", rclcpp::SystemDefaultsQoS(), callback);
  
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GripperController::on_activate(const rclcpp_lifecycle::State&)
{
  // assign command interfaces
  for (auto & interface : command_interfaces_)
  {
    command_interface_map_[interface.get_interface_name()]->push_back(interface);
  }

  // assign state interfaces
  for (auto & interface : state_interfaces_)
  {
    state_interface_map_[interface.get_interface_name()]->push_back(interface);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GripperController::on_deactivate(const rclcpp_lifecycle::State&)
{
  release_interfaces();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type GripperController::update(
  const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  gripper_position_command_interface_[0].get().set_value(gripper_position_command_);
  
  return controller_interface::return_type::OK;
}
}  // namespace chessbot

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  chessbot::GripperController, controller_interface::ControllerInterface)
