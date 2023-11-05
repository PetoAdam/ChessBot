#include "chessbot/lbr3r760iisy_hardware.hpp"
#include "chessbot/simulator_control.pb.h"
#include <string>
#include <vector>

namespace chessbot
{
CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  joint_position_.assign(6, 0);
  joint_velocities_.assign(6, 0);
  joint_position_command_.assign(6, 0);
  joint_velocities_command_.assign(6, 0);
  gripper_position_ = 0.0;
  gripper_position_command_ = 0.0;

  for (const auto & joint : info_.joints)
  {
    if (joint.name != "finger_joint")
    {
      for (const auto & interface : joint.state_interfaces)
      {
        joint_interfaces[interface.name].push_back(joint.name);
      }
    }
    
  }

  gripper_interfaces["position"].push_back("finger_joint");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
  }

  state_interfaces.emplace_back("finger_joint", "position", &gripper_position_);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
  }

  command_interfaces.emplace_back("finger_joint", "position", &gripper_position_command_);

  return command_interfaces;
}

return_type RobotSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  chessbot_simulator_control::JointStates state = (chessbot_simulator_control::JointStates) controller.GetJointState();
  for (auto i = 0ul; i < 6; i++)
  {
    joint_position_[i] = state.states(i).position();
  }

  chessbot_simulator_control::GripperState gripper_state = (chessbot_simulator_control::GripperState) controller.GetGripperState();
  gripper_position_ = gripper_state.position();

  return return_type::OK;
}

return_type RobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  controller.SetJointPosition(joint_position_command_);
  controller.SetGripperPosition(gripper_position_command_);
  return return_type::OK;
}

}  // namespace chessbot

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  chessbot::RobotSystem, hardware_interface::SystemInterface)
