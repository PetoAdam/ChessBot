#ifndef CHESSBOT__LBR3R760IISY_HARDWARE_HPP_
#define CHESSBOT__LBR3R760IISY_HARDWARE_HPP_

#include "string"
#include "unordered_map"
#include "vector"
#include "RobotController.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using hardware_interface::return_type;

namespace chessbot
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HARDWARE_INTERFACE_PUBLIC RobotSystem : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & /*period*/) override;

  return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

protected:

  // TODO: Maybe replace the hardcoded "6", e.g. DI
  RobotController controller = RobotController(6, "tcp://localhost:5555");

  /// The size of this vector is (standard_interfaces_.size() x nr_joints)
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocities_command_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocities_;

  double gripper_position_command_;
  double gripper_position_;

  std::unordered_map<std::string, std::vector<std::string>> joint_interfaces = {
    {"position", {}}, {"velocity", {}}};

  std::unordered_map<std::string, std::vector<std::string>> gripper_interfaces = {
    {"position", {}}
  };
    
};

}  // namespace chessbot

#endif  // CHESSBOT__LBR3R760IISY_HARDWARE_HPP_
