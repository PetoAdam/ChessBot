#include <math.h>

#include <memory>

#include "moveit_example.hpp"
#include "chess_move_srv/srv/chess_move.hpp"
#include "std_msgs/msg/float64.hpp"

class ChessBot : public MoveitExample
{
public:
  const float top_z = 0.3;
  const float gripper_length = 0.18;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gripper_publisher;

  void GripperCommand(double gripper_value)
  {
    std_msgs::msg::Float64 msg;
    msg.data = gripper_value;
    gripper_publisher->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));  // Wait for 1 second for the gripper
  }

  void MoveInitPosition()
  {
    Eigen::Isometry3d init_pose = Eigen::Isometry3d(
      Eigen::Translation3d(0.4, 0.0, top_z) * Eigen::Quaterniond(0, 0, 0, 1));
    auto drop_trajectory = planToPointUntilSuccess(
      init_pose);
    if (drop_trajectory != nullptr) {
      move_group_interface_->execute(*drop_trajectory);
    } else {
      RCLCPP_ERROR(LOGGER, "Planning failed");
    }
  }

  void MoveClashPosition()
  {
    Eigen::Isometry3d init_pose = Eigen::Isometry3d(
      Eigen::Translation3d(0.3, 0.3, top_z) * Eigen::Quaterniond(0, 0, 0, 1));
    auto drop_trajectory = planToPoint(
      init_pose);
    if (drop_trajectory != nullptr) {
      move_group_interface_->execute(*drop_trajectory);
    } else {
      RCLCPP_ERROR(LOGGER, "Planning failed");
    }
  }

  bool NormalMoveCommand(const std::shared_ptr<chess_move_srv::srv::ChessMove::Request> request)
  {
    Eigen::Isometry3d from_top_pose = Eigen::Isometry3d(
      Eigen::Translation3d(request->from_x, request->from_y, top_z) * Eigen::Quaterniond(0, 0, 0, 1));

    Eigen::Isometry3d from_pose = Eigen::Isometry3d(
      Eigen::Translation3d(request->from_x, request->from_y, request->from_z + gripper_length) * Eigen::Quaterniond(0, 0, 0, 1));

    Eigen::Isometry3d to_top_pose = Eigen::Isometry3d(
      Eigen::Translation3d(request->to_x, request->to_y, top_z) * Eigen::Quaterniond(0, 0, 0, 1));

    Eigen::Isometry3d to_pose = Eigen::Isometry3d(
      Eigen::Translation3d(request->to_x, request->to_y, request->to_z + gripper_length) * Eigen::Quaterniond(0, 0, 0, 1));

    auto drop_trajectory = planToPoint(
      from_top_pose, "pilz_industrial_motion_planner", "PTP");
    if (drop_trajectory != nullptr) {
      move_group_interface_->execute(*drop_trajectory);
    } else {
      RCLCPP_ERROR(LOGGER, "Planning failed");
      return false;
    }

    this->moveGroupInterface()->setMaxVelocityScalingFactor(0.3);

    drop_trajectory = planToPoint(
      from_pose, "pilz_industrial_motion_planner", "PTP");
    if (drop_trajectory != nullptr) {
      move_group_interface_->execute(*drop_trajectory);
    } else {
      RCLCPP_ERROR(LOGGER, "Planning failed");
      return false;
    }

    //gripper on
    GripperCommand(0.6);

    this->moveGroupInterface()->setMaxVelocityScalingFactor(1.0); 

    drop_trajectory = planToPoint(
      from_top_pose, "pilz_industrial_motion_planner", "PTP");
    if (drop_trajectory != nullptr) {
      move_group_interface_->execute(*drop_trajectory);
    } else {
      RCLCPP_ERROR(LOGGER, "Planning failed");
      return false;
    }

    drop_trajectory = planToPoint(
      to_top_pose, "pilz_industrial_motion_planner", "PTP");
    if (drop_trajectory != nullptr) {
      move_group_interface_->execute(*drop_trajectory);
    } else {
      RCLCPP_ERROR(LOGGER, "Planning failed");
      return false;
    }

    //gripper off
    GripperCommand(0.4);

    // Currently dropping pieces instead of placing them (placing also works)
    /*this->moveGroupInterface()->setMaxVelocityScalingFactor(0.3);

    drop_trajectory = planToPoint(
      to_pose, "pilz_industrial_motion_planner", "PTP");
    if (drop_trajectory != nullptr) {
      move_group_interface_->execute(*drop_trajectory);
    } else {
      RCLCPP_ERROR(LOGGER, "Planning failed");
      return false;
    }
    */
    

    this->moveGroupInterface()->setMaxVelocityScalingFactor(1.0); 

    drop_trajectory = planToPoint(
      to_top_pose, "pilz_industrial_motion_planner", "PTP");
    if (drop_trajectory != nullptr) {
      move_group_interface_->execute(*drop_trajectory);
    } else {
      RCLCPP_ERROR(LOGGER, "Planning failed");
      return false;
    }

    MoveInitPosition();
    return true;
  }

  bool ClashMoveCommand(const std::shared_ptr<chess_move_srv::srv::ChessMove::Request> /*request*/)
  {
    return false;
  }
};

std::shared_ptr<ChessBot> node;

void ChessCommandService(const std::shared_ptr<chess_move_srv::srv::ChessMove::Request> request,
                          std::shared_ptr<chess_move_srv::srv::ChessMove::Response> response)
{
  if(!request->is_clash)
  {
    if(node->NormalMoveCommand(request))
    {
      response->set__success(true);
    }
    else
    {
      response->set__success(false);
      node->MoveInitPosition();
    }
  }
  else
  {
    if(node->ClashMoveCommand(request))
    {
      response->set__success(true);
    }
    else
    {
      response->set__success(false);
      node->MoveInitPosition();
    }
  }
}

int main(int argc, char * argv[])
{
  // Setup
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  node = std::make_shared<ChessBot>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  RCLCPP_INFO(node->get_logger(),"Starting rclcpp spinner...");
  std::thread(
    [&executor]()
    {executor.spin();})
  .detach();

  node->initialize();
  node->MoveInitPosition();

  // Create a publisher for the gripper control
  node->gripper_publisher = node->create_publisher<std_msgs::msg::Float64>("/gripper_controller/gripper_command", 10);

  node->moveGroupInterface()->setMaxVelocityScalingFactor(1.0);
  node->moveGroupInterface()->setMaxAccelerationScalingFactor(1.0);
  // Add robot platform
  //node->addRobotPlatform();

  rclcpp::Service<chess_move_srv::srv::ChessMove>::SharedPtr command_service = 
    node->create_service<chess_move_srv::srv::ChessMove>("chess_command", &ChessCommandService);

  RCLCPP_INFO(node->get_logger(),"Ready for commands");

  // get it somehow to not shotdown
  RCLCPP_INFO(node->get_logger(),"Press enter to exit");
  do
  {
  } while(std::cin.get() != '\n');
  
  

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
