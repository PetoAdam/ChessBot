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

#include <math.h>

#include <memory>

#include "moveit_example.hpp"
#include "chess_move_srv/srv/chess_move.hpp"

class ChessBot : public MoveitExample
{
public:
  void MoveInitPosition()
  {
    Eigen::Isometry3d init_pose = Eigen::Isometry3d(
      Eigen::Translation3d(0.3, 0.0, 0.35) * Eigen::Quaterniond(0, 1, 0, 0));
    auto drop_trajectory = planToPointUntilSuccess(
      init_pose, "ompl",
      "RRTConnectkConfigDefault");
    if (drop_trajectory != nullptr) {
      move_group_interface_->execute(*drop_trajectory);
    } else {
      RCLCPP_ERROR(LOGGER, "Planning failed");
    }
  }

  void MoveClashPosition()
  {
    Eigen::Isometry3d init_pose = Eigen::Isometry3d(
      Eigen::Translation3d(0.3, 0.3, 0.35) * Eigen::Quaterniond(0, 1, 0, 0));
    auto drop_trajectory = planToPointUntilSuccess(
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
      Eigen::Translation3d(request->from_x, request->from_y, 0.35) * Eigen::Quaterniond(0, 1, 0, 0));

    Eigen::Isometry3d from_pose = Eigen::Isometry3d(
      Eigen::Translation3d(request->from_x, request->from_y, request->from_z) * Eigen::Quaterniond(0, 1, 0, 0));

    Eigen::Isometry3d to_top_pose = Eigen::Isometry3d(
      Eigen::Translation3d(request->to_x, request->to_y, 0.35) * Eigen::Quaterniond(0, 1, 0, 0));

    Eigen::Isometry3d to_pose = Eigen::Isometry3d(
      Eigen::Translation3d(request->to_x, request->to_y, request->to_z) * Eigen::Quaterniond(0, 1, 0, 0));

    auto drop_trajectory = planToPoint(
      from_top_pose, "pilz_industrial_motion_planner", "LIN");
    if (drop_trajectory != nullptr) {
      move_group_interface_->execute(*drop_trajectory);
    } else {
      RCLCPP_ERROR(LOGGER, "Planning failed");
      return false;
    }

    this->moveGroupInterface()->setMaxVelocityScalingFactor(0.3);

    drop_trajectory = planToPoint(
      from_pose, "pilz_industrial_motion_planner", "LIN");
    if (drop_trajectory != nullptr) {
      move_group_interface_->execute(*drop_trajectory);
    } else {
      RCLCPP_ERROR(LOGGER, "Planning failed");
      return false;
    }

    //gripper on

    //wait bit

    this->moveGroupInterface()->setMaxVelocityScalingFactor(1.0); 

    drop_trajectory = planToPoint(
      from_top_pose, "pilz_industrial_motion_planner", "LIN");
    if (drop_trajectory != nullptr) {
      move_group_interface_->execute(*drop_trajectory);
    } else {
      RCLCPP_ERROR(LOGGER, "Planning failed");
      return false;
    }

    drop_trajectory = planToPoint(
      to_top_pose, "pilz_industrial_motion_planner", "LIN");
    if (drop_trajectory != nullptr) {
      move_group_interface_->execute(*drop_trajectory);
    } else {
      RCLCPP_ERROR(LOGGER, "Planning failed");
      return false;
    }

    this->moveGroupInterface()->setMaxVelocityScalingFactor(0.3);

    drop_trajectory = planToPoint(
      to_pose, "pilz_industrial_motion_planner", "LIN");
    if (drop_trajectory != nullptr) {
      move_group_interface_->execute(*drop_trajectory);
    } else {
      RCLCPP_ERROR(LOGGER, "Planning failed");
      return false;
    }

    //gripper off

    //wait bit

    this->moveGroupInterface()->setMaxVelocityScalingFactor(1.0); 

    drop_trajectory = planToPoint(
      to_top_pose, "pilz_industrial_motion_planner", "LIN");
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
  if(request->is_clash)
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
  std::thread(
    [&executor]()
    {executor.spin();})
  .detach();

  node->initialize();

  node->moveGroupInterface()->setMaxVelocityScalingFactor(1.0);
  node->moveGroupInterface()->setMaxAccelerationScalingFactor(1.0);
  // Add robot platform
  node->addRobotPlatform();

  rclcpp::Service<chess_move_srv::srv::ChessMove>::SharedPtr command_service = 
    node->create_service<chess_move_srv::srv::ChessMove>("chess command", &ChessCommandService);

  RCLCPP_INFO(node->get_logger(),"Ready for commands");

  // get it somehow to not shotdown
  RCLCPP_INFO(node->get_logger(),"Press a key, to exit");
  do
  {
  } while(std::cin.get() != '\n');
  
  

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
