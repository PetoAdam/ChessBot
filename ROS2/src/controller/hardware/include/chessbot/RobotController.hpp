#ifndef ROBOTCONTROLLER_HPP
#define ROBOTCONTROLLER_HPP

#include "ZMQClient.hpp"
#include "simulator_control.pb.h"

class RobotController{
private:
    int numberOfAxes;
    const double tolerance = 0.01;
    ZMQClient* zmqClient;

public:
    RobotController(int numberOfAxes, std::string simulatorIp);
    RobotController(int numberOfAxes, const char* simulatorIp);

    // Joints
    bool SetJointPosition(std::vector<double> positions);
    bool SetJointVelocity(std::vector<double> velocities);
    bool SetJointTorque(std::vector<double> torques);
    bool SetJointPositionByAxisId(int axisId, double position);
    bool SetJointVelocityByAxisId(int axisId, double velocity);
    bool SetJointTorqueByAxisId(int axisId, double torque);
    chessbot_simulator_control::JointStates GetJointState();
    chessbot_simulator_control::JointState GetJointStateByAxisId(int axisId);

    // Gripper
    bool SetGripperPosition(double position);
    chessbot_simulator_control::GripperState GetGripperState();

    double GetToleranceValue();
    ~RobotController();
};

#endif //ROBOTCONTROLLER_HPP