#include "chessbot/RobotController.hpp"

RobotController::RobotController(int numberOfAxes, std::string simulatorIp){
        this->numberOfAxes = numberOfAxes;
        this->zmqClient = new ZMQClient(simulatorIp);
}

RobotController::RobotController(int numberOfAxes, const char* simulatorIp){
        this->numberOfAxes = numberOfAxes;
        this->zmqClient = new ZMQClient(simulatorIp);
}

// -------------------------------- JOINTS --------------------------------------------------------

bool RobotController::SetJointPosition(std::vector<double> positions){
    if((int) positions.size() != numberOfAxes){
        std::cout << "Target position axis count mismatch:\n\tThe target position axis count: " << sizeof(positions)/sizeof(double) << " and the number of axes: " << numberOfAxes << " do not match." << std::endl;
        return false;
    }
    
    // Define message
    chessbot_simulator_control::Request_MessageType messageType;
    chessbot_simulator_control::Request req;
    req.MessageType_Parse("SET_JOINT_POSITION", &messageType);
    req.set_type(messageType);

    for(int i = 0; i < numberOfAxes; i++){ req.mutable_set_joint_position()->add_position(positions[i]); }

    // Send message
    std::string data;
    req.SerializeToString(&data);
    //std::cout << "Sending joint position command..." << std::endl;
    zmqClient->send(data);

    // Receive reply
    std::string reply = zmqClient->recv();
    chessbot_simulator_control::Acknowledge ack;
    ack.ParseFromString(reply);
    //std::cout << "Response to position command: " << ack.acknowledge() << std::endl;
    return true;
    
}

bool RobotController::SetJointVelocity(std::vector<double> velocities){
    if((int) velocities.size() != numberOfAxes){
        std::cout << "Target velocity axis count mismatch:\n\tThe target velocity axis count: " << sizeof(velocities)/sizeof(double) << " and the number of axes: " << numberOfAxes << " do not match." << std::endl;
        return false;
    }
    
    // Define message
    chessbot_simulator_control::Request_MessageType messageType;
    chessbot_simulator_control::Request req;
    req.MessageType_Parse("SET_JOINT_VELOCITY", &messageType);
    req.set_type(messageType);

    for(int i = 0; i < numberOfAxes; i++){ req.mutable_set_joint_velocity()->add_velocity(velocities[i]); }

    // Send message
    std::string data;
    req.SerializeToString(&data);
    //std::cout << "Sending joint velocity command..." << std::endl;
    zmqClient->send(data);

    // Receive reply
    std::string reply = zmqClient->recv();
    chessbot_simulator_control::Acknowledge ack;
    ack.ParseFromString(reply);
    //std::cout << "Response to velocity command: " << ack.acknowledge() << std::endl;
    return true;
    
}

bool RobotController::SetJointTorque(std::vector<double> torques){
    if((int)torques.size() != numberOfAxes){
        std::cout << "Target torque axis count mismatch:\n\tThe target torque axis count: " << sizeof(torques)/sizeof(double) << " and the number of axes: " << numberOfAxes << " do not match." << std::endl;
        return false;
    }
    
    // Define message
    chessbot_simulator_control::Request_MessageType messageType;
    chessbot_simulator_control::Request req;
    req.MessageType_Parse("SET_JOINT_TORQUE", &messageType);
    req.set_type(messageType);

    for(int i = 0; i < numberOfAxes; i++){ req.mutable_set_joint_torque()->add_torque(torques[i]); }

    // Send message
    std::string data;
    req.SerializeToString(&data);
    //std::cout << "Sending joint torque command..." << std::endl;
    zmqClient->send(data);

    // Receive reply
    std::string reply = zmqClient->recv();
    chessbot_simulator_control::Acknowledge ack;
    ack.ParseFromString(reply);
    //std::cout << "Response to torque command: " << ack.acknowledge() << std::endl;
    return true;
    
}

bool RobotController::SetJointPositionByAxisId(int axisId, double position){
    // Define message
    chessbot_simulator_control::Request_MessageType messageType;
    chessbot_simulator_control::Request req;
    req.MessageType_Parse("SET_JOINT_POSITION_BY_AXIS_ID", &messageType);
    req.set_type(messageType);

    req.mutable_set_joint_position_by_axis_id()->set_axis_id(axisId);
    req.mutable_set_joint_position_by_axis_id()->set_position(position);

    // Send message
    std::string data;
    req.SerializeToString(&data);
    //std::cout << "Sending joint position command by axis id..." << std::endl;
    zmqClient->send(data);

    // Receive reply
    std::string reply = zmqClient->recv();
    chessbot_simulator_control::Acknowledge ack;
    ack.ParseFromString(reply);
    //std::cout << "Response to joint position command: " << ack.acknowledge() << std::endl;
    return true;
}

bool RobotController::SetJointVelocityByAxisId(int axisId, double velocity){
    // Define message
    chessbot_simulator_control::Request_MessageType messageType;
    chessbot_simulator_control::Request req;
    req.MessageType_Parse("SET_JOINT_VELOCITY_BY_AXIS_ID", &messageType);
    req.set_type(messageType);

    req.mutable_set_joint_velocity_by_axis_id()->set_axis_id(axisId);
    req.mutable_set_joint_velocity_by_axis_id()->set_velocity(velocity);

    // Send message
    std::string data;
    req.SerializeToString(&data);
    //std::cout << "Sending joint velocity command by axis id..." << std::endl;
    zmqClient->send(data);

    // Receive reply
    std::string reply = zmqClient->recv();
    chessbot_simulator_control::Acknowledge ack;
    ack.ParseFromString(reply);
    //std::cout << "Response to joint velocity command: " << ack.acknowledge() << std::endl;
    return true;
}
bool RobotController::SetJointTorqueByAxisId(int axisId, double torque){
    // Define message
    chessbot_simulator_control::Request_MessageType messageType;
    chessbot_simulator_control::Request req;
    req.MessageType_Parse("SET_JOINT_TORQUE_BY_AXIS_ID", &messageType);
    req.set_type(messageType);

    req.mutable_set_joint_torque_by_axis_id()->set_axis_id(axisId);
    req.mutable_set_joint_torque_by_axis_id()->set_torque(torque);

    // Send message
    std::string data;
    req.SerializeToString(&data);
    //std::cout << "Sending joint torque command by axis id..." << std::endl;
    zmqClient->send(data);

    // Receive reply
    std::string reply = zmqClient->recv();
    chessbot_simulator_control::Acknowledge ack;
    ack.ParseFromString(reply);
    //std::cout << "Response to joint torque command: " << ack.acknowledge() << std::endl;
    return true;
}

auto RobotController::GetJointState() -> chessbot_simulator_control::JointStates {
    if(numberOfAxes <= 0){
        std::cout << "Number of axes invalid: " << numberOfAxes << std::endl;
        throw std::out_of_range("Number of axes invalid.");
    }

    // Define message
    chessbot_simulator_control::Request_MessageType messageType;
    chessbot_simulator_control::Request req;
    req.MessageType_Parse("GET_JOINT_STATE", &messageType);
    req.set_type(messageType);

    // Send message
    std::string data;
    req.SerializeToString(&data);
    zmqClient->send(data);

    // Receive reply
    std::string reply = zmqClient->recv();
    chessbot_simulator_control::JointStates jointStates;
    jointStates.ParseFromString(reply);
    
    return jointStates;
}

auto RobotController::GetJointStateByAxisId(int axisId) -> chessbot_simulator_control::JointState {
    if(numberOfAxes <= 0){
        std::cout << "Number of axes invalid: " << numberOfAxes << std::endl;
        throw std::out_of_range("Number of axes invalid.");
    }

    // Define message
    chessbot_simulator_control::Request_MessageType messageType;
    chessbot_simulator_control::Request req;
    req.MessageType_Parse("GET_JOINT_STATE_BY_AXIS_ID", &messageType);
    req.set_type(messageType);

    req.mutable_get_joint_state_by_axis_id()->set_axis_id(axisId);

    // Send message
    std::string data;
    req.SerializeToString(&data);
    zmqClient->send(data);

    // Receive reply
    std::string reply = zmqClient->recv();
    chessbot_simulator_control::JointState jointState;
    jointState.ParseFromString(reply); 

    return jointState;
}

// -------------------------------- GRIPPER --------------------------------------------------------

bool RobotController::SetGripperPosition(double position){ 
    // Define message
    chessbot_simulator_control::Request_MessageType messageType;
    chessbot_simulator_control::Request req;
    req.MessageType_Parse("SET_GRIPPER_POSITION", &messageType);
    req.set_type(messageType);

    req.mutable_set_gripper_position()->set_position(position);

    // Send message
    std::string data;
    req.SerializeToString(&data);
    //std::cout << "Sending gripper position command..." << std::endl;
    zmqClient->send(data);

    // Receive reply
    std::string reply = zmqClient->recv();
    chessbot_simulator_control::Acknowledge ack;
    ack.ParseFromString(reply);
    //std::cout << "Response to position command: " << ack.acknowledge() << std::endl;
    return true;
    
}

auto RobotController::GetGripperState() -> chessbot_simulator_control::GripperState {

    // Define message
    chessbot_simulator_control::Request_MessageType messageType;
    chessbot_simulator_control::Request req;
    req.MessageType_Parse("GET_GRIPPER_STATE", &messageType);
    req.set_type(messageType);

    // Send message
    std::string data;
    req.SerializeToString(&data);
    zmqClient->send(data);

    // Receive reply
    std::string reply = zmqClient->recv();
    chessbot_simulator_control::GripperState gripperState;
    gripperState.ParseFromString(reply); 

    return gripperState;
}

// -------------------------------- MISC --------------------------------------------------------


double RobotController::GetToleranceValue(){ return tolerance; }

RobotController::~RobotController(){
    delete zmqClient;
}