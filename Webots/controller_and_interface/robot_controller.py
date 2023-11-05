import sys
from controller import Robot
from concurrent import futures
import threading
import grpc
import ikpy
from ikpy import chain
import zmq
from enum import Enum
from time import perf_counter_ns
import simulator_control_pb2

DEG2RAD = 0.0174532925
RAD2DEG = 57.2957795

class bcolors:
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'

class zmq_time_sync_server():
    def __init__(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind("tcp://*:5556")
    
    def publish(self):
        # Heartbeat message - has no meaning
        self.socket.send(b"%d %d" % (1, 1))

class zmq_server():
    def __init__(self):
        
        self.context = zmq.Context()

        self.socket = self.context.socket(zmq.REP)
        self.socket.bind("tcp://*:5555")
        
        # This dictionary stores which function to execute upon receiving a zmq request
        self.function_container = {
            simulator_control_pb2.Request.MessageType.GET_ROBOT_PARAMETER: robot_object.os_get_robot_parameters,

            ## JOINTS ##
            simulator_control_pb2.Request.MessageType.SET_JOINT_POSITION: robot_object.os_set_target_joint_position,
            simulator_control_pb2.Request.MessageType.SET_JOINT_VELOCITY: robot_object.os_set_target_joint_velocity,
            simulator_control_pb2.Request.MessageType.SET_JOINT_TORQUE: robot_object.os_set_target_joint_torque,
            simulator_control_pb2.Request.MessageType.SET_JOINT_POSITION_BY_AXIS_ID: robot_object.os_set_target_joint_position_by_axis_id,
            simulator_control_pb2.Request.MessageType.SET_JOINT_VELOCITY_BY_AXIS_ID: robot_object.os_set_target_joint_velocity_by_axis_id,
            simulator_control_pb2.Request.MessageType.SET_JOINT_TORQUE_BY_AXIS_ID: robot_object.os_set_target_joint_torque_by_axis_id,

            simulator_control_pb2.Request.MessageType.GET_JOINT_STATE: robot_object.os_get_joint_state,
            simulator_control_pb2.Request.MessageType.GET_JOINT_STATE_BY_AXIS_ID: robot_object.os_get_joint_state_by_axis_id,
            
            ## GRIPPER ##
            simulator_control_pb2.Request.MessageType.SET_GRIPPER_POSITION: robot_object.os_set_gripper_position,
            simulator_control_pb2.Request.MessageType.GET_GRIPPER_STATE: robot_object.os_get_gripper_state,
            }

        self.zmq_loop()


    def zmq_loop(self):
        while enable_thread:
            try: request_unparsed = self.socket.recv(zmq.NOBLOCK)
            except: continue
            request = simulator_control_pb2.Request()
            request.ParseFromString(request_unparsed)
            thread_lock.acquire()
            result = self.function_container[request.type](request=request)
            thread_lock.release()
            self.socket.send(result.SerializeToString())

class control_mode(Enum):
    POSITION_CONTROL = 1
    VELOCITY_CONTROL = 2
    TORQUE_CONTROL = 3

class Robot_Object():
    def __init__(self, urdf_name):
        self.urdf_name = urdf_name
        self.TIME_STEP = 16
        self.robot = Robot()
        self.positions = []
        self.velocities = []
        self.torques = []
        self.gripper_position = None
        self.machine_data_dictionary = {
            "maximum_positions":[],
            "minimum_positions":[],
            "maximum_velocities":[],
            "minimum_velocities":[],
            "maximum_efforts":[],
            "minimum_efforts":[]
            }
        self.motors = []
        self.sensors = []
        self.gripper_motors = []
        self.gripper_sensors = []

        # Using position control by default
        self.control_mode = control_mode.POSITION_CONTROL

        armChain = ikpy.chain.Chain.from_urdf_file('resource/' + self.urdf_name)
        
        # Arm setup
        for i in range(len(armChain.links)):
            if(armChain.links[i].name.startswith('Joint')):
                motor = self.robot.getDevice(armChain.links[i].name)
                motor.enableTorqueFeedback(self.TIME_STEP)
                position_sensor = self.robot.getDevice(armChain.links[i].name+ '_sensor')
                position_sensor.enable(self.TIME_STEP)
                self.motors.append(motor)
                self.sensors.append(position_sensor)
                self.machine_data_dictionary["maximum_positions"].append(motor.getMaxPosition())
                self.machine_data_dictionary["minimum_positions"].append(motor.getMinPosition())
                self.machine_data_dictionary["maximum_velocities"].append(motor.getMaxVelocity())
                self.machine_data_dictionary["minimum_velocities"].append(0.0)
                self.machine_data_dictionary["maximum_efforts"].append(motor.getMaxTorque())
                self.machine_data_dictionary["minimum_efforts"].append(0.0)

        # Gripper setup
        gripper_joints = ['finger_joint', 'left_inner_knuckle_joint', 'left_inner_finger_joint', 'right_inner_knuckle_joint', 'right_inner_finger_joint', 'right_outer_knuckle_joint']
        for joint in gripper_joints:
            gripper_motor = self.robot.getDevice(joint)
            gripper_sensor = self.robot.getDevice(joint +  '_sensor')
            gripper_sensor.enable(self.TIME_STEP)
            self.gripper_motors.append(gripper_motor)
            self.gripper_sensors.append(gripper_sensor)
        
        # Default values
        self.axes_number = len(self.motors)

        self.target_positions = [None] * self.axes_number
        self.target_velocities = [None] * self.axes_number
        self.target_torques = [None] * self.axes_number
        self.target_gripper_position = None


## --------------------- JOINT FUNCTIONS -----------------------------------------
    def os_set_target_joint_position(self, request):
        #print("Received joint position target:")
        self.target_positions = [request.set_joint_position.position[i] for i in range(self.axes_number)]
        #print(self.target_positions)
        response = simulator_control_pb2.Acknowledge()
        response.acknowledge = "OK"
        self.control_mode = control_mode.POSITION_CONTROL
        return response

    def os_set_target_joint_velocity(self, request):
        #print("Received joint velocity target:")
        self.target_velocities = [request.set_joint_velocity.velocity[i] for i in range(self.axes_number)]
        #print(self.target_velocities)
        response = simulator_control_pb2.Acknowledge()
        response.acknowledge = "OK"
        self.control_mode = control_mode.VELOCITY_CONTROL
        return response

    def os_set_target_joint_torque(self, request):
        #print("Received joint torque target:")
        self.target_torques = [request.set_joint_torque.torque[i] for i in range(self.axes_number)]
        #print(self.target_torques)
        response = simulator_control_pb2.Acknowledge()
        response.acknowledge = "OK"
        self.control_mode = control_mode.TORQUE_CONTROL
        return response
    
    def os_set_target_joint_position_by_axis_id(self, request):
        #print("Received joint position target:")
        axis_id = request.set_joint_position_by_axis_id.axis_id
        self.target_positions[axis_id]=request.set_joint_position_by_axis_id.position
        #print(str(axis_id) + "\t" + str(self.target_positions[axis_id]))
        response = simulator_control_pb2.Acknowledge()
        response.acknowledge = "OK"
        self.control_mode = control_mode.POSITION_CONTROL
        return response

    def os_set_target_joint_velocity_by_axis_id(self, request):
        #print("Received joint velocity target:")
        axis_id = request.set_joint_position_by_axis_id.axis_id
        self.target_velocities[axis_id]=request.set_joint_velocity_by_axis_id.velocity
        #print(str(axis_id) + "\t" + str(self.target_velocities[axis_id]))
        response = simulator_control_pb2.Acknowledge()
        response.acknowledge = "OK"
        self.control_mode = control_mode.VELOCITY_CONTROL
        return response

    def os_set_target_joint_torque_by_axis_id(self, request):
        #print("Received joint torque target:")
        axis_id = request.set_joint_position_by_axis_id.axis_id
        self.target_torques[axis_id]=request.set_joint_torque_by_axis_id.torque
        #print(str(axis_id) + "\t" + str(self.target_torques[axis_id]))
        response = simulator_control_pb2.Acknowledge()
        response.acknowledge = "OK"
        self.control_mode = control_mode.TORQUE_CONTROL
        return response
    
    def os_get_joint_state(self, request):
        response = simulator_control_pb2.JointStates()
        for i in range(self.axes_number):
            joint_state = response.states.add()
            joint_state.axis_id = i
            joint_state.position = self.positions[i]
            joint_state.velocity = self.velocities[i]
            joint_state.torque = self.torques[i]
        return response
    
    def os_get_joint_state_by_axis_id(self, request):
        response = simulator_control_pb2.JointState()
        axis_id = request.get_joint_state_by_axis_id.axis_id
        response.axis_id = axis_id
        response.position = self.positions[axis_id]
        response.velocity = self.velocities[axis_id]
        response.torque = self.torques[axis_id]
        return response

## ------------------------ GRIPPER --------------------------------------------
    def os_set_gripper_position(self, request):
        #print("Received gripper position target:")
        # Set the gripper joint position
        self.target_gripper_position = request.set_gripper_position.position
        response = simulator_control_pb2.Acknowledge()
        response.acknowledge = "OK"
        return response

    def os_get_gripper_state(self, request):
        response = simulator_control_pb2.GripperState()
        response.position = self.gripper_position
        return response

    
## --------------------- MISCELLANEOUS -----------------------------------------

    def os_get_robot_parameters(self, request):
        response = simulator_control_pb2.RobotParameter()
        
        response.axes_number = self.axes_number

        for i in range(self.axes_number):
            position_limit = simulator_control_pb2.JointPositionLimit()
            position_limit.upper_limit = self.machine_data_dictionary["maximum_positions"][i]
            position_limit.lower_limit = self.machine_data_dictionary["minimum_positions"][i]

            velocity_limit = simulator_control_pb2.JointVelocityLimit()
            velocity_limit.upper_limit = self.machine_data_dictionary["maximum_velocities"][i]
            velocity_limit.lower_limit = self.machine_data_dictionary["minimum_velocities"][i]

            effort_limit = simulator_control_pb2.JointEffortLimit()
            effort_limit.upper_limit = self.machine_data_dictionary["maximum_efforts"][i]
            effort_limit.lower_limit = self.machine_data_dictionary["minimum_efforts"][i]

            joint_limit = simulator_control_pb2.JointLimit()

            joint_limit.type = simulator_control_pb2.JointLimit.JointType.ROTATIONAL
            joint_limit.position_limit.CopyFrom(position_limit)
            joint_limit.velocity_limit.CopyFrom(velocity_limit)
            joint_limit.effort_limit.CopyFrom(effort_limit)

            response.joint_limit.extend([joint_limit])

        return response
        

thread_lock = threading.Lock() # for multithreading
robot_object = Robot_Object(sys.argv[1])
enable_thread = True

def main():

    zmq_thread = threading.Thread(target = zmq_server)
    zmq_thread.start()


    zmq_heartbeat = zmq_time_sync_server()

    print(f"{bcolors.OKGREEN}ALL SERVERS ARE READY - STARTING SIMULATION{bcolors.ENDC}")


    while robot_object.robot.step(robot_object.TIME_STEP) != -1:

        thread_lock.acquire()
        robot_object.positions = [robot_object.sensors[i].getValue() for i in range(len(robot_object.motors))]
        robot_object.velocities = [robot_object.motors[i].getVelocity() for i in range(len(robot_object.motors))]
        robot_object.torques = [robot_object.motors[i].getTorqueFeedback() for i in range(len(robot_object.motors))]
        robot_object.gripper_position = robot_object.gripper_sensors[0].getValue()

        if(robot_object.control_mode is control_mode.POSITION_CONTROL):
            [robot_object.motors[i].setPosition(robot_object.target_positions[i]) if robot_object.target_positions[i] is not None else None for i in range(len(robot_object.motors))]
        if(robot_object.control_mode is control_mode.VELOCITY_CONTROL):
            for i in range(len(robot_object.motors)):
                robot_object.motors[i].setPosition(float('+inf'))
                robot_object.motors[i].setVelocity(robot_object.target_velocities[i]) if robot_object.target_velocities[i] is not None else None
        if(robot_object.control_mode is control_mode.TORQUE_CONTROL):
            [robot_object.motors[i].setTorque(robot_object.target_torques[i]) if robot_object.target_torques[i] is not None else None for i in range(len(robot_object.motors))]
            
        [robot_object.gripper_motors[i].setPosition(robot_object.target_gripper_position) if robot_object.target_gripper_position is not None else None for i in range(len(robot_object.gripper_motors))]

        thread_lock.release()

        zmq_heartbeat.publish()
    

if __name__ == '__main__':
    main()
    thread_lock.acquire()
    enable_thread = False
    thread_lock.release()
