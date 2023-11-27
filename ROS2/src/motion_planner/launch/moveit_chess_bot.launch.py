from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources.python_launch_description_source import PythonLaunchDescriptionSource  # noqa: E501
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration('robot_model')
    moveit_config = (
        MoveItConfigsBuilder("chessbot")
        .robot_description(file_path=get_package_share_directory('chessbot_moveit_config')
                           + "/config/{}.urdf.xacro".format(robot_model.perform(context)))
        .robot_description_semantic(get_package_share_directory('chessbot_moveit_config')  # noqa: E501
                                    + "/config/{}.srdf".format(robot_model.perform(context)))
        .robot_description_kinematics(file_path=get_package_share_directory('chessbot_moveit_config') + "/config/kinematics.yaml")
        .trajectory_execution(file_path=get_package_share_directory('chessbot_moveit_config') + "/config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"]
        )
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .joint_limits(file_path=get_package_share_directory('chessbot_moveit_config')
                      + "/config/joint_limits.yaml")
        .to_moveit_configs()
    )

    '''
    startup_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [get_package_share_directory('kuka_rox_hw_interface'), '/launch/startup.launch.py']),
        launch_arguments={'robot_model': "{}".format(robot_model.perform(context))}.items())
    '''
    
    move_group_server = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    to_start = [
        #startup_launch,
        move_group_server,
        
    ]

    return to_start


def generate_launch_description():
    launch_arguments = []
    launch_arguments.append(DeclareLaunchArgument(
        'robot_model',
        default_value='LBR3R760iisy'
    ))
    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])