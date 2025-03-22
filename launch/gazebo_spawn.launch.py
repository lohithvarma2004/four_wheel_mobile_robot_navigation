import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():
    modelrelativepath = 'model/robot.xacro'
    robotname = "prime"
    packagename = "four_wheel_mobile_robot_navigation"
    pathmodelfile = os.path.join(get_package_share_directory(packagename), modelrelativepath)

    robotdescription = xacro.process_file(pathmodelfile).toxml()
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        )
    )

    spawnnode = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=['-topic', 'robot_description', '-entity', robotname],
        output="screen"
    )

    statenode = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robotdescription, "use_sim_time": True}],
        output="screen"
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output="screen"
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_trajectory_controller'],
        output="screen"
    )

    obj = LaunchDescription()
    obj.add_action(gazebo_launch)
    obj.add_action(spawnnode)
    obj.add_action(statenode)
    obj.add_action(load_joint_state_broadcaster)
    obj.add_action(load_joint_trajectory_controller)  # Fixed typo here ✅

    return obj
