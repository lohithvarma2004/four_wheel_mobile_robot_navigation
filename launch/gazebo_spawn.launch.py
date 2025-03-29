import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():
    modelrelativepath = 'model/robot.xacro'
    robotname = "prime"
    packagename = "four_wheel_mobile_robot_navigation"
    pathmodelfile = os.path.join(get_package_share_directory(packagename), modelrelativepath)

    # Process the xacro file to get the robot description
    robotdescription = xacro.process_file(pathmodelfile).toxml()
    
    # Launch Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        )
    )

    # Spawn the robot into Gazebo
    spawnnode = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=['-topic', 'robot_description', '-entity', robotname],
        output="screen"
    )

    # Launch robot_state_publisher
    statenode = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robotdescription, "use_sim_time": True}],
        output="screen"
    )
    

    # Delay loading controllers to allow the controller manager to fully initialize
    load_joint_state_broadcaster = TimerAction(
        period=3.0,  # Wait for 3 seconds
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
                output="screen"
            )
        ]
    )

    load_joint_trajectory_controller = TimerAction(
        period=3.5,  # Wait a bit longer
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_trajectory_controller'],
                output="screen"
            )
        ]
    )

    gzserver_proc = ExecuteProcess(
    cmd=[
        'gzserver',
        '/opt/ros/humble/share/gazebo_ros/worlds/empty.world',
        '-slibgazebo_ros_init.so',
        '-slibgazebo_ros_factory.so'
    ],
    output='screen')

    ld = LaunchDescription()
    ld.add_action(gazebo_launch)
    ld.add_action(spawnnode)
    ld.add_action(statenode)
    ld.add_action(load_joint_state_broadcaster)
    ld.add_action(load_joint_trajectory_controller)
    #ld.add_action(gzserver_proc)

    return ld