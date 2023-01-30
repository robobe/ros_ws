from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "cyruskhan", 
            "-file", "/home/user/ros_ws/src/stereo_camera_demo/models/CyrusKhan/model.sdf",
            "-x", "1.0",
            "-Y", "0.7"
        ],
        output="screen",
    )

    ld.add_action(spawn_entity)

    return ld