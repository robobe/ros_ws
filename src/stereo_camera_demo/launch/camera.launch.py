from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import xacro
from launch_ros.actions import Node

PACKAGE = "stereo_camera_demo"
WORLD = "basic.world"
URDF = "stereo_camera.xacro"

def generate_launch_description():
    ld = LaunchDescription()

    pkg = get_package_share_directory(PACKAGE)
    sim_time = LaunchConfiguration("sim_time")
    arg_sim_time = DeclareLaunchArgument("sim_time", default_value="true")

    robot_description_path = os.path.join(pkg, "urdf", URDF)
    doc = xacro.parse(open(robot_description_path))
    xacro.process_doc(doc)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'use_sim_time': sim_time, 
                'robot_description': doc.toxml()
            }
        ]
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "demo", "-topic", "robot_description"],
        output="screen",
    )

    ld.add_action(arg_sim_time)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_entity)
    return ld
