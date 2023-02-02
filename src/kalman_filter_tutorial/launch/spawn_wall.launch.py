import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription

PACKAGE = "kalman_filter_tutorial"

def generate_launch_description():
    ld = LaunchDescription()
    pkg_models = os.path.join(get_package_share_directory(PACKAGE), 'models')
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "brick_wall", 
            "-file", os.path.join(pkg_models, "brick_box_3x1x3", "model.sdf"),
            "-x", "11.5",
            "-y", "-0.5",
            "-z", "0.0",
            "-Y", "1.5708"
        ],
        output="screen",
    )

    ld.add_action(spawn_entity)

    return ld
