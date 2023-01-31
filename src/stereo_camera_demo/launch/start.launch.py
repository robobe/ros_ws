from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
from launch_ros.actions import Node

PACKAGE = "stereo_camera_demo"
WORLD = "basic.world"

def generate_launch_description():
    ld = LaunchDescription()

    pkg = get_package_share_directory(PACKAGE)
    gazebo_pkg = get_package_share_directory('gazebo_ros')

    verbose = LaunchConfiguration("verbose")
    arg_gazebo_verbose = DeclareLaunchArgument("verbose", default_value="true")
    world = LaunchConfiguration("world")
    arg_gazebo_world = DeclareLaunchArgument("world", default_value=WORLD)
    sim_time = LaunchConfiguration("sim_time")
    arg_sim_time = DeclareLaunchArgument("sim_time", default_value="true")

    resources = [os.path.join(pkg, "worlds")]

    resource_env = AppendEnvironmentVariable(
        name="GAZEBO_RESOURCE_PATH", value=":".join(resources)
    )

    models = [os.path.join(pkg, "models")]

    models_env = AppendEnvironmentVariable(
        name="GAZEBO_MODEL_PATH", value=":".join(models)
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    gazebo_pkg, 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'verbose': verbose, "world": world}.items()
             )

    stereo_camera = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    pkg, 'launch', 'camera.launch.py')]),
                    launch_arguments={'sim_time': sim_time}.items()
             )

    world_env = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    pkg, 'launch', 'env.launch.py')])
             )

    rviz_node = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(pkg, 'config', 'rviz.rviz')]
        )

    ld.add_action(arg_sim_time)
    ld.add_action(models_env)
    ld.add_action(resource_env)
    ld.add_action(arg_gazebo_verbose)
    ld.add_action(arg_gazebo_world)
    ld.add_action(gazebo)
    ld.add_action(stereo_camera)
    ld.add_action(world_env)
    ld.add_action(rviz_node)
    
    return ld
