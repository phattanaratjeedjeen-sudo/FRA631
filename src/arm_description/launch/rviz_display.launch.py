import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription

def generate_launch_description():

    package_name = "arm_description"
    rviz_file_name = "config.rviz"

    # Paths
    rviz_file_path = os.path.join(get_package_share_directory(package_name), "rviz", rviz_file_name)

    # Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), "launch", "rsp.launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items()
    )

    # Set initial pose
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{
            'source_list': ['joint_states'],
            'zeros': {
                'joint_0': 1.5,
                'joint_1': 1.5,
                'joint_2': -1.5,
                'joint_3': 0.0,
                'joint_4': 0.0,
                'joint_5': 0.0,
            }
        }]
    )

    joint_state_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )
    # RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_file_path],
        output="screen"
    )

    # Create LaunchDescription
    launch_description = LaunchDescription()
    launch_description.add_action(rviz)
    launch_description.add_action(rsp)
    launch_description.add_action(joint_state_publisher)
    # launch_description.add_action(joint_state_gui)

    return launch_description