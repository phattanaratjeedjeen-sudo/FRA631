import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    launch_description = LaunchDescription()

    pkg_path = os.path.join(get_package_share_directory('arm_description'))
    xacro_file = os.path.join(pkg_path, 'urdf', 'my-robot.xacro.urdf')
    robot_description_config = xacro.process_file(xacro_file)
    params = {'robot_description': robot_description_config.toxml()}

    robots = [
        ('joint_space', 'arm_joint.py', 'tria', os.path.join(pkg_path, 'rviz', 'joint_space.rviz')),
        ('task_space', 'arm_task.py', 'rec', os.path.join(pkg_path, 'rviz', 'task_space.rviz')),
    ]

    for ns, executable, task_name, rviz_config in robots:
        launch_description.add_action(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=ns,
            output='screen',
            parameters=[params, {'frame_prefix': f'{ns}/'}],
        ))

        launch_description.add_action(Node(
            package='arm_description',
            executable=executable,
            namespace=ns,
            name='arm',
            output='screen',
            parameters=[{'task': task_name}],
        ))

        launch_description.add_action(Node(
            package='rviz2',
            executable='rviz2',
            name=f'{ns}_rviz',
            output='screen',
            arguments=['-d', rviz_config],
        ))

    return launch_description