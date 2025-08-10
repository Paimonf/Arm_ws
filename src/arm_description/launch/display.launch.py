from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('arm_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'blueberry_arm.urdf.xacro')
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf_file],
            parameters=[{
            'arm_description': Command(['xacro ', urdf_file]),  # 关键修改
            'use_sim_time': False
        }]),
            
        # Node(
        #     package='control_pkg',
        #     executable='simulation_joint_publisher',
        #     name='simulation_joint_publisher',
        #     output='screen'),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_path, 'rviz2', 'arm_config.rviz')])
    ])