from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # 导入 orbbec_camera 的 gemini.launch.xml - 使用 FrontendLaunchDescriptionSource
    orbbec_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('orbbec_camera'),
                'launch',
                'gemini.launch.xml'
            ])
        ])
    )
    
    # 导入 arm_description 的 display.launch.py - 使用 PythonLaunchDescriptionSource
    arm_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('arm_description'),
                'launch',
                'display.launch.py'
            ])
        ])
    )
    
    # 控制包的各种节点
    version_node = Node(
        package='test_pkg',
        executable='version_node',
        name='version_node'
    )
    
    start_node = Node(  # 修正了可执行文件名
        package='test_pkg',
        executable='start_node',
        name='start_node'
    )
    
    serial_node = Node(
        package='test_pkg',
        executable='serial_node',
        name='serial_node'
    )
    
    pathplan_node = Node(
        package='test_pkg',
        executable='pathplan_node',
        name='pathplan_node'
    )

    return LaunchDescription([
        orbbec_launch,
        arm_description_launch,
        version_node,
        start_node,
        serial_node,
        pathplan_node
    ])