from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():


    # 声明参数
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    
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
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )
    

    # 控制包的各种节点
    version_node = Node(
        package='test_pkg',
        executable='version_node',
        name='version_node'
    )
    
    start_node = Node(
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

    pathplan_moveit_node = Node(
        package='test_pkg',
        executable='pathplan_moveit_node',
        name='pathplan_moveit_node'
    )

    debug_image_relay = Node(
        package='test_pkg',
        executable='debug_image_relay',
        name='debug_image_relay'
    )

    return LaunchDescription([
        use_sim_time,
        orbbec_launch,
        arm_description_launch,
        # moveit_config_launch,  # 添加MoveIt配置启动
        version_node,
        # start_node,
        serial_node,
        # pathplan_moveit_node,
        pathplan_node,
        # debug_image_relay,
    ])