from launch import LaunchDescription 
from launch_ros.actions  import Node 
from launch.actions  import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler 
from launch.event_handlers  import OnProcessExit 
from launch.launch_description_sources  import PythonLaunchDescriptionSource 
from ament_index_python.packages  import get_package_share_directory 
from launch.substitutions  import Command, FindExecutable, PathJoinSubstitution 
import os 
 
def generate_launch_description():
    # 获取包的路径 
    pkg_path = get_package_share_directory('arm_description')
    urdf_file = os.path.join(pkg_path,  'urdf', 'blueberry_arm1.urdf.xacro')  
    world_file = os.path.join(pkg_path,  'worlds', 'rome.world') 
    controllers_file = os.path.join(pkg_path,  'config', 'controllers.yaml') 
 
    # 1. 启动 Gazebo 
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('gazebo_ros'),
                'launch', 
                'gazebo.launch.py'  
            ])
        ]), 
        launch_arguments={
            'verbose': 'true',
            'world': world_file,
            'pause': 'false'
        }.items()
    )
 
    # 2. 将 URDF 加载到参数服务器 
    robot_description = {
        'robot_description': Command(['xacro ', urdf_file]), 
        'use_sim_time': True 
    }
 
    # 3. 启动 Robot State Publisher 
    robot_state_publisher = Node(
        package='robot_state_publisher', 
        executable='robot_state_publisher', 
        name='robot_state_publisher',
        output='screen', 
        parameters=[robot_description]
    )
 
    # 4. 在 Gazebo 中生成机器人模型 
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',   
        arguments=[
            '-entity', 'blueberry_arm', 
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ], 
        output='screen'
    )
 
    # 5. 检查controller_manager是否可用 
    try:
        get_package_share_directory('controller_manager')
        use_ros2_control = True 
    except:
        use_ros2_control = False 
        print("Warning: controller_manager not found, skipping controller loading")
 
    # 6. 仅当controller_manager可用时加载控制器 
    if use_ros2_control:
        control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[controllers_file],  # 移除了 robot_description
            output="screen",
        )
 
        load_controllers = [
            'joint_state_broadcaster',
            'base_rotation_controller',
            'joint1_controller',
            'joint2_controller',
            'joint3_controller'
        ]
 
        load_controller_actions = []
        for controller in load_controllers:
            load_controller_actions.append( 
                RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=spawn_entity,
                        on_exit=[
                            ExecuteProcess(
                                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', controller],
                                output='screen'
                            )
                        ]
                    )
                )
            )
    else:
        control_node = None 
        load_controller_actions = []
 
    # 7. 构建Launch描述 
    ld = LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])
 
    if use_ros2_control:
        ld.add_action(control_node) 
        for action in load_controller_actions:
            ld.add_action(action) 
 
    return ld 