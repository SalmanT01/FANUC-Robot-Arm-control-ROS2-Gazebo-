from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os
import xacro
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('CRX-10iA_L_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'CRX-10iA_L.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    load_joint_trajectory_controller = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'joint_trajectory_controller'],
    output='screen'
    )
    

    load_joint_state_broadcaster = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'joint_state_broadcaster'],
    output='screen'
    ) 


    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'pause': 'true'
        }.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'CRX-10iA_L',
            '-topic', '/robot_description'
        ],
        output='screen'
    )
    
#    spawn_entity = Node(
#    package='gazebo_ros', 
#    executable='spawn_entity.py', 
#    arguments=['-topic', '/robot_description', '-entity', 'motion_controller'],
#    output='screen'

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        
        RegisterEventHandler(
          event_handler=OnProcessExit(
          target_action=urdf_spawn_node, 
          on_exit=[load_joint_trajectory_controller],
          )
        ),
        
        RegisterEventHandler(
          event_handler=OnProcessExit(
          target_action=load_joint_state_broadcaster, 
          on_exit=[load_joint_state_broadcaster],
          )
        ),
        
        gazebo_server,
        gazebo_client,
        urdf_spawn_node,
    ])
