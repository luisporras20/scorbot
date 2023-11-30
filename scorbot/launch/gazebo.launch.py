import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
	# Constants for paths to different files and folders
    gazebo_models_path = 'models'
    package_name = 'scorbot'
    robot_name_in_model = 'scorbot'
    rviz_config_file_path = 'rviz/config1.rviz'
    urdf_file_path = 'models/scorbot_printer.urdf'
    world_file_path = ''
	 # Pose where we want to spawn the robot
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.05'
    spawn_yaw_val = '0.0'
    my_package_dir = get_package_share_directory('scorbot')

    urdf_file_name = 'scorbot_printer.urdf'
    urdf = os.path.join(my_package_dir,
                        'models',
                        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    robot_description = {'robot_description': robot_desc}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("scorbot"),
            "config",
            "scorbot_er_v_controllers.yaml",
        ]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        arguments=[urdf]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
                '-d', os.path.join(my_package_dir, 'config', 'view_robot.rviz')],
        name='rviz2',
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_trajectory_position_controller"],
    )

    joint_foward_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["forward_position_controller"],
    )

   # Launch the robot
    spawn_entity_cmd = Node(
	    package='gazebo_ros', 
	    executable='spawn_entity.py',
	    arguments=['-entity', robot_name_in_model, 
		        '-topic', 'robot_description',
		            '-x', spawn_x_val,
		            '-y', spawn_y_val,
		            '-z', spawn_z_val,
		            '-Y', spawn_yaw_val],
		            output='screen')

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nodes = [
        gazebo,
        spawn_entity_cmd,
        joint_state_broadcaster_spawner,
        robot_state_pub_node,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(nodes)
