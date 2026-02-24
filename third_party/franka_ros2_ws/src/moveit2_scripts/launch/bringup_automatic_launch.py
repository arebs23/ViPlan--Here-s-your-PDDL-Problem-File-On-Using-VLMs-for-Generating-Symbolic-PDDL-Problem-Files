import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            Shutdown, TimerAction, GroupAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None
pick_config_path = os.path.join(
        get_package_share_directory('moveit2_scripts'),
        'config',
        'pick_config.yaml'
    )

def generate_launch_description():
    # -------------------------------------------------------------
    # 1. SETUP ARGUMENTS & CONFIGURATIONS (From File 2)
    # -------------------------------------------------------------
    robot_ip_parameter_name = 'robot_ip'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    load_gripper_parameter_name = 'load_gripper'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)

    db_arg = DeclareLaunchArgument('db', default_value='False', description='Database flag')

    # -------------------------------------------------------------
    # 2. LOAD ROBOT DESCRIPTION & MOVEIT CONFIG (From File 2)
    # -------------------------------------------------------------
    # Robot Description (URDF)
    franka_xacro_file = os.path.join(get_package_share_directory('franka_description'), 'robots', 'panda_arm.urdf.xacro')
    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, ' hand:=', load_gripper,
         ' robot_ip:=', robot_ip, ' use_fake_hardware:=', use_fake_hardware,
         ' fake_sensor_commands:=', fake_sensor_commands])
    robot_description = {'robot_description': robot_description_config}

    # Semantic Description (SRDF)
    franka_semantic_xacro_file = os.path.join(get_package_share_directory('franka_moveit_config'), 'srdf', 'panda_arm.srdf.xacro')
    robot_description_semantic_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_semantic_xacro_file, ' hand:=', load_gripper]
    )
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    # Kinematics
    kinematics_yaml = load_yaml('franka_moveit_config', 'config/kinematics.yaml')

    # OMPL Planning
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml = load_yaml('franka_moveit_config', 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # Controllers
    moveit_simple_controllers_yaml = load_yaml('franka_moveit_config', 'config/panda_controllers.yaml')
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'publish_robot_description_semantic': True,
    }

    # -------------------------------------------------------------
    # 3. DEFINE NODES (Infrastructure)
    # -------------------------------------------------------------
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    rviz_base = os.path.join(get_package_share_directory('franka_moveit_config'), 'rviz')
    rviz_full_config = os.path.join(rviz_base, 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    ros2_controllers_path = os.path.join(get_package_share_directory('franka_moveit_config'), 'config', 'panda_ros_controllers.yaml')
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ros2_controllers_path],
        remappings=[('joint_states', 'franka/joint_states')],
        output='screen',
        on_exit=Shutdown(),
    )

    load_controllers = []
    for controller in ['panda_arm_controller', 'joint_state_broadcaster']:
        load_controllers += [
            ExecuteProcess(
                cmd=['ros2 run controller_manager spawner {}'.format(controller)],
                shell=True,
                output='screen',
            )
        ]

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'source_list': ['franka/joint_states', 'panda_gripper/joint_states'], 'rate': 30}],
    )

    # -------------------------------------------------------------
    # 4. YOUR CUSTOM NODES (From File 1)
    # -------------------------------------------------------------
    
    # Common remappings
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # We use TimerAction to delay them slightly to ensure MoveGroup is ready
    custom_action_servers = GroupAction([
        Node(
            name='move_action_node',
            package='moveit2_scripts',
            executable='pick_place',
            remappings=remappings,
            parameters=[pick_config_path],
            output='screen'),

        Node(
            name='open_gripper_action_node',
            package='moveit2_scripts',
            executable='open_gripper_action_server',
            remappings=remappings,
            output='screen'),

         Node(
            name='close_gripper_action_node',
            package='moveit2_scripts',
            executable='close_gripper_action_server',
            remappings=remappings,
            output='screen'),
         
         # Optional: Add your test collision node if needed
         Node(
            name='add_table_node',
            package='moveit2_scripts',
            executable='test_collision',
            remappings=remappings,
            output='screen'),
    ])

    # -------------------------------------------------------------
    # 5. ARGUMENTS & FINAL LAUNCH
    # -------------------------------------------------------------
    robot_arg = DeclareLaunchArgument(robot_ip_parameter_name, description='Hostname or IP address of the robot.')
    use_fake_hardware_arg = DeclareLaunchArgument(use_fake_hardware_parameter_name, default_value='false', description='Use fake hardware')
    load_gripper_arg = DeclareLaunchArgument(load_gripper_parameter_name, default_value='true', description='Use Franka Gripper')
    fake_sensor_commands_arg = DeclareLaunchArgument(fake_sensor_commands_parameter_name, default_value='false', description="Fake sensor commands")
    
    gripper_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
        launch_arguments={'robot_ip': robot_ip, use_fake_hardware_parameter_name: use_fake_hardware}.items(),
        condition=IfCondition(load_gripper)
    )

    return LaunchDescription(
        [robot_arg,
         use_fake_hardware_arg,
         fake_sensor_commands_arg,
         load_gripper_arg,
         db_arg,
         rviz_node,
         robot_state_publisher,
         run_move_group_node,
         ros2_control_node,
         joint_state_publisher,
         gripper_launch_file,
         # Launch your custom servers after a 5 second delay to let MoveIt load
         TimerAction(period=5.0, actions=[custom_action_servers])
         ]
        + load_controllers
    )