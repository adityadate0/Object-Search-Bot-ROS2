import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def launch_setup(context, *args, **kwargs):
    base_station_choice = LaunchConfiguration('base_station').perform(context)
    pkg_object_search_bot = get_package_share_directory('object_search_bot')
    yaml_params_file = os.path.join(pkg_object_search_bot, 'maps', 'navigation_targets.yaml')
    
    with open(yaml_params_file, 'r') as f:
        yaml_data = yaml.safe_load(f)
    params = yaml_data['/**']['ros__parameters']
    
    spawn_x = str(params[base_station_choice]['x'])
    spawn_y = str(params[base_station_choice]['y'])
    spawn_yaw = str(params[base_station_choice]['yaw'])
    
    xacro_file = os.path.join(pkg_object_search_bot, 'urdf', 'waffle_depth.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    rsp_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True, 
            'robot_description': robot_description_raw
        }]
    )

    spawn_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'waffle', '-topic', 'robot_description', '-x', spawn_x, '-y', spawn_y, '-Y', spawn_yaw, '-z', '0.05'],
        output='screen'
    )
    
    return [rsp_cmd, spawn_cmd]

def generate_launch_description():
    set_model_cmd = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle')
    pkg_object_search_bot = get_package_share_directory('object_search_bot')
    world_file = os.path.join(pkg_object_search_bot, 'worlds', 'office_teardown.world')
    yaml_params_file = os.path.join(pkg_object_search_bot, 'maps', 'navigation_targets.yaml')
    
    base_station_arg = DeclareLaunchArgument(
        'base_station',
        default_value='top_left_alcove',
        description='Pre-selected home base option'
    )
    
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file, 'gui': 'true'}.items() 
    )

    perception_node = Node(
        package='object_search_bot',
        executable='perception_node',
        name='perception_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return_manager_node = Node(
        package='object_search_bot',
        executable='base_return_manager',
        name='base_return_manager',
        output='screen',
        parameters=[
            yaml_params_file, 
            {'selected_base': LaunchConfiguration('base_station')}
        ]
    )

    map_yaml_file = os.path.join(pkg_object_search_bot, 'maps', 'office_teardown.yaml')
    nav2_params_file = os.path.join(get_package_share_directory('nav2_bringup'), 'params', 'nav2_params.yaml')

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav2_params_file, {'yaml_filename': map_yaml_file, 'use_sim_time': True}]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': True}]
    )

    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': True}]
    )

    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': True}]
    )

    # 🛠️ ADDED: Core behavior recoveries server node to handle spin/wait tasks
    behaviors_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': True}]
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': True}]
    )

    # 🚀 UPDATED LIFECYCLE NODE NAMES: Added 'behavior_server' into the sequence chain
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server', 'amcl', 'planner_server', 'controller_server', 'behavior_server', 'bt_navigator']
        }]
    )

    ld = LaunchDescription()
    ld.add_action(set_model_cmd)
    ld.add_action(base_station_arg)
    ld.add_action(gazebo_cmd)
    
    # Core Navigation Array
    ld.add_action(map_server_node)
    ld.add_action(amcl_node)
    ld.add_action(planner_node)
    ld.add_action(controller_node)
    ld.add_action(behaviors_node) # 👈 Fired into graph
    ld.add_action(bt_navigator_node)
    ld.add_action(lifecycle_manager_node)
    
    # Mission Nodes
    ld.add_action(perception_node)
    ld.add_action(return_manager_node)
    ld.add_action(OpaqueFunction(function=launch_setup))
    
    return ld