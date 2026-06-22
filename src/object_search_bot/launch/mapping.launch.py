import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Paths and Configurations
    pkg_object_search = get_package_share_directory('object_search_bot')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    
    world_file = os.path.join(pkg_object_search, 'worlds', 'office_teardown.world')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 2. Environment Variables
    # Explicitly set the model to waffle
    set_model_cmd = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle')

    # 3. Nodes and Includes
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    rsp_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # FIX: Added a longer timeout (300s) to prevent the "Spawn service failed" error
    spawn_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'waffle',
            '-file', os.path.join(pkg_tb3_gazebo, 'models', 'turtlebot3_waffle', 'model.sdf'),
            '-x', '0.0', '-y', '0.0', '-z', '0.05',
            '-timeout', '300' 
        ],
        output='screen'
    )

    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    ld = LaunchDescription()
    ld.add_action(set_model_cmd)
    ld.add_action(gazebo_cmd)
    ld.add_action(rsp_cmd)
    ld.add_action(spawn_cmd)
    ld.add_action(slam_cmd)

    return ld