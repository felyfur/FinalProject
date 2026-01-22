import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, SetRemap

def generate_launch_description():
    # Definiamo i path ai pacchetti
    fra2mo_dir = FindPackageShare('ros2_fra2mo')
    pizzeria_dir = FindPackageShare('pizzeria') 
    
    nav2_bringup_launch_file_dir = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )
    
    rviz_config_file = os.path.join(get_package_share_directory('pizzeria'), 'config', 'RVIZ_FINAL_CONFIG.rviz')

    # Configurazione Argomenti
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([pizzeria_dir, 'maps', 'pizzeria_map.yaml']), # Controlla se la cartella è 'map' o 'maps'
        description='Full path to map yaml file to load',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([pizzeria_dir, 'config', 'navigation.yaml']),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # --- NODO MANCANTE NEL TUO CODICE ---
    # Questo è il nodo C++ (odom_bl_tf) che crea il frame fra2mo/odom
    odom_tf_publisher_node = Node(
        package='pizzeria',
        executable='odom_bl_tf', 
        name='dynamic_tf_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    # ------------------------------------

    # --- NAV2 BRINGUP CON REMAPPING ---
    nav2_bringup_launch = GroupAction(
        actions=[
            SetRemap(src='/scan', dst='/lidar'),
            SetRemap(src='/odom', dst='/model/fra2mo/odometry'),
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav2_bringup_launch_file_dir]),
                launch_arguments={
                    'map': map_yaml_file,
                    'params_file': params_file,
                    'use_sim_time': use_sim_time,
                }.items(),
            )
        ]
    )
    
    # Nodo RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription(
        [
            declare_map_yaml_cmd,
            declare_params_file_cmd,
            declare_use_sim_time_cmd,
            odom_tf_publisher_node, # <--- IMPORTANTE: Deve essere incluso qui!
            nav2_bringup_launch,
            rviz_node,
        ]
    )