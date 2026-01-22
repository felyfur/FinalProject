import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_pizzeria = get_package_share_directory('pizzeria')
    chef_xacro = os.path.join(pkg_pizzeria, 'urdf', 'chef', 'chef.urdf.xacro')
    
    # Processa Xacro
    chef_desc = ParameterValue(Command(['xacro ', chef_xacro]), value_type=str)

    return LaunchDescription([
        Node(
            package='pizzeria',
            executable='chef_kdl_node',
            name='chef_kdl_logic',
            output='screen',
            parameters=[{
                'robot_description': chef_desc,
                'cmd_interface': 'position' # Usa position per stabilità
            }]
        )
    ])