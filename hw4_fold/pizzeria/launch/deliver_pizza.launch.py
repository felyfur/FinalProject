from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pizzeria',
            executable='deliver_pizza_node', # Nome definito in CMakeLists
            name='waiter_brain_cpp',
            output='screen',
            emulate_tty=True
        )
    ])