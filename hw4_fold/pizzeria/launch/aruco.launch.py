from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # 1. Definiamo i parametri
    marker_id_arg = DeclareLaunchArgument(
        'marker_id', default_value='0', 
        description='Marker ID to track'
    )

    marker_size_arg = DeclareLaunchArgument(
        'marker_size', default_value='0.20', 
        description='Marker size in meters'
    )

    reference_frame_arg = DeclareLaunchArgument(
        'reference_frame', default_value='world',
        description='Reference frame for the pose'
    )

    # 2. Configurazione del nodo Aruco
    aruco_node = Node(
        package='aruco_ros',       
        executable='single',       
        name='aruco_single',       
        parameters=[{
            'marker_id': LaunchConfiguration('marker_id'),
            'marker_size': LaunchConfiguration('marker_size'),
            'reference_frame': LaunchConfiguration('reference_frame'),
            'marker_frame': 'aruco_marker_frame',
            'image_is_rectified': True,
            'camera_frame': 'camera_optical_frame',  # <--- RIMESSO 'camera_link' (Non lasciare vuoto!)
            'marker_dictionary': 'ARUCO_ORIGINAL', 
            'use_sim_time': True        
        }],
        remappings=[
            ('/camera_info', '/chef/camera/camera_info'),
            ('/image', '/chef/camera/image_raw')
        ]
    )

    return LaunchDescription([
        marker_id_arg,
        marker_size_arg,
        reference_frame_arg,
        aruco_node
    ])