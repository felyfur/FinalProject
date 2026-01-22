import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # 1. SETUP PERCORSI
    pkg_pizzeria = get_package_share_directory('pizzeria')
    pkg_fra2mo = get_package_share_directory('ros2_fra2mo')
    
    ### MODIFICA 1: Aggiunto pacchetto Kuka (iiwa_description) ###
    pkg_iiwa = get_package_share_directory('iiwa_description') 
    
    world_file = os.path.join(pkg_pizzeria, 'worlds', 'pizzeria.world')
    models_path = os.path.join(pkg_pizzeria, 'models')
    
    ### MODIFICA 2: Puntiamo ai file URDF dentro 'pizzeria', non quelli originali ###
    # Cameriere (Fra2mo + Vassoio)
    waiter_xacro = os.path.join(pkg_pizzeria, 'urdf', 'waiter', 'waiter.urdf.xacro')
    # Chef (Kuka + Camera + Pinza)
    chef_xacro = os.path.join(pkg_pizzeria, 'urdf', 'chef', 'chef.urdf.xacro')

    # 2. ELABORAZIONE XACRO (Creiamo due descrizioni separate)
    ### MODIFICA 3: Due comandi xacro distinti ###
    waiter_desc = ParameterValue(Command(['xacro ', waiter_xacro]), value_type=str)
    chef_desc = ParameterValue(Command(['xacro ', chef_xacro]), value_type=str)

    # 3. SETUP AMBIENTE
    ### MODIFICA 4: Aggiunto path di iiwa per le mesh ###
    resource_env = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', 
        value=models_path + ':' + 
              os.path.join(pkg_fra2mo, '..') + ':' + 
              os.path.join(pkg_iiwa, '..') + ':' + 
              os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )

    # 4. AVVIO GAZEBO
    gz_args = DeclareLaunchArgument('gz_args', default_value=[world_file, ' -r'])
    
    gazebo_ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                'launch',
                                'gz_sim.launch.py'])]),
        launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items()
    )

    # 5. ROBOT STATE PUBLISHERS (RSP)
    ### MODIFICA 5: Due nodi RSP separati con Namespace ###
    
    # Nodo per il Cameriere (waiter)
    rsp_fra2mo = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='rsp_fra2mo',
        parameters=[{'robot_description': waiter_desc, 'use_sim_time': True}],
        # Remapping fondamentale: scrive su un topic dedicato
        remappings=[('/robot_description', '/fra2mo/robot_description')] 
    )

    # Nodo per lo Chef (kuka)
    rsp_chef = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='rsp_chef',
        parameters=[{'robot_description': chef_desc, 'use_sim_time': False}],
        # Remapping fondamentale: scrive su un topic dedicato
        remappings=[('/robot_description', '/chef/robot_description')]
    )

    # 6. SPAWN ROBOT
    ### MODIFICA 6: Due nodi Spawn separati ###
    
    spawn_fra2mo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'fra2mo',
            '-topic', '/fra2mo/robot_description', # Legge dal topic del cameriere
            '-x', '0.65', '-y', '0.0', '-z', '0.1'
        ],
        output='screen'
    )

    spawn_chef = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'chef_robot', 
            '-topic', '/chef/robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.0'
        ],
        output='screen'
    )

# 7. BRIDGE ROS2 <-> GAZEBO
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Orologio
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            
            # Fra2mo
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/model/fra2mo/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/model/fra2mo/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            
            # Chef Camera
            '/chef/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/chef/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',

            # --- IL BRIDGE FONDAMENTALE PER IL GRASPING (Servizio) ---
            # Questo permette a ROS di chiamare il servizio di posizionamento di Gazebo
            '/world/pizzeria_world/set_pose@ros_gz_interfaces/srv/SetEntityPose',
            '/world/pizzeria_world/remove@ros_gz_interfaces/srv/DeleteEntity'
        ],
        remappings=[
            ('/model/fra2mo/tf', '/tf'),
            # --- QUESTA RIGA SERVE A RINOMINARLO ---
            ('/world/pizzeria_world/remove', '/delete_entity')
        ],
        output='screen'
    )
    
    # 8. CONTROLLER SPAWNERS (Nuova Sezione)
    # Questo nodo attiva il publisher degli stati dei giunti (fondamentale per TF)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    # Questo nodo attiva il controller di traiettoria del braccio
    chef_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['chef_position_controller', '--controller-manager', '/controller_manager'],
    )

    # 9. INIZIALIZZAZIONE POSA CHEF (Script Automatico)
    init_chef_pose = Node(
        package='pizzeria',
        executable='chef_pose.py',
        output='screen'
    )

    return LaunchDescription([
        resource_env,
        gz_args,
        gazebo_ignition,
        rsp_fra2mo,
        spawn_fra2mo,
        rsp_chef,
        spawn_chef,
        bridge,
        joint_state_broadcaster_spawner,
        chef_controller_spawner,
        init_chef_pose
    ])