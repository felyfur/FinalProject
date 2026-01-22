import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- PERCORSI ---
    pkg_pizzeria = get_package_share_directory('pizzeria')
    pkg_fra2mo = get_package_share_directory('ros2_fra2mo')

    # File del Prof da includere (SENZA MODIFICARLO)
    fra2mo_explore_launch_path = os.path.join(
        pkg_fra2mo, 'launch', 'fra2mo_explore.launch.py'
    )

    # Il tuo file di configurazione (fondamentale per Nav2 e Explore Lite)
    my_explore_config = os.path.join(pkg_pizzeria, 'config', 'explore.yaml')

    # --- ARGOMENTI ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 1. NODO TF (Fondamentale per il tuo setup Docker/Bridge)
    # Questo nodo serve a collegare l'odom di Gazebo al base_footprint
    # Assumiamo che tu abbia l'eseguibile 'odom_bl_tf' compilato nel pacchetto pizzeria
    odom_tf_node = Node(
        package='pizzeria',
        executable='odom_bl_tf',
        name='odom_bl_tf',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 2. ESPLORAZIONE DEL PROF
    # Includiamo il file del prof ma gli passiamo il NOSTRO file di parametri.
    # Nota: Il file fra2mo_explore lancia internamente anche SLAM Toolbox.
    # SLAM Toolbox userà la configurazione di default di fra2mo (slam.yaml) per i parametri SLAM,
    # ma Nav2 e Explore Lite useranno il file che passiamo qui ('params_file').
    explore_logic = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fra2mo_explore_launch_path),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': my_explore_config 
        }.items()
    )

    return LaunchDescription([
        odom_tf_node,
        explore_logic
    ])