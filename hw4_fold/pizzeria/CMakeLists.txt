cmake_minimum_required(VERSION 3.8)
project(pizzeria)

# Setup standard compilatore
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. TROVARE I PACCHETTI (Dependencies)
# Dobbiamo includere tutte le librerie usate nel codice C++
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)   # Serve per i messaggi geometrici
find_package(trajectory_msgs REQUIRED) # Serve per JointTrajectory
find_package(kdl_parser REQUIRED)      # Serve per leggere l'URDF
find_package(orocos_kdl REQUIRED)      # La libreria KDL matematica
find_package(ros_gz_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

# 2. FILE DI INTESTAZIONE (Headers)
# Diciamo a CMake dove trovare i file .h (cartella include)
include_directories(include)

add_executable(deliver_pizza_node src/deliver_pizza_node.cpp)
ament_target_dependencies(deliver_pizza_node
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  std_msgs
  ros_gz_interfaces
)

# NODO TF PUBLISHER (Aggiungi questo blocco)
add_executable(odom_bl_tf src/odom_bl_tf.cpp)
ament_target_dependencies(odom_bl_tf
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)

# 3. CREAZIONE ESEGUIBILE C++
# Creiamo l'eseguibile 'chef_kdl_node' compilando il main + le classi di supporto
add_executable(chef_kdl_node
  src/chef_kdl_node.cpp
  src/kdl_robot.cpp
  src/kdl_control.cpp
  src/kdl_planner.cpp
)

# Linkiamo le librerie ROS2 e KDL all'eseguibile
ament_target_dependencies(chef_kdl_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  kdl_parser
  orocos_kdl
  ros_gz_interfaces
)

# 4. INSTALLAZIONE
# Installa l'eseguibile C++ nella cartella lib del pacchetto
install(TARGETS
  chef_kdl_node
  odom_bl_tf
  deliver_pizza_node
  DESTINATION lib/${PROJECT_NAME}
)

# Installa le cartelle di configurazione, launch, modelli, ecc.
install(DIRECTORY
  launch
  models
  worlds
  urdf
  textures
  config
  maps
  DESTINATION share/${PROJECT_NAME}
)

# Installa lo script Python
install(PROGRAMS
   scripts/chef_pose.py
   DESTINATION lib/${PROJECT_NAME}
)

ament_package()
