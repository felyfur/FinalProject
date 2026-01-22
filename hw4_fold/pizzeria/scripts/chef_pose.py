#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray # <--- Cambiato tipo messaggio
import time

class ChefPoseInitializer(Node):
    def __init__(self):
        super().__init__('chef_pose_initializer')
        # Pubblica sul topic dei comandi diretti
        self.publisher_ = self.create_publisher(Float64MultiArray, '/chef_position_controller/commands', 10)
        
        self.get_logger().info('Attesa caricamento Gazebo e Controller (10 secondi)...')
        time.sleep(10) # Ridotto un po' l'attesa
        
        self.send_pose()

    def send_pose(self):
        msg = Float64MultiArray()
        # La posa piegata valida (che non viola i limiti del giunto 6)
        # a2=0.5, a4=-1.2, a6=1.5
        msg.data = [0.0, 0.5, 0.0, -1.2, 0.0, 1.4, 0.0]
        
        # Inviamo il comando più volte per essere sicuri che il controller lo prenda
        for _ in range(5):
            self.publisher_.publish(msg)
            time.sleep(0.1)
        
        self.get_logger().info('Comando inviato allo Chef! Posa di scansione impostata.')

def main(args=None):
    rclpy.init(args=args)
    node = ChefPoseInitializer()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()