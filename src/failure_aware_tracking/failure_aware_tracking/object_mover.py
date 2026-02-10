#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class ObjectMover(Node):
    def __init__(self):
        super().__init__('object_mover')
        
        # Publishers for each object's velocity
        self.red_pub = self.create_publisher(Twist, '/red_cube/cmd_vel', 10)
        self.green_pub = self.create_publisher(Twist, '/green_sphere/cmd_vel', 10)
        self.blue_pub = self.create_publisher(Twist, '/blue_cylinder/cmd_vel', 10)
        
        # Timer for movement updates (30 Hz)
        self.timer = self.create_timer(0.033, self.move_objects)
        self.time = 0.0
        
        self.get_logger().info('Object mover node started')
    
    def move_objects(self):
        self.time += 0.033
        
        # Red cube: circular motion
        red_cmd = Twist()
        red_cmd.linear.x = 0.3 * math.cos(self.time)
        red_cmd.linear.y = 0.3 * math.sin(self.time)
        self.red_pub.publish(red_cmd)
        
        # Green sphere: figure-8 pattern
        green_cmd = Twist()
        green_cmd.linear.x = 0.2 * math.sin(self.time * 0.5)
        green_cmd.linear.y = 0.2 * math.sin(self.time)
        self.green_pub.publish(green_cmd)
        
        # Blue cylinder: linear back and forth
        blue_cmd = Twist()
        blue_cmd.linear.x = 0.3 * math.sin(self.time * 0.3)
        blue_cmd.linear.y = 0.0
        self.blue_pub.publish(blue_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()