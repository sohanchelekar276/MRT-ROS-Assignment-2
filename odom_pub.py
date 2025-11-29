import rclpy
from rclpy.node import Node
from mars_msgs.msg import RoverOdometry
from geometry_msgs.msg import Twist
import random

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        self.publisher_ = self.create_publisher(RoverOdometry, '/odom_data', 10)
        self.timer = self.create_timer(1.0, self.publish_odom) # Every 1 second
        self.orientation = 0.0

    def publish_odom(self):
        msg = RoverOdometry()
        msg.rover_id = 1
        
        # Simulate data
        self.orientation += 0.1 # Slowly turning
        msg.orientation = self.orientation
        
        msg.linear_velocity = Twist()
        msg.linear_velocity.linear.x = random.uniform(0, 5) # Random speed 0-5 m/s
        msg.angular_velocity = random.uniform(-1, 1)
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Sent Odom for ID {msg.rover_id}')

def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
