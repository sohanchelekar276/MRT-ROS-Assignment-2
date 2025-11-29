import rclpy
from rclpy.node import Node
from mars_msgs.msg import RoverOdometry
import math

class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_subscriber')
        self.subscription = self.create_subscription(
            RoverOdometry, '/odom_data', self.listener_callback, 10)
        
        # Current Position
        self.x = 0.0
        self.y = 0.0

    def listener_callback(self, msg):
        v = msg.linear_velocity.linear.x
        theta = msg.orientation
        dt = 1.0 # Since publisher sends every 1 second
        
        # Update coordinates
        self.x += v * math.cos(theta) * dt
        self.y += v * math.sin(theta) * dt
        
        # Warning check
        if v > 3.0:
            self.get_logger().warn(f"SPEED WARNING: {v:.2f} m/s exceeds limit!")
            
        print(f"Position: x={self.x:.2f}, y={self.y:.2f} | Orientation: {theta:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = OdomSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
