import rclpy
from rclpy.node import Node
import random
from rover_odometry.msg import RoverOdometry
from geometry_msgs.msg import Twist

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_pub')
        self.pub = self.create_publisher(RoverOdometry, 'rover_odom', 10)
        self.orientation = 0.0
        self.timer = self.create_timer(1.0, self.publish_odom)

    def publish_odom(self):
        msg = RoverOdometry()
        msg.rover_id = 1
        msg.angular_velocity = random.uniform(-1.0, 1.0)
        msg.linear_velocity = Twist()
        msg.linear_velocity.linear.x = random.uniform(0.0, 5.0)

        self.orientation += msg.angular_velocity * 1.0   # Δθ = ω * dt
        msg.orientation = self.orientation

        self.pub.publish(msg)
        self.get_logger().info(f"Published odometry: v={msg.linear_velocity.linear.x:.2f}, ω={msg.angular_velocity:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
