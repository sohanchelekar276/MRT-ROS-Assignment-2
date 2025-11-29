import rclpy
from rclpy.node import Node
from rover_odometry.msg import RoverOdometry
import math

class OdometrySubscriber(Node):
    def __init__(self):
        super().__init__('odometry_sub')

        self.create_subscription(RoverOdometry, 'rover_odom',
                                 self.odom_cb, 10)

        self.x = 0.0
        self.y = 0.0

    def odom_cb(self, msg):
        v = msg.linear_velocity.linear.x
        w = msg.angular_velocity
        theta = msg.orientation

        dt = 1.0
        self.x += v * math.cos(theta) * dt
        self.y += v * math.sin(theta) * dt

        print(f"\n--- Rover Odometry ---")
        print(f"ID: {msg.rover_id}")
        print(f"Orientation: {theta:.2f} rad")
        print(f"Position: ({self.x:.2f}, {self.y:.2f})")
        print(f"Linear velocity: {v:.2f} m/s")
        print(f"Angular velocity: {w:.2f} rad/s")

        if v > 3.0:
            print("WARNING: Linear velocity exceeds safe limit!")

def main(args=None):
    rclpy.init(args=args)
    node = OdometrySubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
