import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import random

class SensorSim(Node):
    def __init__(self):
        super().__init__('sensor_sim')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/rover_sensors', 10)
        self.timer = self.create_timer(1.0, self.publish_data) # Runs every 1 second

    def publish_data(self):
        msg = Float32MultiArray()
        battery = random.uniform(0, 100)
        temp = random.uniform(-20, 80)
        
        msg.data = [battery, temp]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: Battery={battery:.2f}%, Temp={temp:.2f}C')

def main(args=None):
    rclpy.init(args=args)
    node = SensorSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
