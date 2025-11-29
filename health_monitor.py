import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

class HealthMonitor(Node):
    def __init__(self):
        super().__init__('health_monitor')
        # Subscribe to sensors
        self.subscription = self.create_subscription(
            Float32MultiArray, '/rover_sensors', self.listener_callback, 10)
        # Publish health status
        self.publisher_ = self.create_publisher(String, '/rover_health', 10)

    def listener_callback(self, msg):
        battery_level = msg.data[0]
        status_msg = String()
        
        # Logic: Critical if battery is low
        if battery_level < 20.0:
            status_msg.data = "Critical: Low Battery"
        elif battery_level < 40.0:
             status_msg.data = "Warning: Battery Low"
        else:
            status_msg.data = "Healthy"
            
        self.publisher_.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HealthMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
