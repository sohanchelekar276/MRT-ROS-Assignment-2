import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

class Dashboard(Node):
    def __init__(self):
        super().__init__('dashboard')
        self.sensor_sub = self.create_subscription(
            Float32MultiArray, '/rover_sensors', self.sensor_callback, 10)
        self.health_sub = self.create_subscription(
            String, '/rover_health', self.health_callback, 10)
        
        self.latest_sensors = [0.0, 0.0]

    def sensor_callback(self, msg):
        self.latest_sensors = msg.data

    def health_callback(self, msg):
        # Print everything when we get a health update
        print("----- ROVER STATUS -----")
        print(f"Battery: {self.latest_sensors[0]:.2f}%")
        print(f"Temp:    {self.latest_sensors[1]:.2f} C")
        print(f"Status:  {msg.data}")
        print("------------------------\n")

def main(args=None):
    rclpy.init(args=args)
    node = Dashboard()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
