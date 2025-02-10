import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan

class LidarFilter(Node):
    def __init__(self):
        super().__init__('lidar_filter')

        # Define a QoS profile with Best Effort reliability
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Subscribing to /scan with Best Effort QoS
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )

        # Publishing back to /scan with Best Effort QoS
        self.publisher = self.create_publisher(
            LaserScan,
            '/scan',
            qos_profile
        )

    def scan_callback(self, msg):
        # Filter close-range values
        filtered_ranges = [
            r if r > 0.3 else float('inf')  # Replace values below 15 cm with "inf"
            for r in msg.ranges
        ]
        msg.ranges = filtered_ranges
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarFilter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
