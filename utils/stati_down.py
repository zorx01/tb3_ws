import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import math

class LidarDownsampler(Node):
    def __init__(self):
        super().__init__('lidar_downsampler')
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile)
        self.publisher_ = self.create_publisher(LaserScan, '/scan_downsampled', 10)

    def scan_callback(self, msg):
        # Filter out NaN values from ranges and corresponding intensities
        filtered_ranges_intensities = [(r, i) for r, i in zip(msg.ranges, msg.intensities) if not math.isnan(r) and not math.isnan(i)]
        
        if len(filtered_ranges_intensities) == 0:
            return  # Avoid publishing if there's no valid data
        
        filtered_ranges, filtered_intensities = zip(*filtered_ranges_intensities)

        original_length = len(filtered_ranges)
        
        # Calculate the step to reduce the points to exactly 40
        if original_length > 40:
            step = original_length // 40
        else:
            step = 1  # If less than 40 points, don't downsample

        # Downsample the ranges and intensities to exactly 40 points
        downsampled_ranges = filtered_ranges[::step][:40]
        downsampled_intensities = filtered_intensities[::step][:40]

        # Create a new LaserScan message
        downsampled_scan = LaserScan()
        downsampled_scan.header = msg.header
        downsampled_scan.angle_min = msg.angle_min
        downsampled_scan.angle_max = msg.angle_max
        downsampled_scan.angle_increment = msg.angle_increment * step
        downsampled_scan.time_increment = msg.time_increment * step
        downsampled_scan.scan_time = msg.scan_time
        downsampled_scan.range_min = msg.range_min
        downsampled_scan.range_max = msg.range_max
        downsampled_scan.ranges = downsampled_ranges
        downsampled_scan.intensities = downsampled_intensities

        # Publish the downsampled scan
        self.publisher_.publish(downsampled_scan)

def main(args=None):
    rclpy.init(args=args)
    lidar_downsampler = LidarDownsampler()
    rclpy.spin(lidar_downsampler)
    lidar_downsampler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
