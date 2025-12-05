#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from multi_sensor_system.msg import SensorData
from std_msgs.msg import Header
import random

class HumiditySensor(Node):
    def __init__(self):
        super().__init__('humidity_sensor')
        
        # Configure QoS - Best Effort for non-critical data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # May lose messages
            history=HistoryPolicy.KEEP_LAST,
            depth=5,                                     # Smaller queue
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.publisher_ = self.create_publisher(
            SensorData,
            'sensor/humidity',
            qos_profile
        )
        
        self.timer = self.create_timer(2.0, self.publish_humidity)  # Every 2 seconds
        self.humidity = 60.0
        self.sequence_number = 0
        
        self.get_logger().info('Humidity Sensor Node Started')
        self.get_logger().info('Publishing to: sensor/humidity')
        self.get_logger().info(f'QoS: Best Effort, Keep Last 5, Volatile')
    
    def publish_humidity(self):
        # Simulate humidity reading
        self.humidity += random.uniform(-2.0, 2.0)
        self.humidity = max(30.0, min(90.0, self.humidity))
        
        msg = SensorData()
        
        # Fill header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'humidity_sensor_frame'
        
        # Fill sensor info
        msg.sensor_name = 'HumiditySensor_001'
        msg.sensor_type = 'environmental'
        
        # Fill readings
        msg.temperature = 0.0
        msg.humidity = self.humidity
        msg.pressure = 0.0
        
        # Fill status
        msg.is_active = True
        msg.status_message = f'Reading #{self.sequence_number}'
        
        self.publisher_.publish(msg)
        self.get_logger().info(
            f'[{self.sequence_number}] Humidity: {msg.humidity:.1f}%'
        )
        
        self.sequence_number += 1

def main(args=None):
    rclpy.init(args=args)
    node = HumiditySensor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()