#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from multi_sensor_system.msg import SensorData
from std_msgs.msg import Header
import random

class PressureSensor(Node):
    def __init__(self):
        super().__init__('pressure_sensor')
        
        # Configure QoS - Reliable with Transient Local durability
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL  # Save for late joiners!
        )
        
        self.publisher_ = self.create_publisher(
            SensorData,
            'sensor/pressure',
            qos_profile
        )
        
        self.timer = self.create_timer(1.5, self.publish_pressure)  # Every 1.5 seconds
        self.pressure = 1013.25  # Standard atmospheric pressure
        self.sequence_number = 0
        
        self.get_logger().info('Pressure Sensor Node Started')
        self.get_logger().info('Publishing to: sensor/pressure')
        self.get_logger().info(f'QoS: Reliable, Keep Last 10, Transient Local')
    
    def publish_pressure(self):
        # Simulate pressure reading
        self.pressure += random.uniform(-1.0, 1.0)
        self.pressure = max(980.0, min(1050.0, self.pressure))
        
        msg = SensorData()
        
        # Fill header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'pressure_sensor_frame'
        
        # Fill sensor info
        msg.sensor_name = 'PressureSensor_001'
        msg.sensor_type = 'environmental'
        
        # Fill readings
        msg.temperature = 0.0
        msg.humidity = 0.0
        msg.pressure = self.pressure
        
        # Fill status
        msg.is_active = True
        msg.status_message = f'Reading #{self.sequence_number}'
        
        self.publisher_.publish(msg)
        self.get_logger().info(
            f'[{self.sequence_number}] Pressure: {msg.pressure:.2f} hPa'
        )
        
        self.sequence_number += 1

def main(args=None):
    rclpy.init(args=args)
    node = PressureSensor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()