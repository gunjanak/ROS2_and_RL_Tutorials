#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from multi_sensor_system.msg import SensorData
from std_msgs.msg import Header
import random
import time

class TemperatureSensor(Node):
    def __init__(self):
        super().__init__('temperature_sensor')
        
        # Configure QoS profile for sensor data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # Guaranteed delivery
            history=HistoryPolicy.KEEP_LAST,          # Keep last N messages
            depth=10,                                  # Queue size
            durability=DurabilityPolicy.VOLATILE      # Don't save for late joiners
        )
        
        # Create publisher with custom QoS
        self.publisher_ = self.create_publisher(
            SensorData,
            'sensor/temperature',
            qos_profile
        )
        
        # Timer for periodic publishing
        self.timer = self.create_timer(1.0, self.publish_temperature)
        
        # Sensor state
        self.temperature = 22.0
        self.sequence_number = 0
        
        self.get_logger().info('Temperature Sensor Node Started')
        self.get_logger().info('Publishing to: sensor/temperature')
        self.get_logger().info(f'QoS: Reliable, Keep Last 10, Volatile')
    
    def publish_temperature(self):
        # Simulate temperature reading
        self.temperature += random.uniform(-0.5, 0.5)
        self.temperature = max(18.0, min(28.0, self.temperature))
        
        # Create message
        msg = SensorData()
        
        # Fill header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'temperature_sensor_frame'
        
        # Fill sensor info
        msg.sensor_name = 'TempSensor_001'
        msg.sensor_type = 'environmental'
        
        # Fill readings
        msg.temperature = self.temperature
        msg.humidity = 0.0      # Not measured by this sensor
        msg.pressure = 0.0      # Not measured by this sensor
        
        # Fill status
        msg.is_active = True
        msg.status_message = f'Reading #{self.sequence_number}'
        
        # Publish
        self.publisher_.publish(msg)
        self.get_logger().info(
            f'[{self.sequence_number}] Temp: {msg.temperature:.2f}°C'
        )
        
        self.sequence_number += 1

def main(args=None):
    rclpy.init(args=args)
    node = TemperatureSensor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()