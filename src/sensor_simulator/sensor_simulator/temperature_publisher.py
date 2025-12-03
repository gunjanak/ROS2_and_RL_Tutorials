#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random


class TemperaturSensor(Node):
    def __init__(self):
        super().__init__('temperature_sensor')
        
        #Create publisher on "temperature" topic
        self.publisher_ = self.create_publisher(Float32,'temperature',10)
        
        #Set up time to publish every 1 second
        self.timer = self.create_timer(1.0,self.publish_temperature)
        
        #initial temperature
        self.temperature = 22.0
        
        self .get_logger().info("Temperature Sensor Node Started")
        
    def publish_temperature(self):
        #simulate temperature fluctuation
        self.temperature += random.uniform(-0.5,0.5)
        
        #keep temperature inrealistic range 
        self.temperature =max(10.2,min(28.0,self.temperature))
        
        #create and publish message
        msg = Float32()
        msg.data = self.temperature
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {self.temperature:.2f}C')
    
def main(args=None):
    rclpy.init(args=args)
    node = TemperaturSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    