#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class TemperatureMonitor(Node):
    def __init__(self):
        super().__init__('temperature_monitor')
        
        #Create subscriber to 'temperature' topic
        self .subscription = self.create_subscription(
            Float32,
            'temperature',
            self.temperature_callback,
            10
        )
        
        self.get_logger().info('Temperature Monitor Node Started')
        
        
    def temperature_callback(self,msg):
        temp = msg.data
        
        #Simple alert system
        if temp > 26.0:
            status = "High"
        elif temp < 20.0:
            status = "Low"
        else:
            status= "Normal"
            
            
        self.get_logger().info(f'Temperature: {temp:.2f}C - Status:{status}')
        
def main(args=None):
    rclpy.init(args=args)
    node = TemperatureMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
if __name__ == "__main__":
    main()