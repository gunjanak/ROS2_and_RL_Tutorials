#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from multi_sensor_system.msg import SensorData
from datetime import datetime

class DashboardMonitor(Node):
    def __init__(self):
        super().__init__('dashboard_monitor')
        
        # Store latest readings
        self.latest_temperature = None
        self.latest_humidity = None
        self.latest_pressure = None
        
        # QoS for temperature (match publisher)
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # QoS for humidity (match publisher)
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # QoS for pressure (match publisher)
        qos_transient = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Create subscribers
        self.temp_sub = self.create_subscription(
            SensorData,
            'sensor/temperature',
            self.temperature_callback,
            qos_reliable
        )
        
        self.humidity_sub = self.create_subscription(
            SensorData,
            'sensor/humidity',
            self.humidity_callback,
            qos_best_effort
        )
        
        self.pressure_sub = self.create_subscription(
            SensorData,
            'sensor/pressure',
            self.pressure_callback,
            qos_transient
        )
        
        # Timer to display dashboard
        self.timer = self.create_timer(3.0, self.display_dashboard)
        
        self.get_logger().info('Dashboard Monitor Started')
        self.get_logger().info('Subscribed to: sensor/temperature, sensor/humidity, sensor/pressure')
        print("\n" + "="*60)
        print(" MULTI-SENSOR DASHBOARD")
        print("="*60 + "\n")
    
    def temperature_callback(self, msg):
        self.latest_temperature = msg
    
    def humidity_callback(self, msg):
        self.latest_humidity = msg
    
    def pressure_callback(self, msg):
        self.latest_pressure = msg
    
    def display_dashboard(self):
        print("\n" + "-"*60)
        print(f" Dashboard Update: {datetime.now().strftime('%H:%M:%S')}")
        print("-"*60)
        
        # Display Temperature
        if self.latest_temperature:
            temp = self.latest_temperature.temperature
            status = self.get_temperature_status(temp)
            print(f" 🌡️  Temperature: {temp:.2f}°C  {status}")
        else:
            print(" 🌡️  Temperature: Waiting for data...")
        
        # Display Humidity
        if self.latest_humidity:
            humidity = self.latest_humidity.humidity
            status = self.get_humidity_status(humidity)
            print(f" 💧  Humidity:    {humidity:.1f}%     {status}")
        else:
            print(" 💧  Humidity:    Waiting for data...")
        
        # Display Pressure
        if self.latest_pressure:
            pressure = self.latest_pressure.pressure
            status = self.get_pressure_status(pressure)
            print(f" 🌪️  Pressure:    {pressure:.2f} hPa  {status}")
        else:
            print(" 🌪️  Pressure:    Waiting for data...")
        
        print("-"*60)
        
        # Overall assessment
        if all([self.latest_temperature, self.latest_humidity, self.latest_pressure]):
            overall = self.get_overall_status()
            print(f"\n Overall Status: {overall}\n")
    
    def get_temperature_status(self, temp):
        if temp < 18:
            return "❄️  COLD"
        elif temp > 26:
            return "🔥 HOT"
        else:
            return "✅ COMFORTABLE"
    
    def get_humidity_status(self, humidity):
        if humidity < 30:
            return "⚠️  TOO DRY"
        elif humidity > 70:
            return "⚠️  TOO HUMID"
        else:
            return "✅ COMFORTABLE"
    
    def get_pressure_status(self, pressure):
        if pressure < 1000:
            return "⚠️  LOW (Storm?)"
        elif pressure > 1025:
            return "☀️  HIGH (Clear)"
        else:
            return "✅ NORMAL"
    
    def get_overall_status(self):
        temp = self.latest_temperature.temperature
        humidity = self.latest_humidity.humidity
        pressure = self.latest_pressure.pressure
        
        issues = []
        if temp < 18 or temp > 26:
            issues.append("temperature")
        if humidity < 30 or humidity > 70:
            issues.append("humidity")
        if pressure < 1000:
            issues.append("low pressure")
        
        if not issues:
            return "✅ All systems normal - Conditions are comfortable"
        else:
            return f"⚠️  Check: {', '.join(issues)}"

def main(args=None):
    rclpy.init(args=args)
    node = DashboardMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nDashboard shutting down...")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()