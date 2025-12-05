#Custom message for sensor readings
#This bundles multiple sensor values together

#Header with timestamp
std_msgs/Header header

#Sensor identification
string sensor_name
string sensor_type

#Sensor readings
float32 temperature     # Temperature in Celsius
float32 humidity        # Relative humidity (0-100%)
float32 pressure        # Atmospheric pressure in hPa

#Status information
bool is_active
string status_message
