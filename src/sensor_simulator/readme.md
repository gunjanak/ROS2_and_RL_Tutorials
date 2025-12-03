# The Heartbeat of Robotics: Understanding ROS2's Publisher-Subscriber System

Welcome to the exciting world of ROS2! If you're just starting your journey into robotics software, you'll quickly encounter a fundamental concept that underpins almost everything: the **Publisher-Subscriber (Pub/Sub)** communication model. This isn't just a technical detail; it's the very heartbeat of how independent components in a robot talk to each other.

Let's dive into what Pub/Sub is, why it's so vital in ROS2, and why your first steps in this ecosystem often involve a "Hello World" example built around it.

---

## What is the Publisher-Subscriber System?

Imagine a bustling newsroom. You have reporters (publishers) constantly writing articles on various topics (sports, politics, weather). You also have readers (subscribers) who are only interested in specific topics. A reader interested in 'sports' doesn't care about 'politics', and a reporter writing about 'weather' does not need to know who is reading their articles.

This is precisely how Pub/Sub works in ROS2:

- **Publishers**: Nodes that generate and send data. They "publish" messages to a specific topic.
- **Subscribers**: Nodes that receive and process data. They "subscribe" to a specific topic and execute a "callback function".
- **Topics**: Named channels (like "temperature" or "robot_positions") over which messages flow. Publishers send messages to topics and subscribers receive messages from topics.

The key here is **decoupling**. Publishers and subscribers don't need to know about each other's existence. They only need to agree on the topic name and the type of message being exchanged.

---

## Significance in ROS2: The Power of Decentralization

The Pub/Sub model is absolutely critical for ROS2 because it enables:

- **Modularity**: Break down complex robotic systems into many small, manageable nodes, each responsible for a single task (e.g., camera driver, motor controller, navigation planner)
- **Decentralization**: No single point of failure. If one node crashes, others can often continue operating
- **Scalability**: Easily add new sensors, actuators, or processing algorithms by creating new nodes that publish or subscribe to relevant topics, without altering existing code
- **Concurrency**: Nodes run independently, often in parallel, utilizing modern multi-core processors effectively
- **Reusability**: Individual nodes can be reused across different robot platforms or applications

---

## Why is Pub/Sub the "Hello World" of ROS2?

When we learn a new programming language, our first program is often "Hello World" because it's the simplest way to demonstrate basic output. In ROS2, the Pub/Sub system serves a similar purpose:

- Introduces us to the core communication paradigm
- Demonstrates how to create nodes, the fundamental building blocks
- Shows how data flows through the ROS2 graph
- Highlights the use of client libraries (rclpy for Python) and message types

Mastering Pub/Sub is the gateway to understanding more complex ROS2 features like Services and Actions, which build upon this foundational concept.

---

## Tutorial: Building a Temperature Sensor Simulator

We will create our first ROS2 workspace, build a temperature sensor simulator with a Publisher Node and a Subscriber Node, configure the package, and run it.

### Step 1: Create Workspace

Create a directory named "ROS2_and_RL_Tutorials" and navigate into it:

```bash
mkdir ~/Documents/ROS2_and_RL_Tutorials/
cd ~/Documents/ROS2_and_RL_Tutorials/
```

Create necessary directories:

```bash
mkdir -p src docs scripts resources/images
```

**What `-p` does:**
- Creates parent directories if they don't exist
- Creates nested directories in one command
- Doesn't error if directory already exists

Navigate to src:

```bash
cd src
```

### Step 2: Create Custom Python Package

```bash
ros2 pkg create --build-type ament_python sensor_simulator --dependencies rclpy std_msgs
```

**Simple breakdown:**
- `ros2` – The ROS2 command-line tool
- `pkg create` – Command to create a new package
- `--build-type ament_python` – Make it a Python package (not C++)
- `sensor_simulator` – Name of your package
- `--dependencies rclpy std_msgs` – Automatically add these required libraries:
  - `rclpy` = ROS2 Python library (needed for all Python nodes)
  - `std_msgs` = Standard message types (like Float32, String, etc.)

### Step 3: Build and Load ROS2 Package

```bash
cd ~/Documents/ROS2_and_RL_Tutorials/
colcon build
source install/setup.bash
```

**What each command does:**
- `cd ~/Documents/ROS2_and_RL_Tutorials/` – Go to your workspace
- `colcon build` – Build all ROS2 packages in the workspace
- `source install/setup.bash` – Load the built packages so ROS2 can find them

---

## Creating the Temperature Publisher

### Navigate to Package Directory

```bash
cd src/sensor_simulator/sensor_simulator/
nano temperature_publisher.py
```

### What This Program Does

This program creates a ROS2 node that simulates a temperature sensor. Every second, it generates a realistic temperature reading by slightly adjusting the previous temperature value (adding or subtracting a small random amount), keeps the temperature within a reasonable range, packages this data into a message, and publishes it to a topic called 'temperature' so other nodes can receive and use this information.

### Code Breakdown

#### Import Section

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random
```

- `#!/usr/bin/env python3` – Tells the system to run this file with Python 3
- `import rclpy` – Imports the ROS2 Python library for all ROS2 functionality
- `from rclpy.node import Node` – Imports the Node base class to create ROS2 nodes
- `from std_msgs.msg import Float32` – Imports Float32 message type to send decimal numbers
- `import random` – Imports Python's random library to simulate temperature fluctuations

#### Class Definition

```python
class TemperatureSensor(Node):
```

- Creates a new class inheriting from Node, giving it all ROS2 node capabilities

#### Constructor (`__init__` method)

```python
def __init__(self):
    super().__init__('temperature_sensor')
    
    # Create publisher on "temperature" topic
    self.publisher_ = self.create_publisher(Float32, 'temperature', 10)
    
    # Set up timer to publish every 1 second
    self.timer = self.create_timer(1.0, self.publish_temperature)
    
    # Initial temperature
    self.temperature = 22.0
    
    self.get_logger().info("Temperature Sensor Node Started")
```

- `def __init__(self):` – Constructor that runs when creating a TemperatureSensor object
- `super().__init__('temperature_sensor')` – Initializes the parent Node class and names this node 'temperature_sensor'
- `self.publisher_ = self.create_publisher(Float32, 'temperature', 10)` – Creates a publisher object
  - `Float32` – Message type to publish (floating-point numbers)
  - `'temperature'` – Topic name where messages will be sent
  - `10` – QoS queue size (keeps last 10 messages if subscribers are slow)
- `self.timer = self.create_timer(1.0, self.publish_temperature)` – Creates a timer that calls a function repeatedly
  - `1.0` – Time interval in seconds (calls every 1 second)
  - `self.publish_temperature` – Function to call when timer triggers
- `self.temperature = 22.0` – Sets initial temperature to 22 degrees Celsius (room temperature)
- `self.get_logger().info(...)` – Logs a startup message to the terminal

#### Publishing Function

```python
def publish_temperature(self):
    # Simulate temperature fluctuation
    self.temperature += random.uniform(-0.5, 0.5)
    
    # Keep temperature in realistic range
    self.temperature = max(10.2, min(28.0, self.temperature))
    
    # Create and publish message
    msg = Float32()
    msg.data = self.temperature
    
    self.publisher_.publish(msg)
    self.get_logger().info(f'Publishing: {self.temperature:.2f}C')
```

- `def publish_temperature(self):` – Function called automatically every second by the timer
- `self.temperature += random.uniform(-0.5, 0.5)` – Simulates realistic temperature changes
  - `random.uniform(-0.5, 0.5)` – Generates a random number between -0.5 and +0.5
  - `+=` – Adds this random value to current temperature (making it go up or down slightly)
- `self.temperature = max(10.2, min(28.0, self.temperature))` – Constrains temperature to realistic bounds
  - `min(28.0, self.temperature)` – If temperature exceeds 28°C, cap it at 28°C
  - `max(10.2, ...)` – If temperature drops below 10.2°C, set it to 10.2°C
  - Result: Temperature stays between 10.2°C and 28°C
- `msg = Float32()` – Creates a new Float32 message object
- `msg.data = self.temperature` – Puts the temperature value into the message's data field
- `self.publisher_.publish(msg)` – Sends the message to all subscribers on the 'temperature' topic
- `self.get_logger().info(f'Publishing: {self.temperature:.2f}C')` – Logs what was published
  - `{self.temperature:.2f}` – Formats temperature to 2 decimal places

#### Main Function

```python
def main(args=None):
    rclpy.init(args=args)
    node = TemperatureSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()
```

- `def main(args=None):` – Entry point function
- `rclpy.init(args=args)` – Initializes ROS2 (must be called before creating nodes)
- `node = TemperatureSensor()` – Creates instance of the sensor node (triggers `__init__`)
- `try:` – Starts error handling block
- `rclpy.spin(node)` – Keeps node running and processes timer callbacks (infinite loop that calls `publish_temperature` every second)
- `except KeyboardInterrupt:` – Catches Ctrl+C
- `pass` – Does nothing, allows graceful exit
- `node.destroy_node()` – Cleans up the node resources (good practice)
- `rclpy.shutdown()` – Shuts down ROS2 cleanly

#### Script Entry Point

```python
if __name__ == '__main__':
    main()
```

- `if __name__ == '__main__':` – Checks if file is run directly (not imported)
- `main()` – Starts the program

### How Data Flows Through This Code

1. **Startup:** `main()` → initializes ROS2 → creates `TemperatureSensor` node → sets up publisher and timer
2. **Timer triggers (every 1 second):**
   - `publish_temperature()` is called automatically
   - Temperature is adjusted randomly
   - Temperature is clamped to range
   - Message is created and published
   - Log message displayed
3. **Repeat:** Timer keeps triggering every second
4. **Shutdown:** User presses Ctrl+C → node cleaned up → ROS2 shutdown

### Complete Publisher Code

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class TemperatureSensor(Node):
    def __init__(self):
        super().__init__('temperature_sensor')
        
        # Create publisher on "temperature" topic
        self.publisher_ = self.create_publisher(Float32, 'temperature', 10)
        
        # Set up timer to publish every 1 second
        self.timer = self.create_timer(1.0, self.publish_temperature)
        
        # Initial temperature
        self.temperature = 22.0
        
        self.get_logger().info("Temperature Sensor Node Started")
    
    def publish_temperature(self):
        # Simulate temperature fluctuation
        self.temperature += random.uniform(-0.5, 0.5)
        
        # Keep temperature in realistic range
        self.temperature = max(10.2, min(28.0, self.temperature))
        
        # Create and publish message
        msg = Float32()
        msg.data = self.temperature
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {self.temperature:.2f}C')

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
```

---

## Creating the Temperature Subscriber

### What This Program Does

This program creates a ROS2 node that acts as a temperature monitoring system. It listens to temperature data being published on a topic called 'temperature', receives each temperature reading as it arrives, evaluates whether the temperature is too high, too low, or normal, and then logs an alert message with the current temperature and its status.

### Code Breakdown

#### Imports Section

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
```

- `#!/usr/bin/env python3` – Shebang line that tells the system to run this file with Python 3
- `import rclpy` – Imports the ROS2 Python client library (needed for all ROS2 functionality)
- `from rclpy.node import Node` – Imports the Node class, which is the base class for creating ROS2 nodes
- `from std_msgs.msg import Float32` – Imports the Float32 message type to receive decimal number data

#### Class Definition

```python
class TemperatureMonitor(Node):
```

- Creates a new class that inherits from the ROS2 Node class, giving it all the capabilities of a ROS2 node

#### Constructor (`__init__` method)

```python
def __init__(self):
    super().__init__('temperature_monitor')
    
    # Create subscriber to 'temperature' topic
    self.subscription = self.create_subscription(
        Float32,
        'temperature',
        self.temperature_callback,
        10
    )
    
    self.get_logger().info('Temperature Monitor Node Started')
```

- `def __init__(self):` – Constructor method that runs when you create a TemperatureMonitor object
- `super().__init__('temperature_monitor')` – Calls the parent Node class constructor and names this node 'temperature_monitor'
- `self.subscription = self.create_subscription(...)` – Creates a subscriber object that listens for messages
  - `Float32` – The type of message to expect (floating-point numbers)
  - `'temperature'` – The name of the topic to listen to
  - `self.temperature_callback` – The function to call when a message arrives
  - `10` – Quality of Service (QoS) queue size – keeps the last 10 messages if processing is slow
- `self.get_logger().info(...)` – Logs an informational message to the terminal when the node starts

#### Callback Function

```python
def temperature_callback(self, msg):
    temp = msg.data
    
    # Simple alert system
    if temp > 26.0:
        status = "High"
    elif temp < 20.0:
        status = "Low"
    else:
        status = "Normal"
    
    self.get_logger().info(f'Temperature: {temp:.2f}C - Status:{status}')
```

- `def temperature_callback(self, msg):` – Function that gets called automatically every time a new temperature message arrives
- `temp = msg.data` – Extracts the actual temperature value from the message (Float32 messages have a `.data` field)
- `if temp > 26.0:` – Checks if temperature is above 26 degrees
  - `status = "High"` – Sets status to "High" if temperature exceeds threshold
- `elif temp < 20.0:` – Checks if temperature is below 20 degrees
  - `status = "Low"` – Sets status to "Low" if temperature is too cold
- `else:` – If temperature is between 20 and 26
  - `status = "Normal"` – Sets status to "Normal" for comfortable range
- `self.get_logger().info(f'...')` – Logs the temperature and status
  - `{temp:.2f}` – Formats temperature to 2 decimal places
  - `f'...'` – f-string formatting to insert variables into the string

#### Main Function

```python
def main(args=None):
    rclpy.init(args=args)
    node = TemperatureMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
```

- `def main(args=None):` – Entry point function that starts the program
- `rclpy.init(args=args)` – Initializes the ROS2 Python client library (must be called before creating nodes)
- `node = TemperatureMonitor()` – Creates an instance of the TemperatureMonitor class (this triggers `__init__`)
- `try:` – Begins a try-except block for error handling
- `rclpy.spin(node)` – Keeps the node running and processing callbacks indefinitely (like an infinite loop that responds to incoming messages)
- `except KeyboardInterrupt:` – Catches Ctrl+C keyboard interrupt
- `pass` – Does nothing, allowing clean exit when user presses Ctrl+C

#### Script Entry Point

```python
if __name__ == "__main__":
    main()
```

- `if __name__ == "__main__":` – Checks if this file is being run directly (not imported as a module)
- `main()` – Calls the main function to start the program

### How Data Flows Through This Code

1. **Startup:** `main()` → initializes ROS2 → creates `TemperatureMonitor` node → sets up subscriber
2. **Waiting:** `rclpy.spin(node)` keeps the program alive, waiting for messages
3. **Message arrives:** When publisher sends temperature data → `temperature_callback()` is triggered automatically
4. **Processing:** Callback extracts temperature → evaluates status → logs result
5. **Repeat:** Goes back to waiting for next message
6. **Shutdown:** User presses Ctrl+C → exits cleanly

This subscriber node is **passive** – it just waits and reacts to data published by the temperature_publisher node!

### Complete Subscriber Code

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class TemperatureMonitor(Node):
    def __init__(self):
        super().__init__('temperature_monitor')
        
        # Create subscriber to 'temperature' topic
        self.subscription = self.create_subscription(
            Float32,
            'temperature',
            self.temperature_callback,
            10
        )
        
        self.get_logger().info('Temperature Monitor Node Started')
    
    def temperature_callback(self, msg):
        temp = msg.data
        
        # Simple alert system
        if temp > 26.0:
            status = "High"
        elif temp < 20.0:
            status = "Low"
        else:
            status = "Normal"
        
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
```

---

## Key Differences Between Publisher and Subscriber

| Publisher | Subscriber |
|-----------|------------|
| **Sends** data | **Receives** data |
| Uses `create_publisher()` | Uses `create_subscription()` |
| Uses **timer** (proactive – generates data) | Uses **callback** (reactive – waits for data) |
| Calls `publish()` to send | Callback triggered automatically when data arrives |
| Generates simulated data | Processes received data |

---

## Make Python Code Executable

```bash
chmod +x temperature_publisher.py
chmod +x temperature_subscriber.py
```

**What this does:**
- `chmod` = "change mode" – modifies file permissions
- `+x` = adds "execute" permission, making the file runnable as a program
- After this, both Python scripts can be executed directly by ROS2's `ros2 run` command

---

## Edit the Setup File

Navigate to the package directory and edit setup.py:

```bash
cd ~/Documents/ROS2_and_RL_Tutorials/src/sensor_simulator/
nano setup.py
```

### Complete setup.py

```python
from setuptools import find_packages, setup

package_name = 'sensor_simulator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='janak',
    maintainer_email='janak.klal@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'temperature_publisher = sensor_simulator.temperature_publisher:main',
            'temperature_subscriber = sensor_simulator.temperature_subscriber:main',
        ],
    },
)
```

### Function of setup.py

**setup.py** is the configuration file that tells Python and ROS2 how to install and run your package. It defines:
- Package metadata (name, version, author)
- What files to include
- Dependencies needed
- **Most importantly:** Which Python scripts become executable commands

Think of it as the "instruction manual" for installing your package – it tells `colcon build` what to do with your code.

### What are Entry Points?

**entry_points** create command-line executables from your Python functions. They're the bridge between typing `ros2 run` and your actual Python code.

**Format:** `'command_name = package.module:function'`

Example:
```python
'temperature_publisher = sensor_simulator.temperature_publisher:main'
```

- `temperature_publisher` = The command you type in terminal (`ros2 run sensor_simulator temperature_publisher`)
- `sensor_simulator.temperature_publisher` = The Python file path (`sensor_simulator/temperature_publisher.py`)
- `:main` = The specific function to run (the `main()` function in that file)

---

## Build, Load and Run

### Build and Source the Package

```bash
cd ~/Documents/ROS2_and_RL_Tutorials/
colcon build --packages-select sensor_simulator
source install/setup.bash
```

**What each command does:**
- `cd ~/Documents/ROS2_and_RL_Tutorials/` – Changes your current directory to the workspace folder
- `colcon build --packages-select sensor_simulator` – Compiles/builds only the sensor_simulator package, creating executables from your Python code
- `source install/setup.bash` – Loads the built package into your terminal session so ROS2 knows where to find your executables

### Run the Publisher (Terminal 1)

```bash
ros2 run sensor_simulator temperature_publisher
```

- Runs the temperature_publisher executable from the sensor_simulator package, starting your temperature sensor node

### Run the Subscriber (Terminal 2)

Open a new terminal and run:

```bash
cd ~/Documents/ROS2_and_RL_Tutorials/
source install/setup.bash
ros2 run sensor_simulator temperature_subscriber
```

---

## Expected Output

**Terminal 1 (Publisher):**
```
[INFO] [temperature_sensor]: Temperature Sensor Node Started
[INFO] [temperature_sensor]: Publishing: 22.15°C
[INFO] [temperature_sensor]: Publishing: 22.43°C
[INFO] [temperature_sensor]: Publishing: 22.67°C
...
```

**Terminal 2 (Subscriber):**
```
[INFO] [temperature_monitor]: Temperature Monitor Node Started
[INFO] [temperature_monitor]: Temperature: 22.43°C - Status: Normal
[INFO] [temperature_monitor]: Temperature: 22.67°C - Status: Normal
[INFO] [temperature_monitor]: Temperature: 26.15°C - Status: High
...
```

---

## Congratulations!

You've successfully created your first ROS2 Publisher-Subscriber system! You now understand:

- ✅ The fundamental Pub/Sub communication pattern
- ✅ How to create ROS2 nodes in Python
- ✅ How publishers generate and send data
- ✅ How subscribers receive and process data
- ✅ The role of topics in ROS2 communication
- ✅ How to build and run ROS2 packages

This is the foundation for all ROS2 applications – from simple sensors to complex autonomous robots!

---

## Next Steps

In the next tutorial, we'll explore:
- Creating custom message types
- Understanding Quality of Service (QoS) settings
- Building multi-sensor data aggregators
- Best practices for node design

---

## Resources

- [Full blog post](https://janak-lal.com.np/the-heartbeat-of-robotics-understanding-ros2s-publisher-subscriber-system/)
- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [GitHub Repository](https://github.com/gunjanak/ROS2_and_RL_Tutorials)

---

**Happy Coding! 🤖**