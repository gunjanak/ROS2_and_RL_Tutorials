cmake_minimum_required(VERSION 3.8)
project(multi_sensor_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)

# Generate interfaces (messages)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorData.msg"
  DEPENDENCIES std_msgs
)

# Install Python scripts
install(PROGRAMS
  scripts/temperature_sensor.py
  scripts/humidity_sensor.py
  scripts/pressure_sensor.py
  scripts/dashboard_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()