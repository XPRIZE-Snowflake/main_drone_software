cmake_minimum_required(VERSION 3.8)
project(img_capture)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(/usr/include)
link_directories(/usr/lib)

add_executable(seekcamera_publisher src/seekcamera_publisher.cpp)
ament_target_dependencies(seekcamera_publisher rclcpp sensor_msgs)

# Link against the Seek Thermal library
target_link_libraries(seekcamera_publisher seekcamera)

# Install C++ node
install(TARGETS
  seekcamera_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python node
install(PROGRAMS
  scripts/thermal_subscriber.py
  scripts/odometry_publisher.py
  scripts/kmeans.py
  scripts/low_alt_thermal.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
