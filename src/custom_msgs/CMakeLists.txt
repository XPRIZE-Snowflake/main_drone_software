cmake_minimum_required(VERSION 3.8)
project(custom_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate the custom messages (this must come BEFORE ament_package())
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomOdometryMsg.msg"
  # Add other .msg or .srv files here if necessary.
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright checks if needed.
  set(ament_cmake_copyright_FOUND TRUE)
  # Skip cpplint (adjust if this package is in a git repo).
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
