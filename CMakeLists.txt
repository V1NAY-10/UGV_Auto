cmake_minimum_required(VERSION 3.5)
project(ugv_auto)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY config description launch maps worlds
  DESTINATION share/${PROJECT_NAME}
)

# Python nodes: ros2 run ugv_auto <script>.py
install(
  PROGRAMS
    scripts/esp32_bridge.py
    scripts/cmd_vel_odom.py
    scripts/go_to_goal.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
