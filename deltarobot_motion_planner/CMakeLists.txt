cmake_minimum_required(VERSION 3.8)
project(deltarobot_motion_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(deltarobot_interfaces REQUIRED)
find_package(kalman_filter REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include/deltarobot_motion_planner
  ${deltarobot_interfaces_INCLUDE_DIRS}
)

# Delta Testing Node
add_executable(delta_motion_planner src/delta_motion_planner.cpp)
ament_target_dependencies(delta_motion_planner rclcpp deltarobot_interfaces geometry_msgs)
target_link_libraries(delta_motion_planner ${deltarobot_interfaces_LIBRARIES})

# Install
install(TARGETS
  delta_motion_planner
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Include launch files from deltarobot_trajectory_generator
install(DIRECTORY
  ../deltarobot_trajectory_generator/launch
  DESTINATION share/${PROJECT_NAME}
)

# Include launch files from deltarobot_kinematics
install(DIRECTORY
  ../deltarobot_kinematics/launch
  DESTINATION share/${PROJECT_NAME}
)

# Include launch files from deltarobot_motor_control
install(DIRECTORY
  ../deltarobot_motor_control/launch
  DESTINATION share/${PROJECT_NAME}
)

# Include launch files from kalman_filter
install(DIRECTORY
  ../kalman_filter/kalman_filter/launch
  DESTINATION share/${PROJECT_NAME}
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
