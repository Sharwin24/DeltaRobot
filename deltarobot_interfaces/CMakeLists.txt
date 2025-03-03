cmake_minimum_required(VERSION 3.8)
project(deltarobot_interfaces)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

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

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DeltaJoints.msg"
  "msg/FKTrajectory.msg"
  "msg/IKTrajectory.msg"
  "srv/DeltaFK.srv"
  "srv/DeltaIK.srv"
  "srv/GetDynamixelPositions.srv"
  "srv/PlayFKTrajectory.srv"
  "srv/PlayIKTrajectory.srv"
  "srv/PlayDemoTrajectory.srv"
  "srv/ConvertToJointTrajectory.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_package()
