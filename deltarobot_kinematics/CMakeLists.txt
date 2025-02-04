cmake_minimum_required(VERSION 3.8)
project(deltarobot_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Doxygen)
find_package(rclcpp REQUIRED)
find_package(deltarobot_interfaces REQUIRED)

include_directories(
  include
  ${deltarobot_interfaces_INCLUDE_DIRS}
)

# Delta Kinematics Node
add_executable(kinematics src/kinematics.cpp)
ament_target_dependencies(kinematics rclcpp deltarobot_interfaces)
target_link_libraries(kinematics ${deltarobot_interfaces_LIBRARIES})

# Read/Write Node (Example from Dynamixel SDK)
# add_executable(read_write_node src/read_write_node.cpp)
# ament_target_dependencies(read_write_node
# dynamixel_sdk_custom_interfaces
# dynamixel_sdk
# rclcpp
# )
# target_link_libraries(read_write_node dynamixel_sdk)
# install(TARGETS
# read_write_node
# DESTINATION lib/${PROJECT_NAME}
# )

# Install
install(TARGETS
  kinematics
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

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

# Building documentation should be optional.
# To build documentation pass -DBUILD_DOCS=ON when generating the build system
option(BUILD_DOCS "Build the documentation" OFF)

# build just because Doxygen is missing
if(${DOXYGEN_FOUND} AND ${BUILD_DOCS})
  # Turn the README.md into the homepage of the doxygen docs
  set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)

  # Tell Doxygen where to find the documentation
  doxygen_add_docs(doxygen include/ src/ README.md ALL)

  # The documentation will be in the build/html directory
  # The main page is build/html/index.html
endif()

ament_package()
