cmake_minimum_required(VERSION 3.8)
project(robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(xacro REQUIRED)


#Add Executables
add_executable(motion_control src/motion_control.cpp)


ament_target_dependencies(motion_control rclcpp std_msgs sensor_msgs geometry_msgs trajectory_msgs)

# Define a CMake variable named dependencies that lists all
# ROS 2 packages and other dependencies the project requires.
set(dependencies
  rclcpp
  std_msgs
  trajectory_msgs
  ros2_control
  ros2_controllers
  gazebo_ros2_control
  gazebo_ros_pkgs
)

install(TARGETS
	motion_control
	DESTINATION lib/${PROJECT_NAME}
	)

	
install(DIRECTORY
	launch
	~/Ros2_Workspace/src/CRX-10iA_L_description/urdf
	~/Ros2_Workspace/src/CRX-10iA_L_description/meshes
	~/Ros2_Workspace/src/CRX-10iA_L_description/config
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

ament_package()
