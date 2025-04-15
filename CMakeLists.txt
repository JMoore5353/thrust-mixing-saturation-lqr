cmake_minimum_required(VERSION 3.8)
project(thrust_mixing_lqr)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -fPIC)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(roscopter_msgs REQUIRED)
find_package(rosflight_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Create executable
add_executable(lqr_rate_controller
  src/lqr.cpp)
target_include_directories(lqr_rate_controller
  PRIVATE
    include)
target_link_libraries(lqr_rate_controller Eigen3::Eigen)
ament_target_dependencies(lqr_rate_controller
  rclcpp
  rosflight_msgs
  roscopter_msgs
)
install(
  TARGETS lqr_rate_controller
  DESTINATION lib/${PROJECT_NAME})

add_executable(rc_transcription
  src/rc_transcription.cpp)
ament_target_dependencies(rc_transcription
  rclcpp
  rosflight_msgs)
install(
  TARGETS rc_transcription
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY params
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
