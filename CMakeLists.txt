cmake_minimum_required(VERSION 3.8)
project(pt_potential_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
#find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgcodecs imgproc videoio)
find_package(tuw_geometry REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(pt_msgs)

include_directories(include/pt_potential_controller ${YAML_CPP_INCLUDE_DIRS})

add_executable(controller_node
  src/controller_main.cpp
  src/controller_node.cpp
  src/anchor.cpp
  src/potential.cpp
  src/scenario.cpp
  src/scenario_loader.cpp
)
target_compile_features(controller_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# ament_target_dependencies doesnt work for yaml-cpp --> use target_link_libraries
target_link_libraries(controller_node ${YAML_CPP_LIBRARIES})
ament_target_dependencies(controller_node
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  OpenCV
  tuw_geometry
  pt_msgs
)

install(TARGETS controller_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
    #launch
    config
  DESTINATION
    share/${PROJECT_NAME}/
)
install(
  DIRECTORY include/
  DESTINATION include
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
