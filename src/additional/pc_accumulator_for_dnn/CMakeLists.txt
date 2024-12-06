cmake_minimum_required(VERSION 3.8)
project(pc_accumulator_for_dnn)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(valo_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(PCL REQUIRED)
#find_package(pcl_ros REQUIRED)
#find_package(pcl_conversions REQUIRED)

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

# Add executable
add_library(pc_accumulator_for_dnn SHARED src/pc_accumulator_for_dnn_node.cpp)
target_include_directories(pc_accumulator_for_dnn
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  #  ${PCL_INCLUDE_DIRS})
  target_link_libraries(pc_accumulator_for_dnn Eigen3::Eigen) #${PCL_LIBRARIES})
ament_target_dependencies(pc_accumulator_for_dnn rclcpp rclcpp_components sensor_msgs 
  geometry_msgs valo_msgs) # pcl_ros pcl_conversions)
rclcpp_components_register_nodes(pc_accumulator_for_dnn "pc_acc_for_dnn::PCloudAccForDnnComponent")

# Install executable
install(TARGETS pc_accumulator_for_dnn
  DESTINATION lib/)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
