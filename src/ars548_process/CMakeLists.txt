cmake_minimum_required(VERSION 3.8)
project(ars548_process)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ars548_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED) 
find_package(pcl_conversions REQUIRED)  
#测试节点
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp ars548_interface)         # CHANGE
#测试节点
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp ars548_interface)     # CHANGE
#ars548_process_node节点，接收udp数据，解析数据，发布ros消息
add_executable(ars548_process_node src/ars548_process_node.cpp src/udp_interface.cpp src/data_process.cpp src/converttype.cpp)
ament_target_dependencies(ars548_process_node rclcpp std_msgs ars548_interface sensor_msgs geometry_msgs) 
#info_convert_node节点，接收ros消息,处理成rviz可用消息发布
add_executable(info_convert_node src/info_convert_node.cpp)
ament_target_dependencies(info_convert_node rclcpp std_msgs ars548_interface sensor_msgs geometry_msgs PCL pcl_conversions) 
#测试节点
add_executable(node_helloworld_class src/node_helloworld_class.cpp)
ament_target_dependencies(node_helloworld_class rclcpp)
#pcl 测试节点
add_executable(pcl_test src/pcl_test.cpp)
ament_target_dependencies(pcl_test rclcpp std_msgs ars548_interface sensor_msgs geometry_msgs PCL pcl_conversions)
#install 必须加，否则source后ros2 run无法识别指令
install(TARGETS
  talker
  listener
  node_helloworld_class
  ars548_process_node
  info_convert_node
  pcl_test
  DESTINATION lib/${PROJECT_NAME})

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
