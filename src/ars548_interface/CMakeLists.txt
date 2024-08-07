cmake_minimum_required(VERSION 3.8)
project(ars548_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# set(msg_files
#   "msg/Objects.msg"
#   "msg/ObjectList.msg"
#   "msg/Detections.msg"
#   "msg/DetectionList.msg"
#   "msg/RadarBasicStatus.msg"
#   "msg/AccelerationLateralCog.msg"
#   "msg/AccelerationLongitudinalCog.msg"
#   "msg/CharacteristicSpeed.msg"
#   "msg/DrivingDirection.msg"
#   "msg/SteeringAngleFrontAxle.msg"
#   "msg/VelocityVehicle.msg"
#   "msg/YawRate.msg"
# )

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "msg/Objects.msg"
  "msg/ObjectList.msg"
  "msg/Detections.msg"
  "msg/DetectionList.msg"
  "msg/RadarBasicStatus.msg"
  "msg/AccelerationLateralCog.msg"
  "msg/AccelerationLongitudinalCog.msg"
  "msg/CharacteristicSpeed.msg"
  "msg/DrivingDirection.msg"
  "msg/SteeringAngleFrontAxle.msg"
  "msg/VelocityVehicle.msg"
  "msg/YawRate.msg"
  # "msg/ObjectPosition.msg"
  # "srv/AddTwoInts.srv"
  # "srv/GetObjectPosition.srv"
  # "action/MoveCircle.action"
  DEPENDENCIES std_msgs
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
