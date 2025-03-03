cmake_minimum_required(VERSION 3.16)
project(hydra_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  message_generation
)

add_message_files(
  FILES
  ActiveLayer.msg
  DsgUpdate.msg
  Mask.msg
  Masks.msg
  HydraVisionPacket.msg
  AABB.msg
  InstanceViewHeader.msg
  ObjectLayerInfo.msg
  ObjectNodeInfo.msg
)
add_service_files(FILES GetDsg.srv QueryFreespace.srv)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs 
  sensor_msgs
)

catkin_package(
 CATKIN_DEPENDS std_msgs sensor_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
