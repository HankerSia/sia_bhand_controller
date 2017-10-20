cmake_minimum_required(VERSION 2.8.3)
project(id_data_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  message_runtime
  sensor_msgs
  std_msgs
)




 add_message_files(
   FILES
   ID_Data.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )


 generate_messages(
   DEPENDENCIES
   geometry_msgs   sensor_msgs   std_msgs
 )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES id_data_msgs
  CATKIN_DEPENDS geometry_msgs message_generation message_runtime sensor_msgs std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)


