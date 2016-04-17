cmake_minimum_required(VERSION 2.8.3)
project(worldtime)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  std_msgs
  message_generation
)

add_message_files(
    FILES
    timemsg.msg
 )
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
CATKIN_DEPENDS message_runtime
)






## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )







include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(worldtime src/worldtime.cpp)
target_link_libraries(worldtime ${catkin_LIBRARIES} )


