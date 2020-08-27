cmake_minimum_required(VERSION 2.8.3)
project(GPS_topic)

#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lgps")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

 add_message_files(
   FILES
   Target_GPS.msg
 )


 generate_messages(
   DEPENDENCIES
   std_msgs
 )


catkin_package(
  LIBRARIES GPS_topic
  #LIBRARIES libgps-dev
  CATKIN_DEPENDS roscpp std_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(GPS_publisher src/GPS_publisher.cpp)

add_dependencies(GPS_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(GPS_publisher
   ${catkin_LIBRARIES}
   /usr/lib/x86_64-linux-gnu/libgps.so
 )

