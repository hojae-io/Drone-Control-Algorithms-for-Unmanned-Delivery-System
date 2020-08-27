cmake_minimum_required(VERSION 2.8.3)
project(targetDetection_topic)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  roscpp
)

find_package(OpenCV REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   TargetPosition.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  LIBRARIES targetDetection_topic
  CATKIN_DEPENDS std_msgs roscpp
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


 add_executable(target_publisher src/target_publisher.cpp)

 add_dependencies(target_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(target_publisher
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
 )

add_executable(subscriber2 src/subscriber.cpp)
add_dependencies(subscriber2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(subscriber2
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
)
