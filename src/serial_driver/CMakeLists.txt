cmake_minimum_required(VERSION 3.5.0)
project(serial_driver)

## Compile as C++14, supported in ROS Noetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy serial std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/serial_driver.cpp
)

## Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare C++ executables
add_executable(serial_test src/main.cpp)
add_executable(simple_serial_test src/simple_serial_test.cpp)
add_executable(frame_usage_example src/frame_usage_example.cpp)

## Add cmake target dependencies of the executables
add_dependencies(serial_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(simple_serial_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(frame_usage_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${serial_LIBRARIES}
  pthread
)

target_link_libraries(serial_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${serial_LIBRARIES}
  pthread
)

target_link_libraries(simple_serial_test
  ${catkin_LIBRARIES}
  ${serial_LIBRARIES}
  pthread
)

target_link_libraries(frame_usage_example
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${serial_LIBRARIES}
  pthread
)

#############
## Install ##
#############

## Mark executables for installation
install(TARGETS serial_test simple_serial_test frame_usage_example
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
) 