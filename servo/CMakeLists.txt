cmake_minimum_required(VERSION 3.10)
project(HLS_Servo_Project)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(ROS_VERSION 2)

if(DEFINED ROS_VERSION)
    message(STATUS "Using ROS version: ${ROS_VERSION}")
else()
    set(ROS_VERSION 1)  # Set to 1 for ROS Noetic, 2 for ROS Humble
    message(STATUS "ROS_VERSION is not defined. Default is 1.")
endif()

add_definitions(-DROS_VERSION=${ROS_VERSION})

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find required packages
find_package(PkgConfig REQUIRED)

# Try to find serial library in different locations
set(SERIAL_FOUND FALSE)

if(ROS_VERSION EQUAL 1)
    find_path(SERIAL_INCLUDE_DIR serial/serial.h
        PATHS
        /opt/ros/noetic/include
        NO_DEFAULT_PATH
    )


    find_library(SERIAL_LIBRARY
        NAMES serial
        PATHS
        /opt/ros/noetic/lib
        NO_DEFAULT_PATH
    )

    if(SERIAL_INCLUDE_DIR AND SERIAL_LIBRARY)
        set(SERIAL_FOUND TRUE)
        set(SERIAL_INCLUDE_DIRS ${SERIAL_INCLUDE_DIR})
        set(SERIAL_LIBRARIES ${SERIAL_LIBRARY})
        message(STATUS "Found ROS serial library: ${SERIAL_LIBRARY}")
    else()
        message(FATAL_ERROR "ROS serial library not found. Please check your ROS installation.")
    endif()
endif()

if(ROS_VERSION EQUAL 2)
    find_path(SERIAL_DRIVER_INCLUDE_DIR serial_driver/serial_driver.hpp
        PATHS
        /opt/ros/humble/include
        NO_DEFAULT_PATH
    )

    find_library(SERIAL_DRIVER_LIBRARY
        NAMES serial_driver
        PATHS
        /opt/ros/humble/lib
        NO_DEFAULT_PATH
    )

    if(SERIAL_DRIVER_INCLUDE_DIR AND SERIAL_DRIVER_LIBRARY)
        set(SERIAL_DRIVER_FOUND TRUE)
        set(SERIAL_DRIVER_INCLUDE_DIRS ${SERIAL_DRIVER_INCLUDE_DIR})
        set(SERIAL_DRIVER_LIBRARIES ${SERIAL_DRIVER_LIBRARY})
        message(STATUS "Found ROS serial_driver library: ${SERIAL_DRIVER_LIBRARY}")
    else()
        message(FATAL_ERROR "ROS serial_driver library not found. Please check your ROS installation.")
    endif()

    find_package(io_context REQUIRED) 

endif()

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)
if(ROS_VERSION EQUAL 1)
    include_directories(${SERIAL_INCLUDE_DIRS})
endif()
if(ROS_VERSION EQUAL 2)
    include_directories(${SERIAL_DRIVER_INCLUDE_DIRS})
endif()

# Add executable
add_executable(hls_test src/HLS/test.cpp src/HLS/HLS.cpp)

# Link libraries
if(ROS_VERSION EQUAL 1)
    target_link_libraries(hls_test ${SERIAL_LIBRARIES})
endif()
if(ROS_VERSION EQUAL 2)
    target_link_libraries(hls_test 
    ${SERIAL_DRIVER_LIBRARIES}
    /opt/ros/humble/lib//libio_context.so )
endif()



# Print information
if(ROS_VERSION EQUAL 1)
    message(STATUS "Serial library found: ${SERIAL_FOUND}")
    message(STATUS "Serial include dirs: ${SERIAL_INCLUDE_DIRS}")
    message(STATUS "Serial libraries: ${SERIAL_LIBRARIES}")
endif()

if(ROS_VERSION EQUAL 2)
    message(STATUS "Serial driver library found: ${SERIAL_DRIVER_FOUND}")
    message(STATUS "Serial driver include dirs: ${SERIAL_DRIVER_INCLUDE_DIRS}")
    message(STATUS "Serial driver libraries: ${SERIAL_DRIVER_LIBRARIES}")
endif()

# Add installation target
install(TARGETS hls_test DESTINATION bin)