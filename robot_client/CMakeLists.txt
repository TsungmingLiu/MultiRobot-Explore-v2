cmake_minimum_required(VERSION 2.6)
project(CLIENT)

# Source files
set(SRC
    RobotMap.cc helpers.cc ConfigFileReader.cc MoveHandler.cc compress.cc
    SensorDataHandler.cc SensorDataViewer.cc client.cc)
# Program name
set(PROG client)

# use cmake to find PCL library
find_package(PCL 1.3 REQUIRED 
  COMPONENTS common io visualization filters)

# Aria headers location
include_directories(/usr/local/Aria/include/)
# ArNetworking headers location
include_directories(/usr/local/Aria/ArNetworking/include/)
# PCL headers location
include_directories(${PCL_INCLUDE_DIRS})
# OpenCV headers location for FRCV lab (Ubuntu 12 32bit)
include_directories(/usr/include/opencv)
# OpenCV headers location home (Linux Mint 64bit)
include_directories(/usr/local/include/opencv)

# Aria libraries location
link_directories(/usr/local/Aria/lib/)
# PCL libraries location and definitions
link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})
# OpenCV libraries are in default location

# Compilation flags
add_definitions(-g -Wall -Wno-deprecated-declarations -Wno-unknown-pragmas)
# Create the executable
add_executable(${PROG} ${SRC})

# Link the libraries
# Required Aria libraries
set(Aria_LIBS Aria ArNetworking pthread dl rt)
# Required OpenCV libraries
set(OpenCV_LIBS opencv_core opencv_video)
target_link_libraries(${PROG} 
  ${Aria_LIBS}
  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}
  ${PCL_VISUALIZATION_LIBRARIES}
  ${PCL_FILTERS_LIBRARIES}
  ${OpenCV_LIBS}
)
