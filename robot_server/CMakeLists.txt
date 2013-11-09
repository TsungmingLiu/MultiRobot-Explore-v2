cmake_minimum_required(VERSION 2.6)

project(SERVER)

# use cmake to find PCL library
find_package(PCL 1.3 REQUIRED 
  COMPONENTS common io visualization filters)

# compile stereoCamera.cpp only if robot has a stereo camera
# change to OFF to ON for stereo camera

#option (STEREO_CAMERA
  #"Use stereo camera" OFF)
#if (STEREO_CAMERA)
  #set(SRC_STEREO stereoCamera.cpp)
  ## Include headers for the stereocamera
  #include_directories(/usr/local/BumbleBee2/newPGR/pgrlibdcstereo)
  #include_directories(/usr/local/triclops/include)
  ## Required libraries for stereocamera
  #set(StereoCamera_LIBS dc1394 pgrlibdcstereo triclops pnmutils)
  #add_definitions(-DSTEREO_CAMERA)
#endif (STEREO_CAMERA)

# Source files
set(SRC ${SRC_STEREO} utils.cc compress.cc SensorData.cc server.cc RobotMap.cc)
# Program name
set(PROG server)

# Include Aria headers
include_directories(/usr/local/Aria/include/)
# Include ArNetworking headers
include_directories(/usr/local/Aria/ArNetworking/include/)
# Include OpenCV headers
include_directories(/usr/local/include/opencv/)
# PCL headers location
include_directories(${PCL_INCLUDE_DIRS})

# Aria libraries location
link_directories(/usr/local/Aria/lib/)
# Location of libraries for stereocamera
#link_directories(/usr/local/BumbleBee2/newPGR/pgrlibdcstereo)
#link_directories(/usr/local/triclops/lib)
# PCL libraries location and definitions
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(-g -Wall)
# Create the executable
add_executable(${PROG} ${SRC})

# Required Aria libraries
set(LIBS_ARIA Aria ArNetworking pthread dl rt)
# Required OpenCV libraries
set(OpenCV_LIBS opencv_core opencv_highgui opencv_imgproc)
# Required Aria libraries
set(Aria_LIBS Aria ArNetworking pthread dl rt)
# Link libraries
target_link_libraries(${PROG} ${LIBS_ARIA} 
  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}
  ${PCL_VISUALIZATION_LIBRARIES}
  ${PCL_FILTERS_LIBRARIES}
  ${OpenCV_LIBS} ${StereoCamera_LIBS})
