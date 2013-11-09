This guide will teach you how to build and run this program.

GENERAL PRE-REQUISITES:

1. Cmake : To automate the build process.
2. Aria : To control the robots
3. PCL : For point clouds and visualization.
4. OpenCV : For kalman filtering of robot location.

BUILDING:

1. It is recommended to create a separate build directory to house the
object files and executable. Preferrably called build within the directory
for the server program.

2. Go into your build directory and run the following command:
    cmake ..

3. Now run make from within the build directory.

USAGE:

Type the following command from the build directory run the program. There
are sample configuration files available in sampleConfigs directory.
  client -hosts [PATH TO CONFIGURATION FILE]

Now you can run the client program to connect to this running server.
