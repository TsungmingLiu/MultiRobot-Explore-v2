This guide will teach you how to build and run this program.

GENERAL PRE-REQUISITES:

1. Cmake : To automate the build process.
2. Aria : To control the robots

STEREOCAMERA PRE-REQUISITES:

1. triclops and BumbleBee2 : Stereocamera libraries
2. OpenCV : Required by stereocamera libraries.

BUILDING:

1. Edit CMakeLists.txt file line 8:

  a) For stereocamera: ON
  b) For laser: OFF

2. It is recommended to create a separate build directory to house the
object files and executable. Preferrably called build within the directory
for the server program.

3. Go into your build directory and run the following command:
    cmake ..

4. Now run make from within the build directory.

USAGE:

Type the following command from the build directory run the program:
  server

The following will connect the laser as well.
  server -cl

For lasers, configuration files are provided to transform the data from
laser center to robot center in sampleConfigs directory. Use the following
command:
  server -file [PATH TO FILE]

Now you can run the client program to connect to this running server.

USE WITH MOBILESIM:

1. The following program will start MobileSim with 2 robots. You can use as
many robots as you want.
  MobileSim -m [PATH TO MAP] -r [ROBOT TYPE] -r [ROBOT TYPE]
  e.g. MobileSim -m /usr/local/MobileSim/AMROffice.map -r p3dx -r p3dx

2. By default, MobileSim will simulate robots starting on TCP port 8101.
Extra robots will get incremental port values.

3. Start server program on MobileSim's simulated robots. The following
command will create a server on SERVER PORT on the simulated robot running
on ROBOT PORT. Create as many servers as you have robots.
  server -rrtp [ROBOT PORT] -sp [SERVER PORT] -cl
  e.g. server -rrtp 8101 -sp 7000 -cl

4. Now you can run the client program to connect to these servers using the
hosts configuration file.
