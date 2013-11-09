#ifndef UTILS_H
#define UTILS_H

#include <iostream>

// forward declarations
class ArArgumentParser;
class ArRobot;
class SensorData;

// used to pack x,y,z values together
struct A3dpoint {
  A3dpoint(double v1, double v2, double v3)
    : x(v1), y(v2), z(v3) {}
  double x;
  double y;
  double z;
};


void echo(const std::string &msg);
void echo(const std::string &id, const int value);
void echo(const std::string &id, const double value);
void echo(const std::string &id, const std::string &value);
const char *getConfigFile(ArArgumentParser &parser);
void configureRobot(ArArgumentParser &parser,
    		    int *tilt,
    		    A3dpoint &laserToRobotTranslation,
		    int *maxRange,
		    int *minRange);
void testLaser(ArRobot *robot);
void escapePressed(SensorData *sd);

// distinguish invalid values
const int INVALID = -123456489;


#endif
