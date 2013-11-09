#include <iomanip>
#include <list>
#include <fstream>

#include "Aria.h"
#include "ArNetworking.h"

#include "utils.h"
#include "SensorData.h"


// some message display routines
void echo(const std::string &msg)
{
  std::cout << "\t" << msg << std::endl;
}
void echo(const std::string &id, const int value)
{
  std::cout << "\t" << id << " = " << value << std::endl;
}
void echo(const std::string &id, const double value)
{
  std::cout << "\t" << id << " = " << value << std::endl;
}
void echo(const std::string &id, const std::string &value)
{
  std::cout << "\t" << id << " = " << value << std::endl;
}

// get the configuration file name
const char *getConfigFile(ArArgumentParser &parser)
{
  const char *fileName = NULL;

  // check for -file argument and name of file
  if (parser.checkParameterArgumentString("file", &fileName) &&
      fileName != NULL) {
    // check if file exists
    std::ifstream inf(fileName);
    if (inf) {
      inf.close();
      return fileName;
    }
    else {
      echo("Missing configuration file", fileName);
    }
  }
  echo("Not using custom configuration file");
  return NULL;
}

// get robot configuration information from configuration file
void configureRobot(ArArgumentParser &parser,
    		    int *tilt,
    		    A3dpoint &laserToRobotTranslation,
		    int *maxRange,
		    int *minRange)
{
  const int rightPad = 2;
  const char sepChar = '|';
  const char *configArgs[] = {
    "tilt",
    "laserToRobotTranslationX",
    "laserToRobotTranslationY",
    "laserToRobotTranslationZ",
    "maxRange",
    "minRange"
  };

  // get longest argument name
  int longestArgIndex = 0;
  size_t longestArgLength = strlen(configArgs[longestArgIndex]);
  for (size_t i = 1; i < sizeof(configArgs)/sizeof(configArgs[0]); i++) {
    if (strlen(configArgs[i]) > longestArgLength) {
      longestArgIndex = i;
      longestArgLength = strlen(configArgs[longestArgIndex]);
    }
  }

  int nameColLength = longestArgLength + 1;

  std::cout << "Custom configuration options" << std::endl;

  int val = INVALID;
  // tilt value
  if (parser.checkParameterArgumentInteger(configArgs[0], &val) &&
      val != INVALID) {
    *tilt = val;
    std::cout << std::setw(rightPad) << sepChar
      << std::setw(nameColLength) << "tilt" 
      << std::setw(rightPad) << sepChar << " "
      << val << " degrees" << std::endl;
  }

  // laser to robot translation values
  val = INVALID;
  if (parser.checkParameterArgumentInteger(configArgs[1], &val) &&
      val != INVALID) {
    laserToRobotTranslation.x = val;
    std::cout << std::setw(rightPad) << sepChar
      << std::setw(nameColLength) << "laserToRobotTranslationX" 
      << std::setw(rightPad) << sepChar << " "
      << val << " mm" << std::endl;
  }
  val = INVALID;
  if (parser.checkParameterArgumentInteger(configArgs[2], &val) &&
      val != INVALID) {
    laserToRobotTranslation.y = val;
    std::cout << std::setw(rightPad) << sepChar
      << std::setw(nameColLength) << "laserToRobotTranslationY" 
      << std::setw(rightPad) << sepChar << " "
      << val << " mm" << std::endl;
  }
  val = INVALID;
  if (parser.checkParameterArgumentInteger(configArgs[3], &val) &&
      val != INVALID) {
    laserToRobotTranslation.z = val;
    std::cout << std::setw(rightPad) << sepChar
      << std::setw(nameColLength) << "laserToRobotTranslationZ" 
      << std::setw(rightPad) << sepChar << " "
      << val << " mm" << std::endl;
  }

  // laser range values
  val = INVALID;
  if (parser.checkParameterArgumentInteger(configArgs[4], &val) &&
      val != INVALID) {
    *maxRange = val;
    std::cout << std::setw(rightPad) << sepChar
      << std::setw(nameColLength) << "maxRange" 
      << std::setw(rightPad) << sepChar << " "
      << val << " mm" << std::endl;
  }
  val = INVALID;
  if (parser.checkParameterArgumentInteger(configArgs[5], &val) &&
      val != INVALID) {
    *minRange = val;
    std::cout << std::setw(rightPad) << sepChar
      << std::setw(nameColLength) << "minRange" 
      << std::setw(rightPad) << sepChar << " "
      << val << " mm" << std::endl;
  }
}


// laser testing routine
void testLaser(ArRobot *robot)
{
  ArLaser *laser = robot->findLaser(1);

  const std::list<ArSensorReading *> * readings = NULL;
  std::list<ArSensorReading *>::const_iterator it;
  ArSensorReading *reading = NULL;

  const int WIDTH = 8;
  const std::string SEP = "||";

  std::cout << "\t LASER READINGS" << std::endl;
  std::cout << std::setw(WIDTH) << "localX" << SEP;
  std::cout << std::setw(WIDTH) << "localY" << SEP;
  std::cout << std::setw(WIDTH) << "range" << SEP;
  std::cout << std::setw(WIDTH) << "sensorTh" << SEP;
  std::cout << std::setw(WIDTH) << "robotTh" << SEP;
  std::cout << std::endl;

  int i = 0;
  readings = laser->getRawReadings();
  for (it = readings->begin(); it != readings->end(); it++) {
    ++i;
    if (i % 10 == 0) {
      reading = (*it);
      // check if the laser reading is valid
      if (!reading->getIgnoreThisReading()) {
	std::cout << std::setw(WIDTH) << reading->getLocalX() << SEP;
	std::cout << std::setw(WIDTH) << reading->getLocalY() << SEP;
	std::cout << std::setw(WIDTH) << reading->getRange() << SEP;
	std::cout <<std::setw(WIDTH)<< reading->getSensorTh() << SEP;
	std::cout <<std::setw(WIDTH)<< reading->getThTaken() << SEP;
	std::cout << std::endl;
      }
    }
  }
  std::cout << std::endl;
}

// Function to properly close the sensor. This is important for the stereo
// camera.
void escapePressed(SensorData *sd)
{
  delete sd;
  Aria::exit();
}
