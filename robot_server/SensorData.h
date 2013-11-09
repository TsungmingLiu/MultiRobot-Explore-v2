#ifndef PCLDATA_H
#define PCLDATA_H

#include "Aria.h"
#include "ArNetworking.h"
//#include <cv.h>
#include "RobotMap.h"
#include "utils.h"


// forward declarations
class stereoCam;


// Abstract base class for robot sensor data
class SensorData {
public:
  SensorData(ArServerBase *server, ArRobot *robot);
  virtual ~SensorData() {}
  A3dpoint transformPoint(const ArPose &fromFrame, const A3dpoint &point);

  static const double pi;
  static const double toRadian;
protected:
  ArServerBase *myServer;
  ArRobot *myRobot;

  virtual void send(ArServerClient *serverClient, ArNetPacket *packet) = 0;
  virtual void addData() = 0;
  void robotToBuf(ArNetPacket *packet);
};


// Sends map data
class SensorDataMap : public SensorData {
public:
  SensorDataMap(ArServerBase *server, ArRobot *robot,
      		  int tilt, const A3dpoint &MapToRobotTranslation,
		  int maxRange, int minRange);
  void hookupMap(RobotMap *map);
  virtual void send(ArServerClient *serverClient, ArNetPacket *packet);
  virtual void receive(ArServerClient *serverClient, ArNetPacket *packet);
  virtual void initiate(ArServerClient *serverClient, ArNetPacket *packet);
  virtual void addData();
  RobotMap *myMap;

private:
  ArFunctor2C<SensorDataMap, ArServerClient *, ArNetPacket *> mySendFtr;
  ArFunctor2C<SensorDataMap, ArServerClient *, ArNetPacket *> myReceiveFtr;
  ArFunctor2C<SensorDataMap, ArServerClient *, ArNetPacket *> myInitiateFtr;
  ArLaser *myLaser;
  int myTilt;
  A3dpoint myLaserToRobotTranslation;
  int myMaxRange;
  int myMinRange;
};

// Sends laser data
class SensorDataLaser : public SensorData {
public:
  SensorDataLaser(ArServerBase *server, ArRobot *robot,
      		  int tilt, const A3dpoint &laserToRobotTranslation,
		  int maxRange, int minRange);
  virtual void send(ArServerClient *serverClient, ArNetPacket *packet);
  virtual void addData();

private:
  ArFunctor2C<SensorDataLaser, ArServerClient *, ArNetPacket *> mySendFtr;
  ArLaser *myLaser;
  int myTilt;
  A3dpoint myLaserToRobotTranslation;
  int myMaxRange;
  int myMinRange;
};


#ifdef STEREO_CAMERA

// Sends stereo camera data
class SensorDataStereoCam : public SensorData {
public:
  SensorDataStereoCam(ArServerBase *server, ArRobot *robot);
  virtual ~SensorDataStereoCam();
  virtual void send(ArServerClient *serverClient, ArNetPacket *packet);
  void send2(ArServerClient *serverClient, ArNetPacket *packet);
  virtual void addData();

  // compress coordinates to this type
  typedef short COORDINATE_TYPE;

private:
  struct PointInfo {
    PointInfo(double X, double Y, double Z,
	char R, char G, char B)
      : x(X), y(Y), z(Z), r(R), g(G), b(B) { }
    double x;
    double y;
    double z;
    char r;
    char g;
    char b;
  };

  bool invalidPoint(double x, double y, double z);
  void getImage(IplImage **coordImg, IplImage **colorImg);
  int getValidPoints(std::vector<PointInfo> &points,
    int rowStart, int rowEnd, int maxPoints);

  ArFunctor2C<SensorDataStereoCam, ArServerClient *, ArNetPacket *> 
    mySendFtr;
  ArFunctor2C<SensorDataStereoCam, ArServerClient *, ArNetPacket *> 
    mySendFtr2;
  ArDPPTU *myPTU;
  stereoCam *myCam;
  const short myCaptureWidth;  // in pixels
  const short myCaptureHeight; // in pixels
  const short myPointSize;  // bytes needed for one point
  int myRowIncrement;
  int myColIncrement;
};

#endif

#endif
