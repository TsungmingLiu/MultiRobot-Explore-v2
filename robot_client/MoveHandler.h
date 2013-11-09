#ifndef MOVE_HANDLER_H
#define MOVE_HANDLER_H

#include "ArNetworking.h"
#include "RobotMap.h"

// Holds robot servers state information
struct Mode {
  Mode(bool w, bool s)
    : myWander(w), mySafe(s) { }
  bool myWander;
  bool mySafe;
};


// Abstract base class to provide an interface for
// moving client robots
class MoveHandler {
public:
  virtual void update() = 0;
  virtual void displayKeys() = 0;
  void setClientIndex(size_t i);
  virtual void sendData() = 0;

  std::vector<Mode> *robotModes;
  
protected:
  MoveHandler(std::vector<ArClientBase *> &clients);
  virtual void ratioDrive();
  virtual void wander();
  virtual void stop();
  virtual void safeDrive();
  virtual void nextRobot();
  virtual void prevRobot();
  virtual void wanderAll();
  virtual void stopAll();

  static const char *actions[];

  std::vector<ArClientBase *> &myClients;
  unsigned short myClientIndex;
  ArClientBase *myClient;
  double myTransRatio;
  double myRotRatio;
  double mySpeedLimit;
};

// Move using keyboard
class MoveKeyHandler : public MoveHandler {
public:
  MoveKeyHandler(std::vector<ArClientBase *> &clients,
      ArKeyHandler *keyHandler);
  virtual void update();
  virtual void displayKeys();
  virtual void sendData() { }

private:
  virtual void nextRobot();
  virtual void prevRobot();
  void forward();
  void backward();
  void turnLeft();
  void turnRight();

  static const int keys[];

  ArKeyHandler *myKeyHandler;
  ArFunctorC<MoveKeyHandler> myForwardFtr;
  ArFunctorC<MoveKeyHandler> myBackwardFtr;
  ArFunctorC<MoveKeyHandler> myTurnLeftFtr;
  ArFunctorC<MoveKeyHandler> myTurnRightFtr;
  ArFunctorC<MoveKeyHandler> myWanderFtr;
  ArFunctorC<MoveKeyHandler> myStopFtr;
  ArFunctorC<MoveKeyHandler> myUnsafeFtr;
  ArFunctorC<MoveKeyHandler> myNextRobotFtr;
  ArFunctorC<MoveKeyHandler> myPrevRobotFtr;
  ArFunctorC<MoveKeyHandler> myWanderAllFtr;
  ArFunctorC<MoveKeyHandler> myStopAllFtr;
};

// Move using Joystick
class MoveJoyHandler : public MoveHandler {
public:
  MoveJoyHandler(std::vector<ArClientBase *> &clients,
      ArJoyHandler *joyHandler);
  virtual void update();
  virtual void displayKeys();
  virtual void sendData() { }

private:
  virtual void ratioDrive();
  virtual void nextRobot();
  virtual void prevRobot();

  static const char *keys[];

  ArJoyHandler *myJoyHandler;
};

// useful movement related function declarations
void defaultMoveKeys(std::vector<int> &moveKeys, 
    std::vector<std::string> &moveKeysInfo);
std::string moveKeyToString(int c);
void initControllers(MoveHandler *&key, MoveHandler *&joy, MoveHandler *&coor, int nClients);


/***********************************************************************
 * Potential Map Robot Handler
 **********************************************************************/
// Move using Coordinate
class MoveMapHandler : public MoveHandler {
public:

  MoveMapHandler(std::vector<ArClientBase *> &clients, RobotMap * newMap);
  virtual void update();  // move all the robots
  //virtual void displayCoors();

  virtual void displayKeys(){}
  void setClientIndex(size_t i){}
  
  void moveAllRobots();
  double getNewHeading(int id);
  double getNewDistance(int id);

  void cacheMap(ArNetPacket *packet, int robotID);
  bool isForwardSuccess(int robotId, int i);
  void sendPacket(ArNetPacket *packet, int fromRobot, int toRobot);

  void moveRobot(int id, double distance, double heading);
  void circle();
  virtual void sendData();

  void makeThemMove();
  
private:
  RobotMap * map;
  //virtual void safeDrive();
  //virtual void nextRobot();
  /*
  virtual void nextRobot();
  virtual void prevRobot();
  void forward();
  void backward();
  void turnLeft();
  void turnRight();
  
  static const int Coors[];
  */
};


#endif
