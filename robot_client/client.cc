/* This client program is supposed to provide a means of controlling
 * a robot which has the accompanying server program running on it.
 * The client will allow you to stop or move the robot and even let the
 * robot move automatically avoiding obstacles. Furthermore a point cloud
 * of the robot's reading is displayed in a separate PCL window.
 *
 * Provide a textfile with an IP address on each line corresponding to
 * a robot server. The accompanying server program must be running on
 * those hosts. This program will create clients to connect to each server.
 * The first line on the text file must be:
 * servers 1.0
 *
 * Default controls for the first two clients.
 *                   client     clone
 *
 * (Let the robot wander automatically avoiding obstacles)
 * auto mode          'a'        'q'
 *
 * (Bring the robot to a stop)
 * stop mode          's'        'w'
 *
 * (Move the robot manually in drive mode)
 * drive mode         'd'        'r'
 * go up           up arrow      'i'
 * go down         down arrow    'k'
 * go left         left arrow    'j'
 * go right        right arrow   'l'
 *
 * Press 'p' to create a pcd file of the current point cloud
 */

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <cmath>
using namespace std;

#include "Aria.h"

#include "helpers.h"
#include "ConfigFileReader.h"
#include "MoveHandler.h"
#include "SensorDataHandler.h"
#include "SensorDataViewer.h"
#include "RobotMap.h"


// main main
int main(int argc, char **argv)
{
  // list of information about each host
  std::vector<HostInfo> hostsInfo;
  // list of clients to connect to each server
  std::vector<ArClientBase *> clients;
  // list of sensor data handlers for clients
  std::vector<SensorDataHandler *> sensorDataHandlers;

  // needed to initialize aria framework
  Aria::init();
  // parser for command line arguments
  ArArgumentParser parser(&argc, argv);
  parser.loadDefaultArguments();

  if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed()) {
    Aria::shutdown();
    exit(1);
  }

  // This class reads the file and populates the necessary lists
  ConfigFileReader configFileReader(argc, argv, &parser);
  // fill the vectors with information from file
  configFileReader.readHostsFile(hostsInfo);

  // create a connection to each server
  connectHosts(clients, hostsInfo);

  // create the keyhandler which allows manipulating the robots
  ArKeyHandler keyHandler;
  Aria::setKeyHandler(&keyHandler);
  // keyboard movement controls
  MoveHandler *moveKeyHandler = new MoveKeyHandler(clients, &keyHandler);

  // joystick support for client one
  ArJoyHandler joyHandler;
  MoveHandler *moveJoyHandler = NULL;
  if (!joyHandler.init()) echo("NO JOYSTICK!!!");
  else {
    joyHandler.setSpeeds(50, 100);
    moveJoyHandler = new MoveJoyHandler(clients, &joyHandler);
  }
  
  // Creates an empty map and setup map-coordinate robot control
  RobotMap * map = new RobotMap();///*"init_map.txt", clients*/); // just an empty map
  MoveHandler *moveMapHandler = new MoveMapHandler(clients, map); // hookup all the move hander with map
  
  // initialize the controllers/clients
  initControllers(moveKeyHandler, moveJoyHandler, moveMapHandler, clients.size());

  // start all the clients
  startClients(clients);
  // create the sensor data handlers for the clients
  createSensorDataHandlers(clients, sensorDataHandlers, hostsInfo, map);
  SensorDataViewer *viewer = NULL;

  // create key press handlers
  createKeyHandlers(keyHandler, sensorDataHandlers, viewer);

  // breathing time for inital setup procedures
  ArUtil::sleep(1000);

  for (size_t i = 0; i < sensorDataHandlers.size(); i++){
    sensorDataHandlers[i]->request();
    //((MoveMapHandler*) &moveMapHandler)->makeThemMove();
  }
  // check for key presses, button presses and new data
  while (clients[0]->getRunningWithLock() && map->getGlobalCoverage()<=0.99) {  
    keyHandler.checkKeys();
    //if (moveKeyHandler) moveKeyHandler->update();
    //if (moveJoyHandler) moveJoyHandler->update();
    //std::cout << "Moving\n";
    //if (moveMapHandler) moveMapHandler->update(); // old send move command
    if (moveMapHandler) moveMapHandler->sendData();   // new send map data
    //if (viewer) viewer->updateDisplay();
    ArUtil::sleep(1000);
  }
  Aria::shutdown();
  map->plotAll();
  std::cout <<"Program terminated normally.\n";
}
