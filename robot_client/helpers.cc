#include <sstream>

#include "Aria.h"

#include "helpers.h"
#include "ConfigFileReader.h"
#include "SensorDataViewer.h"
#include "SensorDataHandler.h"


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


void errorExit(std::string msg)
{
  echo(msg);
  Aria::shutdown();
  std::exit(1);
}

// Pack the color information into a single integer which is needed for
// PCL point types.
int rgba(int r, int g, int b) {
  return b + 256*g +256*256*r;
}

// shuts down aria 
void escapePressed()
{ 
  Aria::shutdown(); 
}


// used to create a name of form botxxx where xxx is the last 3 digits
// of the IP address
const char *createRobotName(const HostInfo &hostInfo)
{
  char nameBuffer[100];
  sprintf(nameBuffer, "robot_%s_%d", hostInfo.ip, hostInfo.port);
  return strdup(nameBuffer);
}

// Connects to each IP address in hostsIP.
// The client objects are stored in clients.
void connectHosts(std::vector<ArClientBase *> &clients,
                  const std::vector<HostInfo> &hostsInfo)
{
  ArClientBase *client = NULL;

  for (unsigned int i = 0; i < hostsInfo.size(); i++) {
    client = new ArClientBase;
    client->setRobotName(createRobotName(hostsInfo[i]));

    if (client->blockingConnect(hostsInfo[i].ip, hostsInfo[i].port)) {
      clients.push_back(client);
    }
  }

  // need at least one robot connection
  assert(clients.size());
}

// just start all the clients
void startClients(std::vector<ArClientBase *> clients)
{
  for (unsigned int i = 0; i < clients.size(); i++) {
    clients[i]->runAsync();
    ArNetPacket packet;
    packet.byteToBuf(0);
    clients[i]->requestOnce("setSafeDrive", &packet);
  }
}

// Creats a string representing the current time in a readable format.
// Format: month - day _ hour : min : sec
std::string genTimeStr()
{
  const char SEPARATOR = '_';
  const char DATE_SEPARATOR = '-';
  const char TIME_SEPARATOR = ':';
  time_t seconds = time(NULL);
  struct tm *timeInfo = localtime(&seconds);

  std::ostringstream new_name;
  new_name << timeInfo->tm_mon + 1 << DATE_SEPARATOR
           << timeInfo->tm_mday << SEPARATOR
	   << timeInfo->tm_hour << TIME_SEPARATOR
	   << timeInfo->tm_min << TIME_SEPARATOR
	   << timeInfo->tm_sec;

  return new_name.str();
}

// Creates directory based on the directory name.
// Returns true if successful and false if failure.
bool genDir(const std::string &dirName)
{
  // error occurred while creating a new directory
  if (mkdir(dirName.c_str(), S_IRWXU | S_IRWXG) == -1) {
    std::cout << "Error creating " << dirName << ": " << std::endl;
    std::cout << strerror(errno) << std::endl;
    return false;
  }
  return true;
}

// The first time this function is run, a starting time is set.
// Further calls return the time elapsed from that start time.
// This is necessary to get smaller values for time so that
// millisecond precision can be packet into the same data.
// Hence first packet is marked with time value of 0.
long getElapsedTime()
{
  static timeval startTime;
  timeval currTime;
  static bool firstTime = true;

  // set the start time
  if (firstTime) {
    firstTime = false;
    gettimeofday(&startTime, NULL);
    return 0;
  }
  else {
    gettimeofday(&currTime, NULL);
    long secondsPassed = currTime.tv_sec - startTime.tv_sec;
    // first get milliseconds
    long milliSecondsPassed = currTime.tv_usec/1000;
    // add the seconds passed to it
    milliSecondsPassed += secondsPassed*1000;
    return milliSecondsPassed;
  }
}

// create key press handlers
void createKeyHandlers(
    ArKeyHandler &keyHandler,
    std::vector<SensorDataHandler *> &sensorDataHandlers,
    SensorDataViewer *&viewer)
{
  printTitle("Supported keyboard commands");
  const int colWidthDesc = 15;
  const int colWidthKey = 10;
  std::cout << std::setw(colWidthDesc) << "Action"
    << std::setw(colWidthKey) << "Key" << std::endl
    << std::string(colWidthDesc + colWidthKey, '-') << std::endl;

  // keypress to exit program
  ArFunctor *escapeFtr = new ArGlobalFunctor(escapePressed);
  keyHandler.addKeyHandler(ArKeyHandler::ESCAPE, escapeFtr);
  std::cout << std::setw(colWidthDesc) << "EXIT PROGRAM"
    << std::setw(colWidthKey) << "ESC" << std::endl;

  // keypress to start viewer
  ArFunctor *startViewerFtr = 
    new ArGlobalFunctor2< SensorDataViewer *&,
	std::vector<SensorDataHandler *>& >
    (createViewer, viewer, sensorDataHandlers);
  keyHandler.addKeyHandler('d', startViewerFtr);
  std::cout << std::setw(colWidthDesc) << "DATA TRANSER"
    << std::setw(colWidthKey) << "D" << std::endl;

  // keypress to write cloud files
  ArFunctor *writeToFileFtr = 
    new ArGlobalFunctor1< std::vector<SensorDataHandler *>& >
    (writeSensorDataToDisk, sensorDataHandlers);
  keyHandler.addKeyHandler('c', writeToFileFtr);
  std::cout << std::setw(colWidthDesc) << "CREATE FILES"
    << std::setw(colWidthKey) << "C" << std::endl;
}

// print the string in a box to indicate title
void printTitle(const std::string &title)
{
  const char borderSymbol = '*';
  const std::string horizontalBorder(title.size() + 4, borderSymbol);

  std::cout << std::endl << horizontalBorder << std::endl
    << borderSymbol << ' ' << title << ' ' << borderSymbol << std::endl
    << horizontalBorder << std::endl;
}
