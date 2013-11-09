#ifndef HELPERS_H
#define HELPERS_H

#include <iostream>

#include "ArNetworking.h"

// forward declaratiosn
struct HostInfo;
class SensorDataHandler;
class SensorDataViewer;


// some message display routines
void echo(const std::string &msg);
void echo(const std::string &id, const int value);
void echo(const std::string &id, const std::string &value);
void errorExit(std::string msg);
int rgba(int r, int g, int b);
void escapePressed();
const char *createRobotName(const char *IP);
void connectHosts(std::vector<ArClientBase *> &clients,
                  const std::vector<HostInfo> &hostsInfo);
void startClients(std::vector<ArClientBase *> clients);
std::string genTimeStr();
bool genDir(const std::string &dirName);
long getElapsedTime();
void createKeyHandlers(
    ArKeyHandler &keyHandler,
    std::vector<SensorDataHandler *> &sensorDataHandlers,
    SensorDataViewer *&viewer);
void printTitle(const std::string &title);

#endif
