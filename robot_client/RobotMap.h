#ifndef RobotMap_H
#define RobotMap_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include <algorithm>
#include <time.h>  
#include <cmath>
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
//#include "Robot.h"
//#include <Eigen/Core>	// Eigen library
using namespace std;
const int gridSize = 25;
const int MaxPotential = 256;

typedef pcl::PointXYZRGB MyPoint;
typedef pcl::PointCloud<MyPoint> MyCloud;

struct Vector2D{
	double x,y;
};
struct Vector2I{
	int x,y;
};

class Robot
{
  public:
  	Robot();
    //Robot(double x, double y, double heading);
    //Robot(string initializeFile);
  	
    void setPosition(double x, double y);
    void setHeading(double heading=1.5708);
    //void newPosition(double x, double y, double heading=0.25);
    Vector2D getPosition();
    double getPositionX(){return m_position.x;};
    double getPositionY(){return m_position.y;};
    double getHeading(){return m_heading;};
 
  private:
    Vector2D m_position;      // Robot position
    double m_heading;         // Robot heading in gradient
    double m_sensorRange, m_sensorAngle;
                              // Robot sensor max range and angle
};

struct RobotMapSetting
{
    int repulseRange;     // Sensitivity of repulsion
    int attractRange;  // Sensitivity of attractive
    double velocity;      // Robot max velocity
    double mapDimension, gridDimension;
    //string realRobotMap;       // File name of real RobotMap
    //string initialState;  // File name of robot initialization
};

class RobotMap
{
  public:
    // Constructors
    RobotMap();
    void createRobots();
    //RobotMap(string initialState, string RobotMap);
    
    // Grid/Map operation functions
    void updateOccupancyGrid(int x, int y, int potential);
    void updateObstacleGrid(int x, int y, int potential);
    int getOccupancyGrid(int x, int y);
    int getObstacleGrid(int x, int y);
    int getRealMap(int x, int y);
    void generateNoise(int n);
    void updateNoiseGrid(int x, int y, int potential);
    void updateField(int x, int y, double xVector, double yVector);
    //void moveRobot(int robotIndex, double heading, double velocity);
    //int getMoveMode();
    
    // I/O functions
    //void readFile_InitialState(string fileName);	// Read file to occupancyGrid
    //void readFile_RobotMap(string fileName);						// Read file to obstacleRobotMap
    void printRobotMap();			// output RobotMap onto screen
    void updateRobotMap(int id, MyCloud cloud);
    void updateRobotInfo(int id, double x, double y, double th);
    double getRobotSpeed(int id);
    double getRobotHeading(int id);
    void outputRobotMap();			// output the RobotMap to a file
    void outputVectorField(int RobotMapNum);
    void outputCoverageHistory();
    void outputCoverageEfficiency();
    void initializeGrids();
    //void outputAvgPotential();
    void generateReport();
    void plotAll();				// plot RobotMap from above output; only need to call once at the end of program
    
    // Evaluation functions
    double getGlobalCoverage();
    double getLocalCoverage(int x, int y);
    void calculateCellPotential(int x, int y);
    void calculateMapPotential();
    void updateSensedMap();
    void moveRobots();
    bool isValidCell(int x, int y);
    
  private:
    RobotMapSetting m_setting;                     // RobotMap setting & initialization
    vector<Robot> m_robotList;                // List of robots
    
    int occupancyGrid     [gridSize][gridSize];	// RobotMap matrix, contains visited potential level
    int obstacleGrid	  [gridSize][gridSize];	// RobotMap matrix, contains sensed obstacle potential level
    int noiseGrid		  [gridSize][gridSize];	// contains random generated noise
    //double coverageGrid [gridSize][gridSize];
    Vector2D vectorField  [gridSize][gridSize];	// RobotMap potential potential, contains potential vector
    
    //int realMap         [gridSize][gridSize];   // RobotMap matrix, real map
    //double availableSpace;
    int updateCounter;
    vector<double> coverageHistory;
    //vector<double> averagePotentialHistory;
};


/*
class TSS
{
  public:
    TSS();
    void initialize();
    void solve();
  private:
};
*/
#endif
