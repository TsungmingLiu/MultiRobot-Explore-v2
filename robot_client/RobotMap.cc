//#include "Robot.h"
#include "RobotMap.h"
//#include "Utils.h"


/********************Utility Functions*********************************
 *********************************************************************/
int sgn(double d){
  return d<0? -1:d>0;
}

string fileNumber(int num){
  stringstream ss;  
  if(num<10){
    ss << "00";
  }
  else if(num<100){
    ss << "0";
  }
  ss << num;
  return ss.str();
}

/******************* Class Functions ***********************************
 * 1. Default constructor & Constructor with file initialization
 * 2. File initializer - Initial state and map
 **************** Constructors & Initialization ***********************/
Robot::Robot(){
  setPosition(0,0);
  setHeading(0);
}

/******************** I/O funcitons ***********************************/
void Robot::setPosition(double x, double y){
  m_position.x = x;
  m_position.y = y;
}

void Robot::setHeading(double heading){
  m_heading = heading;
}

Vector2D Robot::getPosition(){
  Vector2D position;
  position.x = m_position.x;
  position.y = m_position.y;
  return position;
}

/********************Class Functions************************************
 * 1. Default constructor & Constructor with file initialization
 * 2. File initializer - Initial state and RobotMap
 ****************Constructors & Initialization*************************/
RobotMap::RobotMap(){
  updateCounter = 0;
  m_setting.repulseRange = 20;
  m_setting.attractRange = 20;
  m_setting.velocity = 1;
  m_setting.mapDimension = 5000;
  m_setting.gridDimension = m_setting.mapDimension / gridSize;
  coverageHistory.push_back(0);
  initializeGrids();
  srand (time(NULL));
}
/*
RobotMap::RobotMap(string initialState, string RobotMapFile){
  // initializing RobotMap
  updateCounter = 0;
  m_setting.repulseRange = 20;
  m_setting.attractRange = 20;
  m_setting.velocity = 1;
  initializeGrids();
  srand (time(NULL));
  //readFile_InitialState(initialState);
  //readFile_RobotMap(RobotMapFile);
}
*/
void RobotMap::initializeGrids(){
  for(int i=0;i<gridSize;i++){
    for(int j=0;j<gridSize;j++){
      occupancyGrid[i][j]= 0;
      obstacleGrid[i][j]= 0;
      noiseGrid[i][j]= 0;
      vectorField[i][j].x=vectorField[i][j].y=0;
    }
  }
}
void RobotMap::createRobots(){
  Robot newRobot;
  m_robotList.push_back(newRobot);
}
/*
void RobotMap::readFile_InitialState(string fileName){
  ifstream myReadFile;
  string tempLine;
  vector <int> tempPotential;
  myReadFile.open(fileName.c_str());
  while(!myReadFile.eof()) // To get you all the lines.
  {
    getline(myReadFile,tempLine); // Saves the line in STRING.
    stringstream ss(tempLine);
    while(!ss.eof()){
      string tempStr;
      ss >> tempStr;
      int tempInt = atoi(tempStr.c_str());
      tempPotential.push_back(tempInt);
    }
  }
  myReadFile.close();
  
  for(int j=gridSize-1;j>=0;j--){
    for(int i=0;i<gridSize;i++){
      //occupancyGrid[i][j]= tempPotential.front();  // initialize sensing grid
      if(tempPotential.front()==2){    // initialize robot position
        //Vector2I tempRobotPosition;
        //tempRobotPosition.x = i;
        //tempRobotPosition.y = j;
        //robotPosition.push_back(tempRobotPosition);
        Robot newRobot;
        newRobot.setPosition(i+0.1,j+0.1);
        m_robotList.push_back(newRobot);
      }
      tempPotential.erase(tempPotential.begin());
      //vectorField[i][j].x=vectorField[i][j].y=0;
    }
  }
}

void RobotMap::readFile_RobotMap(string fileName){
  ifstream myReadFile;
  string tempLine;
  vector <int> tempPotential;
  myReadFile.open(fileName.c_str());
  while(!myReadFile.eof()){ // To get you all the lines.
    getline(myReadFile,tempLine); // Saves the line in STRING.
    stringstream ss(tempLine);
    while(!ss.eof()){
      string tempStr;
      ss >> tempStr;
      int tempInt = atoi(tempStr.c_str());
      tempPotential.push_back(tempInt);
    }
  }
  myReadFile.close();
  
  int obstacleCount;
  for(int j=gridSize-1;j>=0;j--){
    for(int i=0;i<gridSize;i++){
      realMap[i][j]= tempPotential.front();
      tempPotential.erase(tempPotential.begin());
    }
  }
}
*/

void RobotMap::printRobotMap(){
  system("clear");
  cout << "In Step #" << updateCounter << ",\n";
  for(unsigned int i=0; i<m_robotList.size();i++){
    cout << " -->Robot " << i << " located at: " 
      << m_robotList[i].getPositionX() << ", " << m_robotList[i].getPositionY() << endl;
  }
  cout << endl;
  /*  cout << "\n*****OUTPUTTING NOISE RobotMap #" << updateCounter << "*****\n";
  for(int j=gridSize-1;j>=0;j--){
    for(int i=0;i<gridSize;i++){
      cout << "\t" << noiseGrid[i][j];
    }
    cout << endl;
  }*/
  cout << "\n*****OUTPUTTING SENSOR RobotMap #" << updateCounter << "*****\n";
  for(int j=gridSize-1;j>=0;j--){
    for(int i=0;i<gridSize;i++){
      cout << getOccupancyGrid(i,j) << "\t" ;
    }
    cout << endl;
  }
  cout << "\n*****OUTPUTTING OBSTACLE RobotMap #" << updateCounter << "*****\n";
  for(int j=gridSize-1;j>=0;j--){
    for(int i=0;i<gridSize;i++){
      cout << getObstacleGrid(i,j)<< "\t" ;
    }
    cout << endl;
  } 
  
}

/********************Class Functions************************************
 * 1. RobotMap updater - sensing grid, noise grid & vector field
 * 2. RobotMap evaluator - evaluate potential vector for cell & RobotMap
***************************RobotMap update*****************************/
void RobotMap::updateOccupancyGrid(int x, int y, int potential){
// update the specific occupancyGrid cell with given value
  if((x<gridSize&&x>=0)&&(y>=0&&y<gridSize)){   
    if(potential<=MaxPotential && occupancyGrid[x][y] < potential){
      occupancyGrid[x][y] = potential;
    }
  }
}

int RobotMap::getOccupancyGrid(int x, int y){
  if((x<gridSize&&x>=0)&&(y>=0&&y<gridSize))
    return occupancyGrid[x][y];
  else
    return 0;
}
int RobotMap::getObstacleGrid(int x, int y){
  if((x<gridSize&&x>=0)&&(y>=0&&y<gridSize))
    return obstacleGrid[x][y];
  else
    return 0;
}
/*
int RobotMap::getRealMap(int x, int y){
  if((x<gridSize&&x>=0)&&(y>=0&&y<gridSize))
    return realMap[x][y];
  else
    return 0;
}*/
void RobotMap::updateObstacleGrid(int x, int y, int potential){
// update the specific obstacleGrid cell with given value
  if((x<gridSize&&x>=0)&&(y>=0&&y<gridSize)){
    if(potential<=MaxPotential && obstacleGrid[x][y] < potential){
      obstacleGrid[x][y] = potential;
    }
  }
}

void RobotMap::generateNoise(int n){
  for(int i=0;i<gridSize;i++){
    for(int j=0;j<gridSize;j++){
      noiseGrid[i][j]= 0;
    }
  }
  int x, y;
  for(int i=0;i<n;i++){
     x = rand() % gridSize;// + 1;
     y = rand() % gridSize;// + 1;
     updateNoiseGrid(x, y, 1);
  }
}

void RobotMap::updateNoiseGrid(int x, int y, int potential){
// update the specific noiseGrid cell with given value
  if(x<=gridSize&&y<=gridSize){
    noiseGrid[x][y] = potential;
  }
}

void RobotMap::updateField(int x, int y, double xVector, double yVector){
  if(x<=gridSize&&y<=gridSize){
    vectorField[x][y].x = xVector;
    vectorField[x][y].y = yVector;
  }
}

/********************Class Functions************************************
 * 1. RobotMap updater - sensing grid, noise grid & vector field
 * 2. RobotMap evaluator - evaluate potential vector for cell & RobotMap
***************************RobotMap evaluate*****************************/
void RobotMap::updateRobotMap(int id, MyCloud cloud){
  Vector2D position = m_robotList[id].getPosition();
  vector<Vector2D> otherRobotPosition;
  for(unsigned int i=0;i<m_robotList.size();i++){
    if(i!=id){
      Vector2D otherRobot = m_robotList[i].getPosition();
      otherRobotPosition.push_back(otherRobot);
    }
  }
  int sensorStep = 50;
  vector <Vector2I> scanned;
  for(unsigned int i=0;i<cloud.size();i++){
    Vector2D scannedPoint;
    scannedPoint.x = cloud[i].x;
    scannedPoint.y = cloud[i].y;
    double x = position.x, y = position.y;
    double diffX = (scannedPoint.x - position.x)/sensorStep;
    double diffY = (scannedPoint.y - position.y)/sensorStep;
    for(int k = 0; k<sensorStep;k++){
      x += diffX;
      y += diffY;
      int tempX = x/m_setting.gridDimension, tempY = y/m_setting.gridDimension;
      Vector2I newScan;
      newScan.x = tempX;
      newScan.y = tempY;
      //cout << "RobotLocation: " << position.x << ", " << position.y << endl;
      //cout << "Sensor value:  " << scannedPoint.x << ", " << scannedPoint.y << endl;
      //cout << "Diff x/y:      " << diffX << "/" << diffY << endl;
      //cout << "TempStep value:" << x << ", " << y << endl;
      //cout << "TempStep Grid: " << newScan.x << ", " << newScan.y << endl;
      if(scanned.size()==0){
        scanned.push_back(newScan);
      }
      else{
        bool matchFlag = false;
        for(unsigned int z = 0;z<scanned.size();z++){
          if(scanned[z].x==newScan.x && scanned[z].y==newScan.y){
            matchFlag = true;
          }
        }
        if(!matchFlag){
          scanned.push_back(newScan);
        }
      }
    }
    int tempScannedPointX = scannedPoint.x/m_setting.gridDimension, tempScannedPointY = scannedPoint.y/m_setting.gridDimension;
    if(sqrt(pow(scannedPoint.x,2)+pow(scannedPoint.y,2)) <= 10000){
      // if < max sensing range, then it is an obstacle, else not.
      bool isOtherRobot = false;
      for(size_t j=0;j<otherRobotPosition.size();j++){
        int otherRobotX = otherRobotPosition[j].x / m_setting.gridDimension;
        int otherRobotY = otherRobotPosition[j].y / m_setting.gridDimension;
        if(abs(tempScannedPointX - otherRobotX)<=2 && abs(tempScannedPointY - otherRobotY)<=2){
          isOtherRobot = true;
        }
      }
      if(!isOtherRobot)updateObstacleGrid(tempScannedPointX, tempScannedPointY, getObstacleGrid(tempScannedPointX, tempScannedPointY)+1);
    }
  }
  for(unsigned int z=0;z<scanned.size();z++){
    int tempX = scanned[z].x, tempY = scanned[z].y;
    updateOccupancyGrid(tempX, tempY, getOccupancyGrid(tempX, tempY)+1);
  }
}

void RobotMap::updateRobotInfo(int id, double newX, double newY, double newHeading){
  m_robotList[id].setPosition(newX, newY);
  m_robotList[id].setHeading(newHeading);
  
  Vector2I position;
  position.x = (int) newX / m_setting.gridDimension;
  position.y = (int) newY / m_setting.gridDimension;
  cout << "Getting new position for robot #" << id << ", at(" << newX << "," << newY << ")\n";
  cout << "Reposition to RobotMap at(" << position.x << "," << position.y << ")\n";
  updateOccupancyGrid(position.x, position.y, getOccupancyGrid(position.x, position.y)+15);
  updateOccupancyGrid(position.x+1, position.y, getOccupancyGrid(position.x+1, position.y)+10);
  updateOccupancyGrid(position.x-1, position.y, getOccupancyGrid(position.x-1, position.y)+10);
  updateOccupancyGrid(position.x, position.y+1, getOccupancyGrid(position.x, position.y+1)+10);
  updateOccupancyGrid(position.x, position.y-1, getOccupancyGrid(position.x, position.y-1)+10);
}

double RobotMap::getGlobalCoverage(){
  double visitedSpaces = 0;
  //int obstacleCount = 0;
  for(int i=0;i<gridSize;i++){
    for(int j=0;j<gridSize;j++){
      if(occupancyGrid[i][j]!=0){
        visitedSpaces = visitedSpaces+1;
      }
      //if(obstacleRobotMap[i][j]!=0){
			//	obstacleCount++;
			//}
    }
  }
  double availableSpace = (gridSize * gridSize);// - obstacleCount;
  double globalCoverage = visitedSpaces/availableSpace;
  coverageHistory.push_back(globalCoverage);
  return globalCoverage;
}
bool RobotMap::isValidCell(int x, int y){
  if(x>=gridSize||x<0||y>=gridSize||y<0)
    return false;
  else
    return true;
}
double RobotMap::getLocalCoverage(int x, int y){
  int availableSpaces = 0, visitedSpaces = 0;
  int LocalArea = 5;
  int localX = x-2, localY = y-2;
  //int obstacleCount = 0;
  for(int i=0;i<LocalArea;i++){
    for(int j=0;j<LocalArea;j++){
      if(isValidCell(localX+i,localY+j)){
        availableSpaces += 1;
        if(getOccupancyGrid(localX+i,localY+j)>0){
          visitedSpaces = visitedSpaces+1;
        }
      }
    }
  }
  double localCoverage = visitedSpaces/availableSpaces;
  //coverageHistory.push_back(currentCoverage);
  return localCoverage;
}
/*
void RobotMap::getAveragePotential(){
  double potentialSum = 0;
  for(int i=0;i<gridSize;i++){
    for(int j=0;j<gridSize;j++){
        potentialSum = occupancyGrid[i][j];
    }
  }
  double potentialAvg = potentialSum/(gridSize*gridSize);
  averagePotential.push_back(potentialAvg);
}
*/

void RobotMap::calculateCellPotential(int xCoor, int yCoor)
{
// Goes through each block of the RobotMap
// and get the potential vector of THIS SPECIFIC CELL
  double xVect=0, yVect=0;  // sum of potential vector
  //coverageHistory.push_back(getGlobalCoverage());
  // Evaluate robot trail grid
  // Receive repulsive from previous position
	generateNoise(50);
  //int currentPotential = getOccupancyGrid(xCoor,yCoor);
  int currentPotential = occupancyGrid[xCoor][yCoor];
  // Repulsive potential part
  for(int i=0;i<gridSize;i++){
    for(int j=0;j<gridSize;j++){
      double dx = xCoor-i;
      double dy = yCoor-j;
      double absDist = sqrt(dx*dx+dy*dy);  // tho(q)
      double diff = (occupancyGrid[i][j]-currentPotential);
      
      if (getObstacleGrid(i,j)>0){
        if(absDist<=m_setting.repulseRange){
          if(xCoor!=i && yCoor!=j){
            //calculate perpentidular direction
            double vectorDirectionInDegree = atan2(dy,dx)*57.2958;
            vectorDirectionInDegree += 90; //ccw
            if(vectorDirectionInDegree<=180) vectorDirectionInDegree-=360;
            double coverage = 1/coverageHistory.back();
            diff = getObstacleGrid(i,j);
            xVect += sin(vectorDirectionInDegree*0.0175)*(coverage * diff * ((1/absDist)-(1/m_setting.repulseRange)) * dx / (pow(absDist,3)));
            yVect += cos(vectorDirectionInDegree*0.0175)*(coverage * diff * ((1/absDist)-(1/m_setting.repulseRange)) * dy / (pow(absDist,3)));
          }
        }
      }
      
      if(getOccupancyGrid(i,j)>0){      // repulsed by values > 0 in m_setting.repulseRange range
        if(absDist<=m_setting.repulseRange){
          if(xCoor!=i && yCoor!=j){
            double coverage = 1/coverageHistory.back();
            xVect += (coverage * diff * ((1/absDist)-(1/m_setting.repulseRange)) * dx / (pow(absDist,3)));
            yVect += (coverage * diff * ((1/absDist)-(1/m_setting.repulseRange)) * dy / (pow(absDist,3)));
          }
        }
      }
      /* Attractive force too strong; temporary disable to see what happens*/
      else if (getOccupancyGrid(i,j)<=0){// attracted by values <= 0 in any range
        //if(absDist<=m_setting.attractRange){
          double coverage = 0.00001*coverageHistory.back();
          if(getLocalCoverage(i,j)>0.9) coverage = coverage * 10000;
          xVect += (coverage * diff * dx * pow(absDist,2));
          yVect += (coverage * diff * dy * pow(absDist,2));
          //xVect += (coverage * diff * ((1/absDist)-(1/m_setting.attractRange)) * dx / (pow(absDist,3)));
          //yVect += (coverage * diff * ((1/absDist)-(1/m_setting.attractRange)) * dy / (pow(absDist,3)));
        //}
      }
      //else{
       // cout << "ERROR: Sensing Grid value not in range!\n";
       // exit(1);
      //}
      
      // Evaluate random noise RobotMap
			// Treat any positive numbers in the grid as repulsive
      double RNG_diff = noiseGrid[i][j];
		  if(absDist<=m_setting.repulseRange){
				if(xCoor!=i && yCoor!=j){
					xVect += (RNG_diff * ((1/absDist)-(1/m_setting.repulseRange)) * dx / (pow(absDist,3)));
					yVect += (RNG_diff * ((1/absDist)-(1/m_setting.repulseRange)) * dy / (pow(absDist,3)));
				}
		  }
			// Evaluate obstacle occupancy grid
			// Treat any positive numbers in the grid as repulsive
			/*if(absDist<=m_setting.repulseRange){
        double obs_diff = (getObstacleGrid(i,j) );// obstaclePotential
        if(xCoor!=i && yCoor!=j){
          xVect += (pow(obs_diff,1) * ((1/absDist)-(1/m_setting.repulseRange)) * dx / (pow(absDist,3)));
          yVect += (pow(obs_diff,1) * ((1/absDist)-(1/m_setting.repulseRange)) * dy / (pow(absDist,3)));;
        }
      }*/
    }
  }
  // repulsed by the other robots
  
  for(unsigned int i=0;i<m_robotList.size();i++){
    for(unsigned int j=0;j<m_robotList.size();j++){
      if(i!=j){
        Vector2D robotPositionAD = m_robotList[i].getPosition();
        Vector2I robotPositionA;
        robotPositionA.x = robotPositionAD.x / m_setting.gridDimension;
        robotPositionA.y = robotPositionAD.y / m_setting.gridDimension;
        Vector2D robotPositionBD = m_robotList[j].getPosition();
        Vector2I robotPositionB;
        robotPositionB.x = robotPositionBD.x / m_setting.gridDimension;
        robotPositionB.y = robotPositionBD.y / m_setting.gridDimension;
        
        double dx = robotPositionA.x-robotPositionB.x;
        double dy = robotPositionA.y-robotPositionB.y;
        double absDist = sqrt(dx*dx+dy*dy);  // tho(q)
        if(absDist<=m_setting.repulseRange){
          //if(absDist<=3) cout << "Warning: Robot #" << i+1 << " & #" << j+1 << " are very close!\n";
          int x = robotPositionA.x, y = robotPositionA.y;
          vectorField[x][y].x += 0.001*((pow(gridSize,2) * ((1/absDist)-(1/m_setting.repulseRange)) * dx ) / (pow(absDist,3)));
          vectorField[x][y].y += 0.001*((pow(gridSize,2) * ((1/absDist)-(1/m_setting.repulseRange)) * dy ) / (pow(absDist,3)));
        }
      }
    }
  }
  
  double tempSum = sqrt(xVect*xVect+yVect*yVect);
  if(tempSum!=0){
    xVect = xVect / (tempSum*2);
    yVect = yVect / (tempSum*2);
  }
  updateField(xCoor, yCoor, xVect, yVect);
}

double RobotMap::getRobotSpeed(int id){
  int x = m_robotList[id].getPositionX()/m_setting.gridDimension, y = m_robotList[id].getPositionY()/m_setting.gridDimension;
  calculateCellPotential(x,y);
  double currHeading = m_robotList[id].getHeading();
  double potentialHeading = atan2(vectorField[x][y].y, vectorField[x][y].x);
  double diffInHeading = (potentialHeading - currHeading) * 57.2958;
  cout << "currHeading: " << currHeading << endl;
  cout << "potentialHeading: " << potentialHeading << endl;
  cout << "diffInHeading: " << diffInHeading << endl;
  
  vector<Vector2D> otherRobotPosition;
  for(unsigned int i=0;i<m_robotList.size();i++){
    if(i!=id){
      Vector2D otherRobot = m_robotList[i].getPosition();
      otherRobotPosition.push_back(otherRobot);
    }
  }
  
  bool isNearObstacle = false;
  for(int i=0;i<3;i++){
    for(int j=0;j<3;j++){
      if(getObstacleGrid(x-1+i, y-1+j)!=0){
        isNearObstacle = true;
      }
      for(int k=0;k<otherRobotPosition.size();k++){
        if(((otherRobotPosition[k].x/m_setting.gridDimension)==x-1+i) && ((otherRobotPosition[k].y/m_setting.gridDimension)==y-1+i))
          isNearObstacle = true;
      }
    }
  }
  
  bool isObstacleAhead = false;
  for(int i=0;i<7;i++){
    double tempX = x+i*cos(potentialHeading);
    double tempY = y+i*sin(potentialHeading);
    for(int k=0;k<otherRobotPosition.size();k++){
      if(abs((otherRobotPosition[k].x/m_setting.gridDimension)-tempX)<=3 && abs((otherRobotPosition[k].y/m_setting.gridDimension)-tempY)<=3)
        isObstacleAhead = true;
    }
    int tempXint = tempX, tempYint = tempY;
    if(getObstacleGrid(tempXint, tempYint)!=0)  isObstacleAhead = true;
  }
  
  double speed = 10; // percentage of full speed
  if(!isObstacleAhead)
    speed = 30;
  if(diffInHeading >=90 && isNearObstacle)
    speed = -10;
  if(isObstacleAhead)
    speed = -10;
  
  return speed;
}
double RobotMap::getRobotHeading(int id){
  // Sets the rot ratio (from -100 (full right) to 100 (full left)
  int x = m_robotList[id].getPositionX()/m_setting.gridDimension, y = m_robotList[id].getPositionY()/m_setting.gridDimension;
  calculateCellPotential(x,y);
  double currHeading = m_robotList[id].getHeading();
  double potentialHeading = atan2(vectorField[x][y].y, vectorField[x][y].x) * 57.2958;
  double diffInHeading = (potentialHeading - currHeading);
  cout << "currHeading: " << currHeading << endl;
  cout << "potentialHeading: " << potentialHeading << endl;
  cout << "diffInHeading: " << diffInHeading << endl;
  
  if(diffInHeading > 180) diffInHeading -= 180;
  if(diffInHeading < -180) diffInHeading += 180;
  if(diffInHeading > 100 && diffInHeading < 180) diffInHeading = 100;
  if(diffInHeading < -100 && diffInHeading > -180) diffInHeading = -100;
  cout << "RotRatio: " << diffInHeading << endl;
  return diffInHeading; // should be percentage of full speed; temporarily assume 100% for now
}
/*
bool RobotMap::isObstacleCellEnclosed(x,y){
  // go all direction and if all has wall then true
  // flood-fill algorithm
}
*/


void RobotMap::calculateMapPotential(){
  //cout << "calculateMapPotential:start\n";
  for(int i=0;i<gridSize;i++){
    for(int j=0;j<gridSize;j++){
      calculateCellPotential(i,j);
      //cout << "calculateMapPotential:calculatng\n";
    }
  }
  //cout << "calculateMapPotential:finished\n";
}


/********************Class Functions************************************
 * 1. Output RobotMap onto screen / to file
 * 2. RobotMap plotter
 **********************Output & Plotter********************************/
void RobotMap::generateReport(){
  //printRobotMap();
  updateCounter++;
  //cout << "calculateMapPotential\n";
  calculateMapPotential();
  //cout << "outputRobotMap\n";
  outputRobotMap();
  //cout << "outputVectorField\n";
  outputVectorField(updateCounter);
}
void RobotMap::outputRobotMap(){
  stringstream v_fileName;
  v_fileName << "Vector_" << fileNumber(updateCounter) << ".tmp";
  ofstream v_output;
  //assert (! v_output.fail( ));
  v_output.open(v_fileName.str().c_str());
  if(v_output.is_open()){
    cout << "Outputting: outputRobotMap " << v_fileName.str().c_str() << endl;
    v_output << "#*****OUTPUTTING RobotMap #" << updateCounter << "*****\n";
    for(int j=gridSize-1;j>=0;j--){
      for(int i=0;i<gridSize;i++){
        v_output << i+0.5 << "  "
             << j+0.5 << "  "
             << vectorField[i][j].x << "  "
             << vectorField[i][j].y << endl;
      }
    }
    v_output.close();
  }
  else{
    cout << "outputRobotMap failed to open: " << v_fileName << endl;
    //exit(1);
  }
  //assert (! v_output.fail( ));
  //stringstream p_fileName;
  //p_fileName << "RobotPosition_" << updateCounter << ".tmp";
  //ofstream p_output;
  //p_output.open(p_fileName.str().c_str());
  //p_output << "#*****OUTPUTTING ROBOT POSITION #" << updateCounter << "*****\n";
  //p_output << robotPosition.x+0.5 << ", " << robotPosition.y+0.5 << ", 0.2" << endl;
  //p_output.close();

  //outputVectorField(updateCounter);
  //getcoverageHistory();
  //getAveragePotential();
}
void RobotMap::outputVectorField(int RobotMapNumber){
  ofstream fout;
  string fileNumber_str = fileNumber(updateCounter);
  string fileName = "GNUplot_plotVector_" + fileNumber_str + ".dat";
  fout.open(fileName.c_str());
  if(fout.is_open()){
    cout << "Outputting: outputVectorField " << fileName << endl;
    fout << "set terminal png truecolor\n";
    fout << "set output 'VectorField_" << fileNumber_str << ".png'\n";
    fout << "set title \"GNUPlot 2D Potential Vector Field #" << fileNumber_str << "\"\n";
    fout << "set xlabel \"X\"\n";
    fout << "set ylabel \"Y\"\n";
    fout << "set xrange [0:" << gridSize << "]\n";
    fout << "set yrange [0:" << gridSize << "]\n";
    fout << "set xtics 1\n";
    fout << "set ytics 1\n";
    fout << "set grid\n";
    for(int i=0;i<gridSize;i++){
      for(int j=0;j<gridSize;j++){
        if(getObstacleGrid(i,j)==0){
          double level = 3*getOccupancyGrid(i,j)/(MaxPotential+0.1);
          if(level>=1) level = 1;
          fout << "set object " << i*gridSize+j+1 
          << " rect from " << i << "," << j << " to "<< i+1 << "," << j+1
          << " fc rgb \"blue\" fs transparent solid "
          << level << " noborder\n";
        }
        else{
          double level = 3*getObstacleGrid(i,j)/(MaxPotential+0.1);
          if(level>=1) level = 1;
          fout << "set object " << i*gridSize+j+1 
          << " rect from " << i << "," << j << " to "<< i+1 << "," << j+1
          << " fc rgb \"black\" fs transparent solid " /*<< obstacleGrid[i][j]/MaxPotential*/
          << level << " noborder\n";
        }
      }
    }
    for(unsigned int i=0;i<m_robotList.size();i++){
      Vector2D position = m_robotList[i].getPosition();
      fout << "set object " << gridSize*gridSize+2*i << " circle at "
        //<< position.x+0.5 << "," << robotPosition[i].y+0.5 
        << position.x / m_setting.gridDimension << "," << position.y / m_setting.gridDimension
        << " size 0.3 fc rgb \"red\" fs transparent solid 0.3 noborder" << endl;
      double heading = m_robotList[i].getHeading();
      double headingInDegree = heading;// * 180 / 3.1416;
      fout << "set object " << gridSize*gridSize+2*i+1 
        << " circle   arc [ " << headingInDegree-30 << " : " << headingInDegree+30
        << " ] fc rgb \"red\" fs transparent solid 0.3 noborder" << endl;
      fout << "set object " << gridSize*gridSize+2*i+1 
        << "circle at " << position.x/ m_setting.gridDimension << "," << position.y/ m_setting.gridDimension
        << " size 10 " << endl;
    }
  //  fout << "set isosam 101,101\n";
  //  fout << "set key under Left reverse\n";
  //  fout << "plot \"Vector_"<< fileNumber_str <<".tmp\" with vec, \"RobotPosition_" 
  //    << fileNumber_str <<".tmp\" with circles lc rgb \"red\" fs transparent solid 0.3 noborder";
    fout << "plot \"Vector_"<< fileNumber_str <<".tmp\" notitle with vec";
    fout.close();
  }
  else{
    cout << "outputVectorField failed to open: " << fileName << endl;
    //exit(1);
  }
}


void RobotMap::outputCoverageHistory(){
  stringstream c_fileName;
  c_fileName << "Coverage.tmp";
  ofstream c_output;
  c_output.open(c_fileName.str().c_str());
  
  c_output << "#*****OUTPUTTING COVERAGE / Total steps = " << updateCounter << "*****\n";
  for(unsigned int i=0;i<coverageHistory.size();i++){
      c_output << coverageHistory[i]*100 << endl;
  }
  c_output.close();
  
  ofstream fout;
  string fileName = "GNUplot_plotCoverage.dat";
  fout.open(fileName.c_str());
  fout << "set terminal png truecolor\n";
  fout << "set output 'TotalcoverageHistory.png'\n";
  fout << "set title \"GNUPlot Total RobotMap Coverage\"\n";
  fout << "set xlabel \"Timesteps\"\n";
  fout << "set ylabel \"Coverage Percentage\"\n";
//  fout << "set xrange [0:" << updateCounter << "]\n";
  fout << "set yrange [0:100]\n";
  fout << "plot \"Coverage.tmp\" notitle with lines";
  fout.close();
}
/*
void RobotMap::outputCoverageEfficiency(){
  stringstream c_fileName;
  c_fileName << "Efficiency.tmp";
  ofstream c_output;
  c_output.open(c_fileName.str().c_str());
  
  c_output << "#*****OUTPUTTING COVERAGE / Total steps = " << updateCounter << "*****\n";
  for(unsigned int i=1;i<coverageHistory.size();i++){
      c_output << (coverageHistory[i] - coverageHistory[i-1]) << endl;
  }
  c_output.close();
  
  ofstream fout;
  string fileName = "GNUplot_plotEfficiency.dat";
  fout.open(fileName.c_str());
  fout << "set terminal png truecolor\n";
  fout << "set output 'TotalEfficiencyLevel.png'\n";
  fout << "set title \"GNUPlot Total RobotMap Coverage Efficiency\"\n";
  fout << "set xlabel \"Timesteps\"\n";
  fout << "set ylabel \"Coverage Growth Rate\"\n";
//  fout << "set xrange [0:" << RobotMap_renew_counter << "]\n";
  fout << "set yrange [0:100]\n";
  fout << "plot \"Efficiency.tmp\" notitle with lines";
  fout.close();
}*/
/*
void RobotMap::outputAvgPotential(){
  stringstream c_fileName;
  c_fileName << "AvgPotential.tmp";
  ofstream c_output;
  c_output.open(c_fileName.str().c_str());
  
  c_output << "#*****OUTPUTTING Average Potential / Total steps = " << updateCounter << "*****\n";
  for(int i=0;i<averagePotential.size();i++){
      c_output << averagePotential[i] << endl;
  }
  c_output.close();
  
  ofstream fout;
  string fileName = "GNUplot_plotAvgPotential.dat";
  fout.open(fileName.c_str());
  fout << "set terminal png truecolor\n";
  fout << "set output 'AveragePotential.png'\n";
  fout << "set title \"GNUPlot Average RobotMap Potential\"\n";
  fout << "set xlabel \"Timesteps\"\n";
  fout << "set ylabel \"Average Potentail\"\n";
//  fout << "set xrange [0:" << updateCounter << "]\n";

  fout << "plot \"AvgPotential.tmp\" notitle with lines";
  fout.close();
}
*/

void RobotMap::plotAll(){
//  system ("rm *.png");
//  for(int i=0;i<updateCounter;i++){
//    plotVectorField(i);
//    plotOccupancyGrid(i);
//    system ("gnuplot \"GNUplot_plotVector.dat\"");
//  }
  outputCoverageHistory();
  //outputCoverageEfficiency();
  system ("rm ../plots/*");
  system ("rm ../data/*");
  system ("gnuplot GNUplot_*.dat");
  system ("convert -delay 15 -loop 0 VectorField_*.png VectorField.gif");
  //system ("convert -delay 20 -loop 0 OccupancyGrid_*.png OccupancyGrid.gif");
  system ("mv *.dat *.tmp ../data/");
  //system ("mv *.tmp data/");
  system ("mv *.png *.gif ../plots/");
}

/********************Class Functions************************************
 * Temporary disabled functions
 ****************Temporary disabled functions**************************/
/*
void RobotMap::gotoPosition(ArClientBase * client, Vector goal)
{
  if (!client->dataExists("ratioDrive")) return;

  ArNetPacket packet;
  double myTransRatio = sqrt(goal.x*goalx+goal.y*goal.y);
  double myRotRatio = atan(goal.y, goal.x);
  packet.doubleToBuf(myTransRatio);
  packet.doubleToBuf(myRotRatio);

  client->requestOnce("ratioDrive", &packet);
}
*/
/*
void RobotMap::updateRobotLocation(int id, double x, double y)
{
  Vector positionCoor;

  // get robot location from packet
  // reference frame is individual robot's starting point
  positionCoor.x = static_cast<float>(packet->bufToDouble());
  positionCoor.y = static_cast<float>(packet->bufToDouble());

  // get robot heading
  double th = packet->bufToDouble();

  // also display the robot position
  myRobotCloud->push_back(point);
  myViewer->addCloud(myRobotCloud, 
      myClient->getRobotName() + std::string("robot"));

  
// The RobotMap take the position of a robot, and put onto the RobotMap
  int xCoor = (int) positionCoor.x;
  int yCoor = (int) positionCoor.y;  // make a togrid(double) function if required
  {
    updateGrid(xCoor, yCoor, occupancyGrid[x][y]+gridSize);  // position +gridSize
    updateGrid(xCoor+1, yCoor, occupancyGrid[x][y]+1);
    updateGrid(xCoor, yCoor+1, occupancyGrid[x][y]+1);
    updateGrid(xCoor-1, yCoor, occupancyGrid[x][y]+1);
    updateGrid(xCoor, yCoor-1, occupancyGrid[x][y]+1);  // nearby grid +1
    // if any obstacle, should update the obstacleRobotMap
  }
  calculateRobotMap(xCoor, yCoor);
}
*/
/*
void RobotMap::startScan()
{
  int step=0;
  do{
    for(int i=0;i<myClients.size();i++){
      double xCoor = myClient[i]->getX(), yCoor = myClient[i]->getY();
      updateRobotLocation(xCoor, yCoor);
      calculateRobotMap(xCoor, yCoor);
      gotoPosition(myClinet[i], vectorField[xCoor][yCoor]);
    }
    step++;
  }while(step<gridSize);
}
*/
/*
void RobotMap::moveOn(){
  for(int i=0;i<robotPosition.size();i++){
  int xCoor = robotPosition[i].x;
  int yCoor = robotPosition[i].y;
  double moveX = vectorField[xCoor][yCoor].x;
  double moveY = vectorField[xCoor][yCoor].y;
  double moveAngle = atan2(moveY, moveX) * 180 / 3.14159265;
  int moveCase = sgn(moveAngle) * ((abs(moveAngle)+22.5)/45);
//  cout << "\nMoveCase: " << moveCase << endl ;
  switch(moveCase)    // case divide by 45 degrees
  {
    case 0:  // move to the right
      if(robotPosition[i].x < (gridSize-1))
        robotMoveTo(++robotPosition[i].x, robotPosition[i].y);
      else
        robotMoveTo(robotPosition[i].x, robotPosition[i].y);
      break;
    
    case 1:  // move to upper right
      if(robotPosition[i].x < (gridSize-1) && robotPosition[i].y < (gridSize-1)){
        robotMoveTo(++robotPosition[i].x, ++robotPosition[i].y);  // allow move upright
        //cout << "Move: upright\n" ;
      }
      else if (robotPosition[i].x == (gridSize-1) && robotPosition[i].y == (gridSize-1))
        robotMoveTo(robotPosition[i].x, robotPosition[i].y);      // stays
      else if (robotPosition[i].x == (gridSize-1))
        robotMoveTo(robotPosition[i].x, ++robotPosition[i].y);    // allow move right
      else if (robotPosition[i].y == (gridSize-1))
        robotMoveTo(++robotPosition[i].x, robotPosition[i].y);    // allow move up
      break;
    
    case 2:  // move up
      if(robotPosition[i].y < (gridSize-1))
        robotMoveTo(robotPosition[i].x, ++robotPosition[i].y);
      else
        robotMoveTo(robotPosition[i].x, robotPosition[i].y);
      break;
    
    case 3:  // move up left
      if(robotPosition[i].x > 0 && robotPosition[i].y < (gridSize-1)){
//        cout << "in Move: X=" << robotPosition[i].x << ", Y=" << robotPosition[i].y << endl;
        robotMoveTo(--robotPosition[i].x, ++robotPosition[i].y);  // allow move upleft
//        cout << "in Move: X=" << robotPosition[i].x << ", Y=" << robotPosition[i].y << endl;
//        cout << "Move: upleft\n" ;
      }
      else if (robotPosition[i].x == 0 && robotPosition[i].y == (gridSize-1)){
        robotMoveTo(robotPosition[i].x, robotPosition[i].y);      // stays
//        cout << "Move: stay\n" ;
      }
      else if (robotPosition[i].x == 0){
        robotMoveTo(robotPosition[i].x, ++robotPosition[i].y);    // allow move up
//        cout << "Move: upleft-->up\n" ;
      }
      else if (robotPosition[i].y == (gridSize-1)){
        robotMoveTo(--robotPosition[i].x, robotPosition[i].y);    // allow move left
//        cout << "Move: upleft-->left\n" ;
      }
      break;
    
    case 4:  // move left
      if(robotPosition[i].x > 0)
        robotMoveTo(--robotPosition[i].x, robotPosition[i].y);
      else
        robotMoveTo(robotPosition[i].x, robotPosition[i].y);
      break;
    case -4:
      if(robotPosition[i].x > 0)
        robotMoveTo(--robotPosition[i].x, robotPosition[i].y);
      else
        robotMoveTo(robotPosition[i].x, robotPosition[i].y);
      break;
    
    case -3: // move down left
      if(robotPosition[i].x > 0 && robotPosition[i].y > 0)
        robotMoveTo(--robotPosition[i].x, --robotPosition[i].y);  // allow move upleft
      else if (robotPosition[i].x == 0 && robotPosition[i].y == 0)
        robotMoveTo(robotPosition[i].x, robotPosition[i].y);      // stays
      else if (robotPosition[i].x == 0)
        robotMoveTo(robotPosition[i].x, --robotPosition[i].y);    // allow move down
      else if (robotPosition[i].y == 0)
        robotMoveTo(--robotPosition[i].x, robotPosition[i].y);    // allow move left
      break;
      
    case -2:  // move down
      if(robotPosition[i].y > 0)
        robotMoveTo(robotPosition[i].x, --robotPosition[i].y);
      else
        robotMoveTo(robotPosition[i].x, robotPosition[i].y);
      break;
    
    case -1:  // move down right
      if(robotPosition[i].x < gridSize-1 && robotPosition[i].y >= 0)
        robotMoveTo(++robotPosition[i].x, --robotPosition[i].y);  // allow move upleft
      else if (robotPosition[i].x == gridSize-1 && robotPosition[i].y == 0)
        robotMoveTo(robotPosition[i].x, robotPosition[i].y);      // stays
      else if (robotPosition[i].x == gridSize-1)
        robotMoveTo(robotPosition[i].x, --robotPosition[i].y);    // allow move down
      else if (robotPosition[i].y == 0)
        robotMoveTo(++robotPosition[i].x, robotPosition[i].y);    // allow move left
      break;
    
    default:
      cout << "ERROR! Cannot find propriate next step!\n";
      cout << "Vector field x: " << moveX << ", y: " << moveY << endl;
      exit(1);
  }
      if(robotPosition[i].x < 0 || robotPosition[i].x >= gridSize){
  cout << "ERROR!! Robot goes out of bound in X direction.\n";
  cout << "Robot #" << i << " at " << robotPosition[i].x << "," << robotPosition[i].y << endl;
  exit(1);
      }
      if(robotPosition[i].y < 0 || robotPosition[i].y >= gridSize){
  cout << "ERROR!! Robot goes out of bound in Y direction.\n";
  cout << "Robot #" << i << " at " << robotPosition[i].x << "," << robotPosition[i].y << endl;
  exit(1);
      }
    }
}

void RobotMap::updateMapWithRobotLocation(int x, int y)
{
  // Update the sensing grid with new robot position, x and y
  // x = new x for robot position, y = new x for robot position
  //updateOccupancyGrid(x, y, occupancyGrid[x][y]+1);    // position +1
  //{// sensing the inner rim
    //updateOccupancyGrid(x+1, y, 1);
    //updateOccupancyGrid(x, y+1, 1);
    //updateOccupancyGrid(x-1, y, 1);
    //updateOccupancyGrid(x, y-1, 1);  // nearby grid +1
  //}
  //{ // sensing the outer rim
    //updateOccupancyGrid(x+2, y, 1);
    //updateOccupancyGrid(x+1, y+1, 1);
    //updateOccupancyGrid(x, y+2, 1);
    //updateOccupancyGrid(x-1, y+1, 1);
    //updateOccupancyGrid(x-2, y, 1);
    //updateOccupancyGrid(x-1, y-1, 1);
    //updateOccupancyGrid(x, y-2, 1);
    //updateOccupancyGrid(x+1, y-1, 1);
  //}
  updateOccupancyGrid(x, y, occupancyGrid[x][y]+3);    // position +1
  {// The inner rim
    updateOccupancyGrid(x+1, y, occupancyGrid[x+1][y]+2);
    updateOccupancyGrid(x, y+1, occupancyGrid[x][y+1]+2);
    updateOccupancyGrid(x-1, y, occupancyGrid[x-1][y]+2);
    updateOccupancyGrid(x, y-1, occupancyGrid[x][y-1]+2);  // nearby grid +1
  }
  { // The outer rim
    updateOccupancyGrid(x+2, y, occupancyGrid[x+2][y]+1);
    updateOccupancyGrid(x+1, y+1, occupancyGrid[x+1][y+1]+1);
    updateOccupancyGrid(x, y+2, occupancyGrid[x][y+2]+1);
    updateOccupancyGrid(x-1, y+1, occupancyGrid[x-1][y+1]+1);
    updateOccupancyGrid(x-2, y, occupancyGrid[x-2][y]+1);
    updateOccupancyGrid(x-1, y-1, occupancyGrid[x-1][y-1]+1);
    updateOccupancyGrid(x, y-2, occupancyGrid[x][y-2]+1);
    updateOccupancyGrid(x+1, y-1, occupancyGrid[x+1][y-1]+1);
  }
}
*/
//void RobotMap::updateSensedMap(){
  //// get robot location, robot heading, angle
  //for(unsigned int i=0;i<m_robotList.size();i++){
    //Vector2D position = m_robotList[i].getPosition();
    //double sensorAngle = 1.0472;// 60 degrees
    //double sensorRange = 10, sensorStep = 20;     // 10 meters
    //int numLines = 10;// cast 5 line of sights
    //double dtheta = sensorAngle / (numLines-1), dr = sensorRange / sensorStep ;
    //double baseAngle = m_robotList[i].getHeading() - (sensorAngle/2);
    //for(int j=0;j<numLines;j++){
      //double lineAngle = baseAngle + j*dtheta;
      //double x = position.x, y = position.y;
      //double dx = dr*cos(lineAngle), dy = dr*sin(lineAngle);
      //vector <Vector2I> scanned;
      //for(int k=0;k<sensorStep;k++){
        //x += dx;
        //y += dy;
        //int tempX = x, tempY = y;
        //int mapValue = 100;//getRealMap(tempX, tempY);
        //if(mapValue!=0){
          //updateObstacleGrid(tempX, tempY, mapValue);
          //updateOccupancyGrid(tempX, tempY, MaxPotential); // or mapValue?
          //break; // look up position and check realMap for obstacle;
          //// break out of line of sight loop if see something
        //}
        //Vector2I newScan;
        //newScan.x = tempX;
        //newScan.y = tempY;
        //if(scanned.size()==0){
          //scanned.push_back(newScan);
        //}
        //else{
          //bool matchFlag = false;
          //for(unsigned int z = 0;z<scanned.size();z++){
            //if(scanned[z].x==newScan.x && scanned[z].y==newScan.y){
              //matchFlag = true;
            //}
          //}
          //if(!matchFlag){
            //scanned.push_back(newScan);
          //}
        //}
      //}
      //for(unsigned int z=0;z<scanned.size();z++){
        //int tempX = scanned[z].x, tempY = scanned[z].y;
        ////cout << "(" << tempX << "," << tempY <<")" << endl;
        //updateOccupancyGrid(tempX, tempY, getOccupancyGrid(tempX, tempY)+1);
      //}
    //}
    //updateOccupancyGrid(position.x, position.y, getOccupancyGrid(position.x, position.y)+5);
    //updateOccupancyGrid(position.x+1, position.y, getOccupancyGrid(position.x+1, position.y)+5);
    //updateOccupancyGrid(position.x-1, position.y, getOccupancyGrid(position.x-1, position.y)+5);
    //updateOccupancyGrid(position.x, position.y+1, getOccupancyGrid(position.x, position.y+1)+5);
    //updateOccupancyGrid(position.x, position.y-1, getOccupancyGrid(position.x, position.y-1)+5);
  //}
//}


//void RobotMap::moveRobots(){
  //double velocity = m_setting.velocity;
  //for(unsigned int i=0;i<m_robotList.size();i++){
    //// get robot[i] location and move it accordingly
    //Vector2D position = m_robotList[i].getPosition();
    //double heading = m_robotList[i].getHeading();
    //int x = position.x, y = position.y;
    ////Vector2D potentialForce = vectorField[x][y];
    //double potentialDirection = atan2(vectorField[x][y].y , vectorField[x][y].x);
    //double potentialDirectionInDegree = potentialDirection * 57.2958;
    //double headingInDegree = heading * 57.2958;
    //if(headingInDegree > 180){
      ////cout << "ERRRRROR! Heading = " << headingInDegree << endl;
      //headingInDegree = headingInDegree - 360;
      ////cout << "Now! Heading = " << headingInDegree << endl;
    //}
    //double differenceInDegree = (potentialDirectionInDegree - headingInDegree);
    //if (differenceInDegree <= 45){
      //double horizontalMove = velocity*cos(potentialDirection), verticalMove = velocity*sin(potentialDirection);
      //int newX = position.x+horizontalMove, newY = position.y+verticalMove;
      //if(potentialDirectionInDegree < 90 && potentialDirectionInDegree >= 0){
        //// check if allow to move upper right
        //if(newX >= gridSize-1) horizontalMove = 0;
        //if(newY >= gridSize-1) verticalMove = 0;

        //if(getObstacleGrid(newX, y)!=0) horizontalMove = 0;
        //else if(getObstacleGrid(x, newY)!=0) verticalMove = 0;
        //else if(getObstacleGrid(newX, newY)!=0){
          //if(potentialDirectionInDegree > 45){
            //horizontalMove = 0;
          //}
          //else{
            //verticalMove = 0;
          //}
        //}
      //}
      //else if(potentialDirectionInDegree >= 90){
        //// check if allow to move upper left
        //if(newX <= 0) horizontalMove = 0;
        //if(newY >= gridSize-1) verticalMove = 0;
       
        //if(getObstacleGrid(newX, y)!=0) horizontalMove = 0;
        //else if(getObstacleGrid(x, newY)!=0) verticalMove = 0;
        //else if(getObstacleGrid(newX, newY)!=0){
          //if(potentialDirectionInDegree < 135){
            //horizontalMove = 0;
          //}
          //else{
            //verticalMove = 0;
          //}
        //}
      //}
      //else if(potentialDirectionInDegree <= -90){
        //// check if allow to move left
        //if(newX <= 0) horizontalMove = 0;
        //if(newY <= 0) verticalMove = 0;
        //if(getObstacleGrid(newX, y)!=0) horizontalMove = 0;
        //else if(getObstacleGrid(x, newY)!=0) verticalMove = 0;
        //else if(getObstacleGrid(newX, newY)!=0){
          //if(potentialDirectionInDegree > -135){
            //horizontalMove = 0;
          //}
          //else{
            //verticalMove = 0;
          //}
        //}
      //}
      //else if(potentialDirectionInDegree <= 0){
        //// check if allow to move down
        //if(newX >= gridSize-1) horizontalMove = 0;
        //if(newY <= 0) verticalMove = 0;
        //if(getObstacleGrid(newX, y)!=0) horizontalMove = 0;
        //else if(getObstacleGrid(x, newY)!=0) verticalMove = 0;
        //else if(getObstacleGrid(newX, newY)!=0){
          //if(potentialDirectionInDegree < -45){
            //horizontalMove = 0;
          //}
          //else{
            //verticalMove = 0;
          //}
        //}
      //}
      
      //m_robotList[i].setPosition(position.x+horizontalMove,position.y+verticalMove);
      //if(potentialDirectionInDegree<0) potentialDirectionInDegree += 360;
      //m_robotList[i].setHeading(potentialDirectionInDegree*0.0175);
    //}
    //else{
      ///*cout << "Robot #" << i << " stays at (" << position.x << "," << position.y << ")\n"
          //<< " heading = " << headingInDegree << endl
          //<< " VectorField = " << potentialDirectionInDegree << endl
          //<< " difference = " << differenceInDegree << endl;*/
      //bool isCCW = false;
      //if(abs(potentialDirectionInDegree-(headingInDegree+1))<differenceInDegree){
        //isCCW = true;
      //}      
      //if(isCCW) 
        //headingInDegree += 45;
      //else
        //headingInDegree -= 45;
      
      ////if(headingInDegree > 360) headingInDegree -= 360;
      //if(headingInDegree < 0) headingInDegree += 360;
      //m_robotList[i].setHeading(headingInDegree*0.0175);

    //}
  //}
  //// get new occupancy grid level with respect to this new location/sensor
  //// and update occupancy grid/obstacle grid
  //updateSensedMap();
  //coverageHistory.push_back(getGlobalCoverage());
  //// updateMapWithRobotPosition();
  //calculateMapPotential();
//}
