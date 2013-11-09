#include <ctime>
#include <cmath>

#include "pcl/io/pcd_io.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>


// 'cv.h' conflicts with 'statistical_outlier_removal.h'
//#define KALMAN_FILTER
#ifdef KALMAN_FILTER
#include <cv.h>
#endif

#define STEREO_CAM_DECOMPRESS

#include "helpers.h"
#include "compress.h"
#include "SensorDataHandler.h"


// useful constants for the kalman filter
const int stateDims = 2;
const int measurementDims = 2;
const float processNoiseCovValue = 1e-1;
const float measurementNoiseCovValue = 1e-2;

const double SensorDataHandler::pi = 3.14159165f;
const double SensorDataHandler::toRadian = pi/180;


SensorDataHandler::SensorDataHandler(ArClientBase *client, 
    const char *dataName, const HostInfo &hostInfo)
  : myClient(client), myDataName(strdup(dataName)),
    myRequestFreq(hostInfo.requestFreq), myDisplayCloud(new MyCloud),
    myRobotCloud(new MyCloud), myRobotColor(hostInfo.locationColor),
    myTransformInfo(hostInfo.transformInfo),
    robotID(hostInfo.id)
{
  // set the voxel leaf to 1 cm
  // note: the clouds are storing in mm
  myVoxelLeaf.x = myVoxelLeaf.y = myVoxelLeaf.z = 10.0f;
}

// free up resources
SensorDataHandler::~SensorDataHandler()
{
  delete myDataName;
}

MyCloud::Ptr SensorDataHandler::getDisplayCloud()
{
  return myDisplayCloud;
}

// Writes the single point cloud consisting of robot locations to
// given directory. Writes all the laser point clouds to the given
// directory.
void SensorDataHandler::writeTo(const std::string &outDir)
{
  std::string subDir("");
  std::string filePath = "";
  std::string extension = ".pcd";
  MyCloud::Ptr cloud;

  // create subdirectory for this client's data
  subDir = outDir + "/" + myClient->getRobotName();
  if (!genDir(subDir)) return;

  // robot position cloud filename
  filePath = subDir + "/" + "path" + extension;
  pcl::io::savePCDFile(filePath, *myRobotCloud);

  // write the laser cloud files using the timestamp as name
  for (size_t i = 0; i < myTSClouds.size(); i++) {
    cloud = myTSClouds[i]->getCloud();
    std::ostringstream os;
    os << myTSClouds[i]->getTimeStamp();
    filePath = subDir + "/" + os.str() + extension;
    pcl::io::savePCDFile(filePath, *cloud);
  }
}

// @param fromFrame: local x-y reference frame which is offset from
//   global x-y frame (0,0)
// @param point: coordinates in that frame
// @return: transformed co-ordinates to global x-y frame (0,0)
MyPoint SensorDataHandler::transformPoint(
    const TransformInfo &fromFrame, const MyPoint &point)
{
  MyPoint pointTrans;
  // angle of rotation counterclockwise
  double theta = fromFrame.thetaOffset * toRadian;
  double cosTheta = cos(theta);
  double sinTheta = sin(theta);
  // rotate to align with global frame
  pointTrans.x = point.x * cosTheta - point.y * sinTheta;
  pointTrans.y = point.x * sinTheta + point.y * cosTheta;
  // translate to global frame
  pointTrans.x += fromFrame.xOffset;
  pointTrans.y += fromFrame.yOffset;
  pointTrans.z = point.z;
  return pointTrans;
}

////////////////////////////////////////////////////////////////////
// SensorDataMapHandler
////////////////////////////////////////////////////////////////////

// Attach packet handler.
//
// Kalman filter is applied to estimate the robot position which consists
// of x and y co-ordinate values.
// The measurement is the odometer reading from the robot itself which
// consists of x and y co-ordinate values.
// Currently control information is not incorporated. The directional
// velocities could be used.
SensorDataMapHandler::SensorDataMapHandler(ArClientBase *client,
    const HostInfo &hostInfo)
  : SensorDataHandler(client, "getSensorDataMap", hostInfo), 
    myHandleFtr(this, &SensorDataMapHandler::handle),
    myMapColor(hostInfo.MapColor),
    myCosTheta(cos(myTransformInfo.thetaOffset*toRadian)),
    mySinTheta(sin(myTransformInfo.thetaOffset*toRadian)),
    myRobotCloudFiltered(new MyCloud),
    myKalmanFilter(
#ifdef KALMAN_FILTER
	new cv::KalmanFilter(stateDims, measurementDims)
#else
	NULL
#endif
    )
{
  myClient->addHandler(myDataName, &myHandleFtr);

#ifdef KALMAN_FILTER
  ////////////////////////////////////////////////
  // initialize the model for the kalman filter//
  // ////////////////////////////////////////////

  // set the transition matrix to [ 1 0 ]
  // 				  [ 0 1 ]
  // state transition model
  setIdentity(myKalmanFilter->transitionMatrix);
  setIdentity(myKalmanFilter->processNoiseCov, 
      	      cv::Scalar::all(processNoiseCovValue));
  // measurement model
  setIdentity(myKalmanFilter->measurementMatrix);
  setIdentity(myKalmanFilter->measurementNoiseCov,
      	      cv::Scalar::all(measurementNoiseCovValue));
  setIdentity(myKalmanFilter->errorCovPost);

  // initial state is robot's starting pose
  myKalmanFilter->statePost.at<float>(0) = 0 + myTransformInfo.xOffset;
  myKalmanFilter->statePost.at<float>(1) = 0 + myTransformInfo.yOffset;
#endif
}

// stop the data requests and free up resources
SensorDataMapHandler::~SensorDataMapHandler()
{
  myClient->requestStop(myDataName);
  for (size_t i = 0; i < myRobotInfos.size(); i++)
    delete myRobotInfos[i];
  for (size_t i = 0; i < myTSClouds.size(); i++)
    delete myTSClouds[i];
#ifdef KALMAN_FILTER
  delete myKalmanFilter;
#endif
}


// Decode the data packet received
void SensorDataMapHandler::handle(ArNetPacket *packet)
{
  long timeStamp = getElapsedTime();
  updateRobotLocation(packet, timeStamp);
  updateMapReadings(packet, timeStamp);
}

// Start requesting data packets 
void SensorDataMapHandler::request()
{
  myClient->request(myDataName, myRequestFreq);
}

// Extract robot location from packet and update the clouds holding
// robot locations.
void SensorDataMapHandler::updateRobotLocation(ArNetPacket *packet,
    long timeStamp)
{
  MyPoint origPoint;

  // get robot location from packet
  // reference frame is individual robot's starting point
  origPoint.x = static_cast<float>(packet->bufToDouble());
  origPoint.y = static_cast<float>(packet->bufToDouble());
  origPoint.z = 0;

  // transformation needed for multiple robots
  MyPoint pointTrans = transformPoint(myTransformInfo, origPoint);
  pointTrans.rgba = myRobotColor;

  // get robot heading
  double th = packet->bufToDouble();
  // store the robot position when the scan is taken
  myRobotInfos.push_back(new RobotInfo(pointTrans, timeStamp, th));

  // also add to cloud which will displayed
  myDisplayCloud->push_back(pointTrans);
  // add to cloud for robot positions so that it can be written
  // as single file
  myRobotCloud->push_back(pointTrans);
  
  // Update robot location in RobotMap
  // is this where the new robot is??????
  map->updateRobotInfo(robotID, pointTrans.x, pointTrans.y, th);
  //std::cout << "******UPDATING ROBOT INFO******\n";
#ifdef KALMAN_FILTER
  // kalman filter of robot position
  filterRobotLocation(pointTrans);
#endif
}

// Extract Map readings from packet and update the clouds holding
// Map readings.
void SensorDataMapHandler::updateMapReadings(ArNetPacket *packet, 
    long timeStamp)
{
  MyPoint origPoint;
  MyPoint pointTrans;
  MyCloud::Ptr tempMapCloud(new MyCloud);

  // get number of Map readings
  int nPoints = packet->bufToByte4();

  // extraction of point co-ordinates
  for (int i = 0; i < nPoints; i++) {
    // reference frame is individual robot's starting point
    origPoint.x = static_cast<float>(packet->bufToDouble());
    origPoint.y = static_cast<float>(packet->bufToDouble());
    origPoint.z = static_cast<float>(packet->bufToDouble());

    // transformation needed for multiple robots
    pointTrans = transformPoint(myTransformInfo, origPoint);
    pointTrans.rgba = myMapColor;

    tempMapCloud->push_back(pointTrans);
    
    // Update Scans in RobotMap
    // Need to get this scan into the map!!!
  }
  map->updateRobotMap(robotID, *tempMapCloud);
  //std::cout << "******UPDATING ROBOT MAP******\n";
  // downsample the current Map cloud and store it
  tempMapCloud = voxelFilter(tempMapCloud, myVoxelLeaf);
  *myDisplayCloud += *tempMapCloud;
  myTSClouds.push_back(new TSCloud(tempMapCloud, timeStamp));
  // downsample the aggregate Map cloud
  myDisplayCloud = voxelFilter(myDisplayCloud, myVoxelLeaf);
}

// kalman filtering of robot position
void SensorDataMapHandler::filterRobotLocation(MyPoint &measured)
{
#ifdef KALMAN_FILTER
  myKalmanFilter->predict();	// perform prediction

  // fill measurement matrix with values from odometer
  cv::Mat measurement(2, 1, CV_32F);
  measurement.at<float>(0) = measured.x;
  measurement.at<float>(1) = measured.y;

  // adjust kalman filter state with measurement
  cv::Mat state(2, 1, CV_32F);
  state = myKalmanFilter->correct(measurement);

  // retreive state values and give it white color for display
  MyPoint pointFiltered;
  pointFiltered.x = state.at<float>(0);
  pointFiltered.y = state.at<float>(1);
  pointFiltered.z = 0;
  pointFiltered.rgba = rgba(255,255,255);

  // remember the filtered positions and display
  myRobotCloudFiltered->push_back(pointFiltered);
  myDisplayCloud->push_back(pointFiltered);
#endif
}

////////////////////////////////////////////////////////////////////
// SensorDataLaserHandler
////////////////////////////////////////////////////////////////////

// Attach packet handler.
//
// Kalman filter is applied to estimate the robot position which consists
// of x and y co-ordinate values.
// The measurement is the odometer reading from the robot itself which
// consists of x and y co-ordinate values.
// Currently control information is not incorporated. The directional
// velocities could be used.
SensorDataLaserHandler::SensorDataLaserHandler(ArClientBase *client,
    const HostInfo &hostInfo)
  : SensorDataHandler(client, "getSensorDataLaser", hostInfo), 
    myHandleFtr(this, &SensorDataLaserHandler::handle),
    myLaserColor(hostInfo.laserColor),
    myCosTheta(cos(myTransformInfo.thetaOffset*toRadian)),
    mySinTheta(sin(myTransformInfo.thetaOffset*toRadian)),
    myRobotCloudFiltered(new MyCloud),
    myKalmanFilter(
#ifdef KALMAN_FILTER
	new cv::KalmanFilter(stateDims, measurementDims)
#else
	NULL
#endif
    )
{
  myClient->addHandler(myDataName, &myHandleFtr);

#ifdef KALMAN_FILTER
  ////////////////////////////////////////////////
  // initialize the model for the kalman filter//
  // ////////////////////////////////////////////

  // set the transition matrix to [ 1 0 ]
  // 				  [ 0 1 ]
  // state transition model
  setIdentity(myKalmanFilter->transitionMatrix);
  setIdentity(myKalmanFilter->processNoiseCov, 
      	      cv::Scalar::all(processNoiseCovValue));
  // measurement model
  setIdentity(myKalmanFilter->measurementMatrix);
  setIdentity(myKalmanFilter->measurementNoiseCov,
      	      cv::Scalar::all(measurementNoiseCovValue));
  setIdentity(myKalmanFilter->errorCovPost);

  // initial state is robot's starting pose
  myKalmanFilter->statePost.at<float>(0) = 0 + myTransformInfo.xOffset;
  myKalmanFilter->statePost.at<float>(1) = 0 + myTransformInfo.yOffset;
#endif
}

// stop the data requests and free up resources
SensorDataLaserHandler::~SensorDataLaserHandler()
{
  myClient->requestStop(myDataName);
  for (size_t i = 0; i < myRobotInfos.size(); i++)
    delete myRobotInfos[i];
  for (size_t i = 0; i < myTSClouds.size(); i++)
    delete myTSClouds[i];
#ifdef KALMAN_FILTER
  delete myKalmanFilter;
#endif
}

// Decode the data packet received
void SensorDataLaserHandler::handle(ArNetPacket *packet)
{
  long timeStamp = getElapsedTime();
  updateRobotLocation(packet, timeStamp);
  updateLaserReadings(packet, timeStamp);
}

// Start requesting data packets 
void SensorDataLaserHandler::request()
{
  myClient->request(myDataName, myRequestFreq);
}

// Extract robot location from packet and update the clouds holding
// robot locations.
void SensorDataLaserHandler::updateRobotLocation(ArNetPacket *packet,
    long timeStamp)
{
  MyPoint origPoint;

  // get robot location from packet
  // reference frame is individual robot's starting point
  origPoint.x = static_cast<float>(packet->bufToDouble());
  origPoint.y = static_cast<float>(packet->bufToDouble());
  origPoint.z = 0;

  // transformation needed for multiple robots
  MyPoint pointTrans = transformPoint(myTransformInfo, origPoint);
  pointTrans.rgba = myRobotColor;

  // get robot heading
  double th = packet->bufToDouble();
  // store the robot position when the scan is taken
  myRobotInfos.push_back(new RobotInfo(pointTrans, timeStamp, th));

  // also add to cloud which will displayed
  myDisplayCloud->push_back(pointTrans);
  // add to cloud for robot positions so that it can be written
  // as single file
  myRobotCloud->push_back(pointTrans);
  
  // Update robot location in RobotMap
  // is this where the new robot is??????
  map->updateRobotInfo(robotID, pointTrans.x, pointTrans.y, th);
  //std::cout << "******UPDATING ROBOT INFO******\n";
#ifdef KALMAN_FILTER
  // kalman filter of robot position
  filterRobotLocation(pointTrans);
#endif
}

// Extract laser readings from packet and update the clouds holding
// laser readings.
void SensorDataLaserHandler::updateLaserReadings(ArNetPacket *packet, 
    long timeStamp)
{
  MyPoint origPoint;
  MyPoint pointTrans;
  MyCloud::Ptr tempLaserCloud(new MyCloud);

  // get number of laser readings
  int nPoints = packet->bufToByte4();

  // extraction of point co-ordinates
  for (int i = 0; i < nPoints; i++) {
    // reference frame is individual robot's starting point
    origPoint.x = static_cast<float>(packet->bufToDouble());
    origPoint.y = static_cast<float>(packet->bufToDouble());
    origPoint.z = static_cast<float>(packet->bufToDouble());

    // transformation needed for multiple robots
    pointTrans = transformPoint(myTransformInfo, origPoint);
    pointTrans.rgba = myLaserColor;

    tempLaserCloud->push_back(pointTrans);
    
    // Update Scans in RobotMap
    // Need to get this scan into the map!!!
  }
  map->updateRobotMap(robotID, *tempLaserCloud);
  //std::cout << "******UPDATING ROBOT MAP******\n";
  // downsample the current laser cloud and store it
  tempLaserCloud = voxelFilter(tempLaserCloud, myVoxelLeaf);
  *myDisplayCloud += *tempLaserCloud;
  myTSClouds.push_back(new TSCloud(tempLaserCloud, timeStamp));
  // downsample the aggregate laser cloud
  myDisplayCloud = voxelFilter(myDisplayCloud, myVoxelLeaf);
}

// kalman filtering of robot position
void SensorDataLaserHandler::filterRobotLocation(MyPoint &measured)
{
#ifdef KALMAN_FILTER
  myKalmanFilter->predict();	// perform prediction

  // fill measurement matrix with values from odometer
  cv::Mat measurement(2, 1, CV_32F);
  measurement.at<float>(0) = measured.x;
  measurement.at<float>(1) = measured.y;

  // adjust kalman filter state with measurement
  cv::Mat state(2, 1, CV_32F);
  state = myKalmanFilter->correct(measurement);

  // retreive state values and give it white color for display
  MyPoint pointFiltered;
  pointFiltered.x = state.at<float>(0);
  pointFiltered.y = state.at<float>(1);
  pointFiltered.z = 0;
  pointFiltered.rgba = rgba(255,255,255);

  // remember the filtered positions and display
  myRobotCloudFiltered->push_back(pointFiltered);
  myDisplayCloud->push_back(pointFiltered);
#endif
}


////////////////////////////////////////////////////////////////////
// SensorDataStereoCamHandler
////////////////////////////////////////////////////////////////////

SensorDataStereoCamHandler::SensorDataStereoCamHandler(ArClientBase *client,
    const HostInfo &hostInfo)
  : SensorDataHandler(client, "getSensorDataStereoCam", hostInfo),
    myHandleFtr(this, &SensorDataStereoCamHandler::handle),
    myStatFilterK(20),
    myDataName2("getSensorDataStereoCam2"),
    myHandleFtr2(this, &SensorDataStereoCamHandler::handle2)
{
  myClient->addHandler(myDataName, &myHandleFtr);
  myClient->addHandler(myDataName2, &myHandleFtr2);
}

// stop the data requests and free up resources
SensorDataStereoCamHandler::~SensorDataStereoCamHandler()
{
  myClient->requestStop(myDataName);
  myClient->requestStop(myDataName2);
}

void SensorDataStereoCamHandler::request()
{
  myClient->request(myDataName, myRequestFreq);
  myClient->request(myDataName2, myRequestFreq);
}

// Decodes packet received from stereocamera
void SensorDataStereoCamHandler::handle(ArNetPacket *packet)
{
  long timeStamp = getElapsedTime();

  static MyPoint point;
  MyCloud::Ptr tempCloud(new MyCloud);

  // get robot location from packet
  // reference frame is individual robot's starting point
  point.x = static_cast<float>(packet->bufToDouble());
  point.y = static_cast<float>(packet->bufToDouble());
  point.z = 0.0;
  point.rgba = myRobotColor; 

  // get robot heading
  double th = packet->bufToDouble();
  myDisplayCloud->push_back(point);
  myRobotCloud->push_back(point);
  // store the robot position when the scan is taken
  myRobotInfos.push_back(new RobotInfo(point, timeStamp, th));

  // Retrieve header information
  int nPoints = packet->bufToByte4();

#ifdef STEREO_CAM_DECOMPRESS
  static unsigned compressed = 0;
  TransformInfo localPose(point.x, point.y, th);
  static MyPoint pointTrans;
#endif
  // create a point using data section of packet
  for (int i = 0; i < nPoints; i++) {
    // get co-ordinate information
#ifdef STEREO_CAM_DECOMPRESS
    compressed = packet->bufToUByte4();
    point.x = extractX(compressed);
    point.y = extractY(compressed);
    point.z = extractZ(compressed);
    pointTrans = transformPoint(localPose, point);
    point.x = pointTrans.x;
    point.y = pointTrans.y;
#else
    point.x = static_cast<float>(packet->bufToByte2());
    point.y = static_cast<float>(packet->bufToByte2());
    point.z = static_cast<float>(packet->bufToByte2());
#endif
    // get color information
    point.r = packet->bufToByte();
    point.g = packet->bufToByte();
    point.b = packet->bufToByte();
    // add point to the cloud
    tempCloud->push_back(point);
  }

  tempCloud = statsFilter(tempCloud, myStatFilterK);
  *myDisplayCloud += *tempCloud;
  myTSClouds.push_back(new TSCloud(tempCloud, timeStamp));
}


// Decodes packet received from stereocamera
void SensorDataStereoCamHandler::handle2(ArNetPacket *packet)
{
  long timeStamp = getElapsedTime();

  static MyPoint point;
  MyCloud::Ptr tempCloud(new MyCloud);

  // get robot location from packet
  // reference frame is individual robot's starting point
  point.x = static_cast<float>(packet->bufToDouble());
  point.y = static_cast<float>(packet->bufToDouble());
  point.z = 0.0;
  point.rgba = myRobotColor; 

  // get robot heading
  double th = packet->bufToDouble();
  myDisplayCloud->push_back(point);
  myRobotCloud->push_back(point);
  // store the robot position when the scan is taken
  myRobotInfos.push_back(new RobotInfo(point, timeStamp, th));

  // Retrieve header information
  int nPoints = packet->bufToByte4();

#ifdef STEREO_CAM_DECOMPRESS
  static unsigned compressed = 0;
  TransformInfo localPose(point.x, point.y, th);
  static MyPoint pointTrans;
#endif
  // create a point using data section of packet
  for (int i = 0; i < nPoints; i++) {
    // get co-ordinate information
#ifdef STEREO_CAM_DECOMPRESS
    compressed = packet->bufToUByte4();
    point.x = extractX(compressed);
    point.y = extractY(compressed);
    point.z = extractZ(compressed);
    pointTrans = transformPoint(localPose, point);
    point.x = pointTrans.x;
    point.y = pointTrans.y;
#else
    point.x = static_cast<float>(packet->bufToByte2());
    point.y = static_cast<float>(packet->bufToByte2());
    point.z = static_cast<float>(packet->bufToByte2());
#endif
    // get color information
    point.r = packet->bufToByte();
    point.g = packet->bufToByte();
    point.b = packet->bufToByte();
    // add point to the cloud
    tempCloud->push_back(point);
  }

  tempCloud = statsFilter(tempCloud, myStatFilterK);
  *myDisplayCloud += *tempCloud;
  myTSClouds.push_back(new TSCloud(tempCloud, timeStamp));
}


////////////////////////////////////////////////////////////////////
// Helper Functions
////////////////////////////////////////////////////////////////////

// Create sensor data handler objects
void createSensorDataHandlers(
    std::vector<ArClientBase *> &clients,
    std::vector<SensorDataHandler *> &sensorDataHandlers,
    std::vector<HostInfo> &hostsInfo,
    RobotMap * map)
{
  SensorDataHandler *sensorDataHandler = NULL;

  for (unsigned int i = 0; i < clients.size(); i++) {
    if (hostsInfo[i].sensor == "laser") {
      sensorDataHandler = new SensorDataLaserHandler(clients[i], hostsInfo[i]);
    }
    else if (hostsInfo[i].sensor == "stereoCam") {
      sensorDataHandler = new SensorDataStereoCamHandler(clients[i], hostsInfo[i]);
    }
    else if (hostsInfo[i].sensor == "map"){
      sensorDataHandler = new SensorDataMapHandler(clients[i], hostInfo[i]);
    }
    //sensorDataHandler->hookupRobotMap(map); // no need to hook up map anymore
    sensorDataHandlers.push_back(sensorDataHandler);
  }
}

// It creates a new ouput folder where all the point clouds will be stored.
// The output folder has name that is based on a chosen prefix which is
// held in the string variable outDirPrefix and the current time.
void writeSensorDataToDisk(
    std::vector<SensorDataHandler *> &sensorDataHandlers)
{
  const std::string outDirPrefix = "clouds";

  // generate a new output directory based on current time
  std::string outDir = outDirPrefix + genTimeStr();
  // error message already spawned by genOutDir
  if (!genDir(outDir)) return;

  for (size_t i = 0; i < sensorDataHandlers.size(); i++)
    sensorDataHandlers[i]->writeTo(outDir);

  std::cout << "Wrote sensor data to: " << outDir << std::endl;
}

// Return average density in given region of a point cloud.
// MinVal holds the minimum values for the co-ordinates and maxVal
// holds the maximum values. These two points represent the furthest
// points in a cuboid region of space.
// Also uses the given divison value to convert to required units
double calcRegionDensity(MyCloud::Ptr cloud,
    			 const MyPoint &minVal, const MyPoint &maxVal,
			 int divisor)
{
  // get number of points in the region
  int nPoints = 0;
  for (size_t i = 0; i < cloud->size(); i++) {
    if (minVal.x <= (*cloud)[i].x && maxVal.x >= (*cloud)[i].x &&
        minVal.y <= (*cloud)[i].y && maxVal.y >= (*cloud)[i].y &&
        minVal.z <= (*cloud)[i].z && maxVal.z >= (*cloud)[i].z) {
      nPoints++;
    }
  }

  // get the volume of the region
  double l = 0.0, b = 0.0, h = 0.0;
  // the min and max of a dimension could have the same sign
  if ((minVal.x < 0.0 && maxVal.x < 0.0) ||
      (minVal.x > 0.0 && maxVal.x > 0.0))
    l = fabs(fabs(maxVal.x) - fabs(minVal.x));
  else
    l = fabs(minVal.x) + fabs(maxVal.x);
  if ((minVal.y < 0.0 && maxVal.y < 0.0) ||
      (minVal.y > 0.0 && maxVal.y > 0.0))
    b = fabs(fabs(maxVal.y) - fabs(minVal.y));
  else
    b = fabs(minVal.y) + fabs(maxVal.y);
  if ((minVal.z < 0.0 && maxVal.z < 0.0) ||
      (minVal.z > 0.0 && maxVal.z > 0.0))
    h = fabs(fabs(maxVal.z) - fabs(minVal.z));
  else
    h = fabs(minVal.z) + fabs(maxVal.z);

  // scale the volume to preferred units
  l /= divisor;
  b /= divisor;
  h /= divisor;
  double v = (h > 0.0) ? l*b*h : l*b;

  return nPoints/v;
}

// Perform voxel filter and return filtered cloud
MyCloud::Ptr voxelFilter(MyCloud::Ptr source, const MyPoint &leafSize)
{
  // set filter settings
  pcl::VoxelGrid<MyPoint> voxelGrid;
  voxelGrid.setInputCloud(source);
  voxelGrid.setLeafSize(leafSize.x, leafSize.y, leafSize.z);

  // filter and return filtered cloud
  MyCloud::Ptr filteredCloud(new MyCloud);
  voxelGrid.filter(*filteredCloud);
  return filteredCloud;
}

// Remove outliers and return filtered cloud
MyCloud::Ptr statsFilter(MyCloud::Ptr source, const int k)
{
  // set filter settings
  pcl::StatisticalOutlierRemoval<MyPoint> outlierRemoval;
  outlierRemoval.setInputCloud(source);
  outlierRemoval.setMeanK(k);
  outlierRemoval.setStddevMulThresh(1.0);

  // filter and return filtered cloud
  MyCloud::Ptr filteredCloud(new MyCloud);
  outlierRemoval.filter(*filteredCloud);
  return filteredCloud;
}
