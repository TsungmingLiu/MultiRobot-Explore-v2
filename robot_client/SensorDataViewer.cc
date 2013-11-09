#include <sstream>

#include "SensorDataHandler.h"
#include "SensorDataViewer.h"


// Initialize the viewer window
SensorDataViewer::SensorDataViewer(const std::string& title,
    std::vector<SensorDataHandler *> &sensorDataHandlers)
  : myViewer(title), myRefreshTime(200),
    mySensorDataHandlers(sensorDataHandlers)
{
  myViewer.setBackgroundColor(0,0,0);
  myViewer.addCoordinateSystem(170.0);
  myViewer.initCameraParameters();
  request();
  initDisplay();
}

// Add point clouds to be displayed in window
void SensorDataViewer::initDisplay()
{
  static std::ostringstream os;
  for (size_t i = 0; i < mySensorDataHandlers.size(); i++) {
    os.str("");
    os << "cloud" << i;
    myViewer.addPointCloud(
	mySensorDataHandlers[i]->getDisplayCloud(),
	os.str());
  }
  myViewer.spinOnce(myRefreshTime/2);
}

// Show the update point clouds
void SensorDataViewer::updateDisplay()
{
  static std::ostringstream os;
  for (size_t i = 0; i < mySensorDataHandlers.size(); i++) {
    os.str("");
    os << "cloud" << i;
    myViewer.updatePointCloud(
      mySensorDataHandlers[i]->getDisplayCloud(), 
      os.str());
  }
  myViewer.spinOnce(myRefreshTime);
}

// Refresh the viewer window so that it may load new data
void SensorDataViewer::request()
{
  for (size_t i = 0; i < mySensorDataHandlers.size(); i++)
    mySensorDataHandlers[i]->request();
}


// create a viewer
void createViewer(SensorDataViewer *&viewer,
    std::vector<SensorDataHandler *> &sensorDataHandlers)
{
  if (viewer) return;
  viewer = new SensorDataViewer("SENSOR DATA VIEW", sensorDataHandlers);
}

