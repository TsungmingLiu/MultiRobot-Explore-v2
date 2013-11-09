#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cstdlib>

#include "Aria.h"

#include "helpers.h"
#include "ConfigFileReader.h"


// this is the required command line argument
const char *ConfigFileReader::hostsArg = "-hosts";
// this header is required as the first valid line
const char *ConfigFileReader::hostsFileHeader = "servers 2.0";
// these are the valid fields
const char *ConfigFileReader::infoFields[] = {
  "id", // -JL Sep 13
  "ip",
  "port",
  "sensor",
  "locationColor",
  "laserColor",
  "transformInfo",
  "requestFreq",
};
// number of information field types
const size_t ConfigFileReader::infoFieldTypes =
  sizeof(infoFields) / sizeof(infoFields[0]);
// separates fields in the file
const char *ConfigFileReader::myFieldSeparator = " ";
// separates sub fields in the file
const char *ConfigFileReader::mySubFieldSeparator = ",";
// valid sensor names
const char *ConfigFileReader::sensors[] = {
  "laser",
  "stereoCam",
};


// check if argument option is given in command line
// and returns the command line index of the filename
int ConfigFileReader::checkFileArg()
{
  std::string errorMsg("PROVIDE CONFIGURATION FILE USING ARGUMENT\n\t");
  errorMsg += "-hosts 'filename'";

  int fileIndex = -1;
  if (!myParser->checkArgument(hostsArg)) {
    errorExit(errorMsg);
  }

  for (int i = 1; i < myArgc; i++) {
    if (strcmp(hostsArg, myArgv[i]) == 0) {
      fileIndex = i + 1;
      break;
    }
  }

  if (fileIndex == -1 || fileIndex >= myArgc)
    errorExit(errorMsg);
  
  return fileIndex;
}

// return index for given field name from infoFields array
size_t ConfigFileReader::matchFieldIndex(const char *fieldName)
{
  // check each specified field name for a match
  for (size_t i = 0; i < infoFieldTypes; i++)
    if (strcmp(fieldName, infoFields[i]) == 0)
      return static_cast<size_t>(i);
  echo("invalid field name", fieldName);
  printInfoFields();
  errorExit("");
  return 100; // will not reach because of exit above;
}

// Checks the buffer for valid words and stores the indices of those field
// names in the fieldTypes array
void ConfigFileReader::getFieldTypeIndices(const std::string &buffer,
    std::vector<size_t> &fieldTypes)
{
  char *temp = strdup(buffer.c_str());
  char *tok = strtok(temp, myFieldSeparator);
  int i = 0;

  do {
    i = matchFieldIndex(tok);
    fieldTypes.push_back(i);
  }
  while ((tok = strtok(NULL, myFieldSeparator))); 
  free(temp);
}

// Checks what information is supplied in the file
// and returns an array with indicies for information type.
// Hence the information can be in any order.
std::ifstream *ConfigFileReader::getFieldTypes(
    std::vector<size_t> &fieldTypes)
{
  std::string buffer;

  // filename is in this index of myArgv
  int fileIndex = checkFileArg();

  // check for existence of file
  std::ifstream *file = new std::ifstream(myArgv[fileIndex], std::ifstream::in);
  if (file->fail()) errorExit("NO SUCH FILE");

  // check for header line
  getValidLine(*file, buffer);
  // invalid header so cleanup
  if (strcmp(buffer.c_str(), hostsFileHeader) != 0) {
    file->close();
    delete file;
    std::string errorMsg = "REQUIRED HEADER GIVEN BELOW NOT FOUND!!!\n\t";
    errorMsg += hostsFileHeader;
    errorExit(errorMsg);
    return NULL;	// never reached due to exit above
  }

  // next valid line should hold names of info types
  getValidLine(*file, buffer);
  getFieldTypeIndices(buffer, fieldTypes);

  return file;
}

// fill buffer with the next line in stream which is not a comment
void ConfigFileReader::getValidLine(std::ifstream &inFile,
    				    std::string &buffer) 
{
  while (getline(inFile,buffer) && buffer[0] == myCommentChar);
}

// get a list of sub fields as integer types from a single string 
// containing sub fields separated by a selected sub field separator
void ConfigFileReader::getIntSubFields(const std::string &s, 
    std::vector<int> &subFields)
{
  char *temp = strdup(s.c_str());
  char *currSubField = strtok(temp, mySubFieldSeparator);

  do {
    subFields.push_back(atoi(currSubField));
  } while ((currSubField = strtok(NULL, mySubFieldSeparator)));

  free(temp);
}

// reads files of type : servers 1.0
void ConfigFileReader::readHostsFile(std::vector<HostInfo> &hostsInfo)
{
  // store current host info
  HostInfo currHostInfo(0, NULL, 0, "", 0, 0, TransformInfo(0,0,0), 0);

  std::string line;
  std::string field;
  std::vector<int> subFields;
  int fieldIndex;
  std::istringstream lineStream;

  // get array with section information
  std::vector<size_t> fieldTypes;
  std::ifstream *file = getFieldTypes(fieldTypes);

  // process each line according to the field type information
  while (file->good()) {
    // refresh host info to default
    currHostInfo = HostInfo(0,  // temp Robot ID - JL 13 
          NULL,	// ip address
			    myDefaultPort,// port number
			    "laser",// sensor
			    rgba(200,0,0),// location color
			    rgba(0,200,0),// laser data color
			    TransformInfo(0,0,0),
			    1000);	// request frequency in millisec
    // get a single line
    getValidLine(*file, line);
    if (file->fail() || file->eof()) break;

    // set the stream
    lineStream.clear();
    lineStream.str(line);

    // extract fields according to given format
    for (size_t i = 0; i < fieldTypes.size(); i++) {
      // clear the sub fields array
      subFields.clear();
      // first set field type index
      fieldIndex = fieldTypes[i];
      // get the field from the string
      lineStream >> field;

      // extract from the field according to field type
      switch (fieldIndex) {
  case 0:   // Robot ID
	  currHostInfo.id = atoi(field.c_str());
    break;
	case 1:		// ip address
	  currHostInfo.ip = strdup(field.c_str());
	  break;
	case 2:		// port number
	  currHostInfo.port = atoi(field.c_str());
	  break;
	case 3:		// sensor
	  checkSensorName(field);
	  currHostInfo.sensor = field;
	  break;
	case 4:		// location color
	  getIntSubFields(field, subFields);
	  currHostInfo.locationColor = 
	    rgba(subFields[0], subFields[1], subFields[2]);
	  break;
	case 5:		// laser data color
	  getIntSubFields(field, subFields);
	  currHostInfo.laserColor = 
	    rgba(subFields[0], subFields[1], subFields[2]);
	  break;
	case 6:		// transformation info
	  getIntSubFields(field, subFields);
	  currHostInfo.transformInfo = 
	    TransformInfo(subFields[0], subFields[1], subFields[2]);
	  break;
	case 7:		// request frequency in milliseconds
	  currHostInfo.requestFreq = atoi(field.c_str());
	  break;
	default:
	  break;
      }
    }

    // debugging to see if host information was extracted from file
#define debug
#ifdef debug
    echo("robot id", currHostInfo.id);
    echo("ip address", currHostInfo.ip);
    echo("port number", currHostInfo.port);
    echo("sensor", currHostInfo.sensor);
    echo("location color", currHostInfo.locationColor);
    echo("laser color", currHostInfo.laserColor);
    echo("x offset", currHostInfo.transformInfo.xOffset);
    echo("y offset", currHostInfo.transformInfo.yOffset);
    echo("theta offset", currHostInfo.transformInfo.thetaOffset);
    echo("request frequency", currHostInfo.requestFreq);
    std::cout << std::endl;
#endif

    // add current host information to the list of hosts
    // fields not set will get default values
    hostsInfo.push_back(currHostInfo);
  }

  file->close();
  delete file;
}

// display the valid info fields in configuration file
void ConfigFileReader::printInfoFields()
{
  std::cout << std::endl;
  echo("VALID FIELDS ARE:");

  for (size_t i = 0; i < sizeof(infoFields)/sizeof(infoFields[0]); i++) {
    echo(infoFields[i]);
  }
}

// display the valid values for sensor field
void ConfigFileReader::printSensors()
{
  std::cout << std::endl;
  echo("VALID SENSORS ARE:");

  for (size_t i = 0; i < sizeof(sensors)/sizeof(sensors[0]); i++) {
    echo(sensors[i]);
  }
}

// check the supplied string name of sensor for validity
void ConfigFileReader::checkSensorName(const std::string &s)
{
  for (size_t i = 0; i < sizeof(sensors)/sizeof(sensors[0]); i++)
    if (s == sensors[i]) return;

  echo("invalid field name", s);
  printSensors();
  errorExit("");
}
