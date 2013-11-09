#ifndef CONFIG_FILE_READER_H
#define CONFIG_FILE_READER_H

// initial offsets of robots from global co-ordinates
struct TransformInfo {
  TransformInfo(int xo, int yo, int to)
    : xOffset(xo), yOffset(yo), thetaOffset(to) { }
  int xOffset;
  int yOffset;
  int thetaOffset;
};

// information about each host which is a server running on a robot
struct HostInfo {
  HostInfo(int id, const char *ipa, int p, const std::string &s, 
      int loc, int lac, TransformInfo ti, int rf)
    : ip(ipa), port(p), sensor(s),
      locationColor(loc), laserColor(lac),
      transformInfo(ti), requestFreq(rf) { }
  int id;
  const char *ip;
  int port;
  std::string sensor;
  int locationColor;
  int laserColor;
  TransformInfo transformInfo;
  int requestFreq;	// in millisecond
};

// Performs checking of the custom file type
class ConfigFileReader {
public:
  ConfigFileReader(int c, char **v, ArArgumentParser *parser)
    : myArgc(c), myArgv(v), myParser(parser) { }
  void readHostsFile(std::vector<HostInfo> &hostsInfo);
  static void printInfoFields();
  static void printSensors();

  static const char *hostsArg;
  static const char *hostsFileHeader;
  static const char *infoFields[];
  static const size_t infoFieldTypes;
  static const char myCommentChar = '#';
  static const char *myFieldSeparator;
  static const char *mySubFieldSeparator;
  static const int myDefaultPort = 7272;
  static const char *sensors[];

private:
  int myArgc;
  char **myArgv;
  ArArgumentParser *myParser;

  int checkFileArg();
  std::ifstream *getFieldTypes(std::vector<size_t> &fieldTypes);
  void getValidLine(std::ifstream &inFile, std::string &buffer);
  void getFieldTypeIndices(const std::string &buffer,
      			   std::vector<size_t> &fieldTypes);
  size_t matchFieldIndex(const char *fieldName);
  void getIntSubFields(const std::string &s, std::vector<int> &subFields);
  void checkSensorName(const std::string &s);
};



#endif
