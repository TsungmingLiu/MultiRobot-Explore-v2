#ifndef STEREO_CAM_COMPRESS
#define STEREO_CAM_COMPRESS

bool checkBit(unsigned val, unsigned n);
unsigned setBit(unsigned val, unsigned n);
unsigned extractBits(unsigned val, unsigned s, unsigned l);
unsigned compressMagnitude(double distance);
unsigned compressSigns(double y, double z);
unsigned compressPoint(double x, double y, double z);
unsigned extractMeter(unsigned mag);
unsigned extractCentimeter(unsigned mag);
int extractX(unsigned val);
int extractY(unsigned val);
int extractZ(unsigned val);
void testBits();
void testCompress();
void testDecompress();

#endif
