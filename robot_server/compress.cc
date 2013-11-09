#include <iostream>
#include <cmath>
#include <cassert>
using namespace std;

// check nth bit where n starts from right side
bool checkBit(unsigned val, unsigned n)
{
  assert(n < 32);
  unsigned mask = (1 << n);
  return val & mask;
}

// set nth bit where n starts from right side
unsigned setBit(unsigned val, unsigned n)
{
  assert(n < 32);
  unsigned mask = (1 << n);
  return val | mask;
}

// extract bit segment starting at s and l bits long
//
// e.g. suppose the bit format is
// 31 30 ... 3 2 1 0
// and s = 2, l = 20 then bits 2 to 21 are extracted
unsigned extractBits(unsigned val, unsigned s, unsigned l)
{
  assert(s + l <= 32);
  unsigned mask = (1 << l) - 1;
  unsigned segment = (val >> s);
  return segment & mask;
}

// @param distance: units is mm
// @return: 10bit representation of magnitude of distance.
//   Precision is reduced to cm.
//   Format: 3 bits for meteres and 7 bits for centimeter
//   Range: [0m, 8m)
unsigned compressMagnitude(double distance)
{
  // work on magnitude only
  distance = fabs(distance);
  // get meter portion
  unsigned m = distance/1000;
  // get centimeter portion
  unsigned cm = static_cast<unsigned>(distance/10) % 100;
  return (m << 7) | cm;
}

// @params: mm units
// @return: 2 bits representing sign of y and sign of z.
//   Bit on means negative sign.
unsigned compressSigns(double y, double z)
{
  unsigned ySign = y < 0.0;
  unsigned zSign = z < 0.0;
  return (ySign << 1) | zSign;
}

// @params: mm units
// @return: 32 bit compressed block in which signs represent
//   direction of y and z coordinates. If sign bit is on,
//   the coordinate is in negative axis.
//   format:
//   | 2 bits | 10 bits | 10 bits | 10 bits |
//     signs      x         y         z
unsigned compressPoint(double x, double y, double z)
{
  unsigned signs = compressSigns(y, z) << 30;
  unsigned xMag = compressMagnitude(x) << 20;
  unsigned yMag = compressMagnitude(y) << 10;
  unsigned zMag = compressMagnitude(z);

  return signs | xMag | yMag | zMag;
}

// @param mag: 10 bit compressed magnitude
// @return: meter portion
unsigned extractMeter(unsigned mag)
{
  return extractBits(mag, 7, 3);
}

// @param mag: 10 bit compressed magnitude
// @return: centimeter portion
unsigned extractCentimeter(unsigned mag)
{
  return extractBits(mag, 0, 7);
}

// @param val: 32 bit compressed point value
// @return: x coordinate in mm units with cm precision
int extractX(unsigned val)
{
  unsigned c = extractBits(val, 20, 10);
  unsigned mag = extractMeter(c) * 100 + extractCentimeter(c);
  return mag*10;
}

// @param val: 32 bit compressed point value
// @return: y coordinate in mm units with cm precision
int extractY(unsigned val)
{
  unsigned sign = checkBit(val, 31);
  unsigned c = extractBits(val, 10, 10);
  unsigned mag = extractMeter(c) * 100 + extractCentimeter(c);
  if (sign) mag *= -1;
  return mag*10;
}

// @param val: 32 bit compressed point value
// @return: z coordinate in mm units with cm precision
int extractZ(unsigned val)
{
  unsigned sign = checkBit(val, 30);
  unsigned c = extractBits(val, 0, 10);
  unsigned mag = extractMeter(c) * 100 + extractCentimeter(c);
  if (sign) mag *= -1;
  return mag*10;
}

void testBits()
{
  assert(checkBit(4, 2)); 
  assert(!checkBit(5, 1)); 
  assert(setBit(8, 0) == 9); 
  assert(setBit(10, 2) == 14); 
  assert(extractBits(201201, 8, 4) == 1);
  assert(extractBits(50512, 4, 20) == 3157);
}

void testCompress()
{
  assert(compressMagnitude(5000) == 640);
  assert(compressMagnitude(-5000) == 640);
  assert(compressMagnitude(1123) == 140);
  assert(compressMagnitude(-7053.50) == 901);
  assert(compressSigns(10, 20) == 0);
  assert(compressSigns(-1, 20) == 2);
  assert(compressSigns(100, -33.3) == 1);
  assert(compressSigns(-100, -33.3) == 3);
  assert(compressPoint(1000, 2345, 321.01) == 134514720);
  //assert(compressPoint(3000, -2345.35, -200.23) == 3624175636);
  assert(compressPoint(3000, 2345.35, -200.23) == 1476691988);
}

void testDecompress()
{
  assert(extractMeter(compressMagnitude(5100)) == 5);
  assert(extractMeter(compressMagnitude(-1023)) == 1);
  assert(extractCentimeter(compressMagnitude(-1023)) == 2);
  assert(extractCentimeter(compressMagnitude(-1523)) == 52);
  assert(extractX(compressPoint(7010.03, 7000, 7000)) == 7010);
  assert(extractX(compressPoint(2130.23, 7000, 7000)) == 2130);
  assert(extractY(compressPoint(7010.03, -3321.0, 7000)) == -3320);
  assert(extractY(compressPoint(2130.23, 801, 7000)) == 800);
  assert(extractZ(compressPoint(7010.03, 7000, -100)) == -100);
  assert(extractZ(compressPoint(2130.23, 7000, 2569.22)) == 2560);
}
