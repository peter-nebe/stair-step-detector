/*
 * stair-step-detector
 * Copyright (c) 2021 Peter Nebe (mail@peter-nebe.dev)
 *
 * This file is part of stair-step-detector.
 *
 * stair-step-detector is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * stair-step-detector is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with stair-step-detector.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "calibrationTriangle.h"
#include <fstream>
#include <cmath>
using namespace std;

namespace stairs
{

namespace
{

const string paramsFilename = "triangle-parameters";
const string paramsId = "triangle parameters";
const string triangleFilename = "calibration-triangle";
const string triangleId = "calibration triangle";
const string equalSign = "=";
const char space = ' ';
const string equalPart = space + equalSign + space;
const string xPart = "x";
const string yPart = "y";
const string zPart = "z";
const string delim = ", ";
const string lowerQuadrantPart = "lowerQuadrant";
const string undefined = "undefined";
const string left = "left";
const string right = "right";
const double minSideLength = 0.01; // meter

template<typename ValueType>
bool readValue(ifstream &file, const string &name, ValueType &value)
{
  while(file)
  {
    string chars;
    file >> chars;
    if(chars == name)
    {
      string sign;
      file >> sign;
      if(sign == equalSign)
      {
        file >> value;
        return static_cast<bool>(file);
      }
    }
  }
  return false;
};

using Side = CalibrationTriangle::Side;

string toString(Side side)
{
  switch(side)
  {
    case Side::left:
      return left;
    case Side::right:
      return right;
    default:
      return undefined;
  }
}

Side fromString(const string &side)
{
  if(side == left)
    return Side::left;
  if(side == right)
    return Side::right;

  return Side::undefined;
}

} // namespace

int CalibrationTriangle::load()
{
  *this = {};

  ifstream file(triangleFilename);
  string id;
  getline(file, id);
  if(id != triangleId)
    return -1;

  bool r = true;
  auto readCorner = [&file, &r](int n, TriangleCorner &c)
                    {
                      const string ns = to_string(n);
                      r = r && readValue(file, xPart + ns, c.x);
                      r = r && readValue(file, yPart + ns, c.y);
                      r = r && readValue(file, zPart + ns, c.z);
                    };

  readCorner(1, triangleCorners[0]);
  readCorner(2, triangleCorners[1]);
  readCorner(3, triangleCorners[2]);

  string side;
  r = r && readValue(file, lowerQuadrantPart, side);
  lowerQuadrant = fromString(side);

  return r ? 0 : -2;
}

int CalibrationTriangle::save() const
{
  ofstream file(triangleFilename);
  file << triangleId << endl;

  auto saveCorner = [&file](int n, const TriangleCorner &c)
                    {
                      file << xPart << n << equalPart << c.x << delim
                           << yPart << n << equalPart << c.y << delim
                           << zPart << n << equalPart << c.z << endl;
                    };

  saveCorner(1, triangleCorners[0]);
  saveCorner(2, triangleCorners[1]);
  saveCorner(3, triangleCorners[2]);

  file << lowerQuadrantPart << equalPart << toString(lowerQuadrant) << endl;

  return file ? 0 : -1;
}

bool CalibrationTriangle::isValid() const
{
  const double minDistQu = minSideLength * minSideLength;
  auto distQu = [](const TriangleCorner &c1, const TriangleCorner &c2)
                {
                  const double dx = c2.x - c1.x;
                  const double dy = c2.y - c1.y;
                  const double dz = c2.z - c1.z;
                  return dx * dx + dy * dy + dz * dz;
                };
  if(distQu(triangleCorners[0], triangleCorners[1]) < minDistQu)
    return false;
  if(distQu(triangleCorners[1], triangleCorners[2]) < minDistQu)
    return false;
  if(distQu(triangleCorners[2], triangleCorners[0]) < minDistQu)
    return false;
  if(lowerQuadrant != Side::left && lowerQuadrant != Side::right)
    return false;

  return true;
}

int TriangleParams::load()
{
  *this = {};

  ifstream file(paramsFilename);
  string id;
  getline(file, id);
  if(id != paramsId)
    return -1;

  bool r = readValue(file, "a", sideLengths.a);
  r = r && readValue(file, "b", sideLengths.b);
  r = r && readValue(file, "c", sideLengths.c);
  r = r && readValue(file, "y3", y3);
  r = r && readValue(file, "z", z);

  return r ? 0 : -2;
}

bool TriangleParams::SideLengths::isValid() const
{
  return a >= minSideLength
      && b >= minSideLength
      && c >= minSideLength;
}

double TriangleParams::SideLengths::calcHeightA() const
{
  const double s = (a + b + c) / 2;
  return 2 * sqrt(s * (s - a) * (s - b) * (s - c)) / a;
}

int TriangleParams::calcCalibrationTriangle(CalibrationTriangle &triangle) const
{
  triangle = {};

  const SideLengths &le = sideLengths;
  if(!le.isValid())
    return -1;

  const double ha = le.calcHeightA();
  const double a2 = le.a / 2;

  CalibrationTriangle::TriangleCorner &c2 = triangle.triangleCorners[2];
  const double minDiff = minSideLength / 10;
  if(le.b < le.c - minDiff)
  {
    const double d = sqrt(le.b * le.b - ha * ha);
    c2.x = -a2 + d;
    triangle.lowerQuadrant = Side::left;
  }
  else if(le.b > le.c + minDiff)
  {
    const double d = sqrt(le.c * le.c - ha * ha);
    c2.x = a2 - d;
    triangle.lowerQuadrant = Side::right;
  }
  else
    return -2;

  CalibrationTriangle::TriangleCorner &c0 = triangle.triangleCorners[0];
  c0.x = -a2;
  c0.y = y3 + ha;
  c0.z = z;

  CalibrationTriangle::TriangleCorner &c1 = triangle.triangleCorners[1];
  c1.x = a2;
  c1.y = c0.y;
  c1.z = z;

  c2.y = y3;
  c2.z = z;

  return 0;
}

} /* namespace stairs */
