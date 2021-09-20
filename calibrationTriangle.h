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

#ifndef CALIBRATIONTRIANGLE_H_
#define CALIBRATIONTRIANGLE_H_

#include "types.h"

namespace stairs
{

class CalibrationTriangle
{
public:
  int load();
  int save() const;
  bool isValid() const;

  typedef Point3 TriangleCorner;
  typedef std::array<TriangleCorner, 3> TriangleCorners;
  enum class Side
  {
    undefined, left, right
  };

  const TriangleCorners &getTriangleCorners() { return triangleCorners; }
  const Side &getLowerQuadrant() { return lowerQuadrant; }

private:
  friend class TriangleParams;

  TriangleCorners triangleCorners{};
  Side lowerQuadrant{};
};

class TriangleParams
{
public:
  int load();
  int calcCalibrationTriangle(CalibrationTriangle &calibrationTriangle) const;

private:
  struct SideLengths
  {
    double a, b, c;

    bool isValid() const;
    double calcHeightA() const;
  } sideLengths{};
  double y3{}, z{};
};

} /* namespace stairs */

#endif /* CALIBRATIONTRIANGLE_H_ */
