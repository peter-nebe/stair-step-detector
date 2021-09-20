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

#include "geometricCalibration.h"
#include "calibrationTriangle.h"
#include "calibrationMark.h"
#include "image.h"
#include "window.h"
#include "drawing.h"
#include "log.h"
#include "qvmTraits.h"
#include "boost/qvm/vec_operations.hpp"
#include <fstream>
#include <iostream>
using namespace std;

namespace stairs
{

namespace
{

const int __numIterations = 10;
const string __calibrationPointsFilename = "calibration-points";
const string __calibrationPointsId = "calibration points";

ostream &operator<<(ostream &os, const Point3f &p)
{
  os << fixed << setprecision(6)
     << setw(9) << p.x << ", "
     << setw(9) << p.y << ", "
     << setw(8) << p.z;
  return os;
}

istream &operator>>(istream &is, Point3f &p)
{
  char ch;
  is >> p.x >> ch >> p.y >> ch >> p.z;
  return is;
}

bool savePoints(const GeometricCalibration::PointSets_t &pointSets)
{
  ofstream file(__calibrationPointsFilename);
  file << __calibrationPointsId << endl;

  for(const GeometricCalibration::MarkerPoints3_t &mp : pointSets)
  {
    assert(mp.size() == 3);
    file << mp[0] << "; " << mp[1] << "; " << mp[2] << endl;
  }

  return file.good();
}

bool loadPoints(GeometricCalibration::PointSets_t &pointSets)
{
  pointSets.reserve(__numIterations);

  ifstream file(__calibrationPointsFilename);
  string id;
  getline(file, id);
  if(id != __calibrationPointsId)
    return false;

  while(true)
  {
    char ch;
    GeometricCalibration::MarkerPoints3_t mp;
    assert(mp.size() == 3);
    file >> mp[0] >> ch >> mp[1] >> ch >> mp[2];
    if(!file)
      break;

    pointSets.push_back(mp);
    if(pointSets.size() == __numIterations)
      return true;
  }

  return false;
}

bool loadCalibrationTriangle(CalibrationTriangle &calibrationTriangle)
{
  if(calibrationTriangle.load())
  {
    logerror << "calibration triangle could not be loaded" << endl;
    return false;
  }
  if(!calibrationTriangle.isValid())
  {
    logerror << "calibration triangle is not valid" << endl;
    return false;
  }

  return true;
}

using boost::qvm::operator+=;
using boost::qvm::operator/=;

Point3 &operator+=(Point3 &p, const Point3f &q)
{
  p += Point3(q);
  return p;
}

using RefPointSet_t = array<Point3, GeometricCalibration::numMarkers>;

RefPointSet_t calcAverageRefPointSet(const GeometricCalibration::PointSets_t &pointSets)
{
  RefPointSet_t avgPoints;
  for(const GeometricCalibration::MarkerPoints3_t &mp : pointSets)
  {
    avgPoints[0] += mp[0];
    avgPoints[1] += mp[1];
    avgPoints[2] += mp[2];
  }
  avgPoints[0] /= pointSets.size();
  avgPoints[1] /= pointSets.size();
  avgPoints[2] /= pointSets.size();

  return avgPoints;
}

} // namespace

GeometricCalibration::GeometricCalibration(const Window &window)
: _window(window)
{
  CalibrationTriangle calibrationTriangle;
  if(!loadCalibrationTriangle(calibrationTriangle))
    return;

  _lowerQuadrantIsLeft = calibrationTriangle.getLowerQuadrant() == CalibrationTriangle::Side::left;
  _pointSets.reserve(__numIterations);
  _initialized = true;
}

bool GeometricCalibration::detectPoints(const Camera::Frameset &frames)
{
  if(_initialized)
  {
    MarkerPoints2_t markerPixels;
    if(locateMarkers(frames.infraredFrame(), markerPixels))
    {
      MarkerPoints3_t markerPoints;
      if(detectPoints(frames.depthFrame(), markerPixels, markerPoints))
      {
        _pointSets.push_back(markerPoints);
        if(_pointSets.size() == __numIterations)
        {
          _initialized = false;
          if(savePoints(_pointSets))
          {
            loginfo << "calibration points saved, finished" << endl;
            return true;
          }
          logerror << "calibration points could not be saved" << endl;
        }
      }
    }
  }

  return false;
}

GeometricTransformation GeometricCalibration::load()
{
  CalibrationTriangle triangle;
  if(loadCalibrationTriangle(triangle))
  {
    PointSets_t pointSets;
    if(loadPoints(pointSets))
    {
      const RefPointSet_t &wor = triangle.getTriangleCorners();    // external world coordinates
      const RefPointSet_t cam = calcAverageRefPointSet(pointSets); // camera coordinates

      return{{ wor[0], wor[1], wor[2] },
             { cam[0], cam[1], cam[2] }};
    }
    logerror << "calibration points could not be loaded" << endl;
  }

  return{};
}

bool GeometricCalibration::locateMarkers(const Camera::VideoFrame &frame, MarkerPoints2_t &markerPixels)
{
  const int halfWidth = frame.width() / 2;
  const int halfHeight = frame.height() / 2;

  vector<Rect2i> rects[2];
  rects[0].reserve(3);
  rects[1].reserve(3);

  _window.setViewport(viewportId::infrared);

  Rect2i rect{ 0, 0, halfWidth, halfHeight };
  const bool res0 = locateMarker(rect, frame, markerPixels[0]);
  rects[res0 ? 0:1].push_back(rect);

  rect.x = halfWidth;
  const bool res1 = locateMarker(rect, frame, markerPixels[1]);
  rects[res1 ? 0:1].push_back(rect);

  rect.x = _lowerQuadrantIsLeft ? 0 : halfWidth;
  rect.y = halfHeight;
  const bool res2 = locateMarker(rect, frame, markerPixels[2]);
  rects[res2 ? 0:1].push_back(rect);

  for(const Rect2i &r : rects[0])
    drawSubframeRect(r, green);
  for(const Rect2i &r : rects[1])
    drawSubframeRect(r, red);

  return res0 && res1 && res2;
}

bool GeometricCalibration::locateMarker(const Rect2i &rect, const Camera::VideoFrame &frame, MarkerPoint2_t &markerPixel)
{
  const void *rectPixels = static_cast<const uint8_t*>(frame.pixels()) + rect.y * frame.stride()
                                                                       + rect.x * frame.bytesPerPixel();
  const Image image(rect.width, rect.height, const_cast<void*>(rectPixels), frame.stride());
  const CalibrationMark::Detections detections = CalibrationMark::detect(image);

  if(detections.size())
  {
    setDrawOffset(rect.x, rect.y);

    for(const auto &marker : detections)
    {
      using boost::qvm::operator+;
      markerPixel = marker.centroid + Point2(rect.x, rect.y);
      drawMarker(marker.centroid, marker.contour);
    }

    resetDrawOffset();

    if(detections.size() == 1)
      return true;
  }

  logerror << detections.size() << " markers found, rect " << rect.x << " " << rect.y << endl;

  return false;
}

bool GeometricCalibration::detectPoints(const Camera::DepthFrame &frame, const MarkerPoints2_t &markerPixels, MarkerPoints3_t &markerPoints)
{
  bool ret = true;
  const float zMin = 0.19f;
  const float zMax = 10.0f;

  frame.deproject(markerPixels, markerPoints);

  _window.setViewport(viewportId::depth);

  auto point = markerPoints.begin();
  for(const MarkerPoint2_t &pixel : markerPixels)
  {
    if(point->z < zMin || point->z > zMax)
      ret = false;

    drawMarker(pixel, *point++);
  }

  return ret;
}

} /* namespace stairs */
