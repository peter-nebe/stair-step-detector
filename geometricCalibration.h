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

#ifndef GEOMETRICCALIBRATION_H_
#define GEOMETRICCALIBRATION_H_

#include "camera.h"
#include "transformation.h"

namespace stairs
{

class Window;

class GeometricCalibration
{
public:
  GeometricCalibration(const Window &window);
  bool detectPoints(const Camera::Frameset &frames);
  static GeometricTransformation load();

  static const int numMarkers = 3;
  using MarkerPoint2_t = Point2;
  using MarkerPoints2_t = std::array<MarkerPoint2_t, numMarkers>;
  using MarkerPoint3_t = Point3f;
  using MarkerPoints3_t = std::array<MarkerPoint3_t, numMarkers>;
  using PointSets_t = std::vector<MarkerPoints3_t>;

private:
  bool locateMarkers(const Camera::VideoFrame &frame, MarkerPoints2_t &markerPixels);
  bool locateMarker(const Rect2i &rect, const Camera::VideoFrame &frame, MarkerPoint2_t &markerPixel);
  bool detectPoints(const Camera::DepthFrame &frame, const MarkerPoints2_t &markerPixels, MarkerPoints3_t &markerPoints);

  bool _initialized = false;
  bool _lowerQuadrantIsLeft = false;
  PointSets_t _pointSets;
  const Window &_window;
};

} /* namespace stairs */

#endif /* GEOMETRICCALIBRATION_H_ */
