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

#ifndef CALIBRATIONMARK_H_
#define CALIBRATIONMARK_H_

#include "types.h"

namespace stairs
{

class Image;

class CalibrationMark
{
public:
  struct Marker
  {
    Point2 centroid;
    Contour_t contour;
  };
  using Detections = std::vector<Marker>;

  static Detections detect(const Image &image);
};

} /* namespace stairs */

#endif /* CALIBRATIONMARK_H_ */
