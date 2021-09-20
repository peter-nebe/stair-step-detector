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

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

namespace stairs
{

struct Configuration
{
  struct Streams
  {
    struct
    {
      const int width, height, bytesPerPixel;
      const bool active;
    }
    depth     { 640, 480, 2, true },
    infrared  { depth.width, depth.height, 1, true },
    grayscale { 960, 540, 2, false };
  } streams;

  struct MeasuringRange
  {
    struct { const double min, max; }
    x { -0.6, 0.6 },
    y {  0.1, 1.3 },
    z { -0.1, 1.1 };
  } measuringRange;

  const double heightInterval = 0.01;
  const double minHeightAboveGround = 0.05;
  const double minStepDepth = 0.1; // minimum extent in forward (y) direction
};

} /* namespace stairs */

#endif /* CONFIGURATION_H_ */
