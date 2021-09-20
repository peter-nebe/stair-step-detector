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

#ifndef IMAGE_H_
#define IMAGE_H_

#include "types.h"
#include "opencv2/core.hpp"

namespace stairs
{

class Image : public cv::Mat
{
public:
  Image()
  {
  }
  Image(const Size2i &s)
  : Image(s.width, s.height)
  {
  }
  Image(int width, int height)
  : Mat(Mat::zeros(height, width, CV_8U))
  {
  }
  Image(int width, int height, void *pixels, int step)
  : Mat(height, width, CV_8U, pixels, step)
  {
  }
  int width() const
  {
    return cols;
  }
  int height() const
  {
    return rows;
  }
  uint8_t *pixels() const
  {
    return data;
  }
  int step() const
  {
    return Mat::step;
  }
  const uint8_t *ptr() const
  {
    return Mat::ptr();
  }
  const uint8_t *ptr(int x, int y) const
  {
    return Mat::ptr(y, x);
  }
  uint8_t *ptr(int x, int y)
  {
    return Mat::ptr(y, x);
  }
  uint8_t *ptr(const Point2i &p)
  {
    return ptr(p.x, p.y);
  }
  void extractChannel(int channelIdx, Image &channel) const
  {
    cv::extractChannel(*this, channel, channelIdx);
  }
};

} /* namespace stairs */

#endif /* IMAGE_H_ */
