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

#ifndef DRAWING_H_
#define DRAWING_H_

#include "types.h"
#include <cstdint>

namespace stairs
{

struct Rgb
{
  uint8_t r, g, b;
};

const Rgb red{ 0xff, 0, 0 };
const Rgb green{ 0, 0xff, 0 };
const Rgb blue{ 0, 0, 0xff };
const Rgb white{ 0xff, 0xff, 0xff };

void setColor(Rgb color);
void drawRectangle(int x, int y, int w, int h);
void drawQuadrilateral(const int corners[4][2]);
void drawDot(int x, int y);
void drawCross(int x, int y);
void drawText(int x, int y, const char *text);
void setDrawOffset(int x, int y);
void resetDrawOffset();

void drawSubframeRect(const Rect2i &rect, Rgb color);
void drawMarker(const Point2 &center, const Contour_t &contour);
void drawMarker(const Point2 &center, const Point3f &coordinates);
void drawPoints(const Points2i_t &points);
void drawLineStrip(const Points2i_t &points);
void drawPolygon(const Points2i_t &points);
void drawRectangle(const Rect2i &rect);
void drawOctagon(const Point2f &center, float radius);
void drawQuadrilateral(const Quadrilateralf_t &corners, const Quadrilateral_t &labeling, Coordinate_t zLabel);

template<typename T>
inline int toInt(T a)
{
  return static_cast<int>(a);
}

template<typename PointType>
void drawCross(const PointType &pos)
{
  drawCross(toInt(pos.x), toInt(pos.y));
}

} /* namespace stairs */

#endif /* DRAWING_H_ */
