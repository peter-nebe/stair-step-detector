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

#ifndef TYPES_H_
#define TYPES_H_

#include <vector>
#include <array>

namespace stairs
{

template<class T>
struct Point2_
{
  T x, y;
};
using Point2i = Point2_<int>;
using Point2f = Point2_<float>;

struct Point3f
{
  float x, y, z;
};

using Coordinate_t = double;

template<int Dim>
struct Point_;
using Point2 = Point_<2>;
using Point3 = Point_<3>;

template<>
struct Point_<2>
{
  Coordinate_t x, y;

  Point_();
  Point_(Coordinate_t x, Coordinate_t y);
  Point_(const Point3 &p);
  bool operator!=(const Point2 &other) const;
};

template<>
struct Point_<3>
{
  Coordinate_t x, y, z;

  Point_();
  Point_(Coordinate_t x, Coordinate_t y, Coordinate_t z);
  Point_(const Point3f &p);
  Point_(const Point2 &p, Coordinate_t z);
};

inline Point2::Point_()
: Point_(0, 0) {}

inline Point2::Point_(Coordinate_t _x, Coordinate_t _y)
: x(_x), y(_y) {}

inline Point2::Point_(const Point3 &p)
: Point_(p.x, p.y) {}

inline bool Point2::operator!=(const Point2 &other) const
{
  return x != other.x
      || y != other.y;
}

inline Point3::Point_()
: Point_(0, 0, 0) {}

inline Point3::Point_(Coordinate_t _x, Coordinate_t _y, Coordinate_t _z)
: x(_x), y(_y), z(_z) {}

inline Point3::Point_(const Point3f &p)
: Point_(p.x, p.y, p.z) {}

inline Point3::Point_(const Point2 &p, Coordinate_t _z)
: Point_(p.x, p.y, _z) {}

struct Rect2i
{
  int x, y, width, height;
};

struct Size2i
{
  int width, height;
};

using Points2i_t = std::vector<Point2i>;
using Contour_t = Points2i_t;

template<typename PointType>
using Quadrilateral_ = std::array<PointType, 4>;
using Quadrilateral_t = Quadrilateral_<Point2>;
using Quadrilateralf_t = Quadrilateral_<Point2f>;

template<typename CoordinateType>
class LineCoordinates
{
protected:
  const CoordinateType _a, _b, _c;

  LineCoordinates(CoordinateType a,
                  CoordinateType b,
                  CoordinateType c)
  : _a(a),
    _b(b),
    _c(c)
  {
  }
  CoordinateType det(const LineCoordinates<CoordinateType> &other) const
  {
    return _a * other._b - other._a * _b;
  }
  CoordinateType detx(const LineCoordinates<CoordinateType> &other) const
  {
    return _b * other._c - other._b * _c;
  }
  CoordinateType dety(const LineCoordinates<CoordinateType> &other) const
  {
    return other._a * _c - _a * other._c;
  }

private:
  LineCoordinates(CoordinateType x1,
                  CoordinateType y1,
                  CoordinateType x2,
                  CoordinateType y2)
  : LineCoordinates(y2 - y1,
                    x1 - x2,
                    x2 * y1 - x1 * y2)
  {
  }

public:
  template<typename PointType>
  LineCoordinates(const PointType &p,
                  const PointType &q)
  : LineCoordinates(p.x, p.y,
                    q.x, q.y)
  {
  }
}; // class LineCoordinates

} // namespace stairs

#endif // TYPES_H_
