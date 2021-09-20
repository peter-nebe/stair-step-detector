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

#include "segmentation.h"
#include "image.h"
#include "drawing.h" // Rgb
#include "qvmTraits.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "boost/qvm/vec_operations.hpp"
#include <numbers>
using namespace std;
using namespace cv;

#define DEBUG_CV 0

namespace stairs
{

namespace
{

struct Scan
{
  int x, yFirst, ySecond;
};
typedef vector<Scan> Scans_t;

class Scanner
{
  const Image &_image;
  const int _minImgYExtent;

public:
  Scanner(const Image &img, int minImgYExtent)
  : _image(img),
    _minImgYExtent(minImgYExtent)
  {
  }

  Scans_t scan(int xStart, int xStep) const
  {
    assert(xStart >= 0 && xStart < _image.width());
    assert(xStep);

    const size_t max = xStep > 0 ? _image.width() - xStart - 1 : xStart;
    Scans_t scans;
    scans.reserve(max / abs(xStep) + 1);

    int x = xStart;
    int yFirst, ySecond;
    while(probeVertical(x, yFirst, ySecond))
    {
      if(ySecond - yFirst < _minImgYExtent)
        break;

      scans.emplace_back(x, yFirst, ySecond);

      x += xStep;
      if(x < 0 || x >= _image.width())
        break;
    }
    return scans;
  }

private:
  bool probeVertical(int x, int &yFirst, int &ySecond) const
  {
    const uint8_t *const colPtr = _image.ptr() + x;
    const uint8_t *ptr = colPtr;

    for(int yf = 0; yf < _image.height(); yf++)
    {
      if(*ptr)
      {
        yFirst = yf;
        ptr = colPtr + (_image.height() - 1) * _image.step();

        for(int ys = _image.height() - 1; ys >= yf; ys--)
        {
          if(*ptr)
          {
            ySecond = ys;
            return true;
          }
          ptr -= _image.step();
        }
        return false;
      }
      ptr += _image.step();
    }
    return false;
  }

public:
  struct PointInserter
  {
    Points2i_t &_front, &_back;
    void reserve(size_t n)
    {
      _front.reserve(n);
      _back.reserve(n);
    }
    void pushBack(const Scan &s)
    {
      _front.emplace_back(s.x, s.ySecond);
      _back.emplace_back(s.x, s.yFirst);
    }
  };

  static void obtainLinePoints(const Scans_t &scansLeft, const Scans_t &scansRight, PointInserter pointsLeft, PointInserter pointsRight)
  {
    const size_t scansTotal = scansLeft.size() + scansRight.size();
    const size_t half = scansTotal / 2 + 1;
    pointsLeft.reserve(half);
    pointsRight.reserve(half);
    size_t indLeft = 0, indRight = 0;

    if(scansLeft.size() >= half)
    {
      indLeft = scansLeft.size() - half;
      for(int i = indLeft; i >= 0; i--)
        pointsRight.pushBack(scansLeft[i]);
    }
    else
    {
      if(scansRight.size() > half)
        indRight = scansRight.size() - half;
      for(int i = indRight; i >= 0; i--)
        pointsLeft.pushBack(scansRight[i]);
    }

    for( ; indRight < scansRight.size(); indRight++)
      pointsRight.pushBack(scansRight[indRight]);

    for( ; indLeft < scansLeft.size(); indLeft++)
      pointsLeft.pushBack(scansLeft[indLeft]);
  }
}; // class Scanner

class BottomScanner
{
  const Image &_image;

public:
  BottomScanner(const Image &img)
  : _image(img)
  {
  }

  Points2i_t scan(int xStart, int xStep) const
  {
    assert(xStart >= 0 && xStart < _image.width());
    assert(xStep);

    Points2i_t points;
    points.reserve(_image.width() / xStep + 1);

    int x = xStart, y;
    do
    {
      if(probeBottomUp(x, y))
      {
        points.emplace_back(x, y);
        break;
      }
      x += xStep;
    }
    while(x < _image.width());

    if(points.empty())
    {
      x = xStart - xStep;
      do
      {
        if(probeBottomUp(x, y))
        {
          points.emplace_back(x, y);
          break;
        }
        x -= xStep;
      }
      while(x >= 0);

      if(points.empty())
        return points;
    }

    xStart = x;
    x += xStep;
    while(x < _image.width() && probeBottomUp(x, y))
    {
      points.emplace_back(x, y);
      x += xStep;
    }

    x = xStart - xStep;
    while(x >= 0 && probeBottomUp(x, y))
    {
      points.emplace_back(x, y);
      x -= xStep;
    }
    return points;
  }

private:
  bool probeBottomUp(int x, int &yEdge) const
  {
    const int yStop = _image.height() / 2;
    const uint8_t *ptr = _image.ptr() + x + (_image.height() - 1) * _image.step();

    for(int y = _image.height() - 1; y > yStop; y--)
    {
      if(*ptr)
      {
        yEdge = y;
        return true;
      }
      ptr -= _image.step();
    }
    return false;
  }
}; // class BottomScanner

class VerticalEdgePointsDetector
{
  const Image &_image;
  const int _x0, _y0, _length, _yEnd, _yStep;

public:
  VerticalEdgePointsDetector(const Image &img, int x0, int y0, int length, int yEnd, int yStep)
  : _image(img), _x0(x0), _y0(y0), _length(length), _yEnd(yEnd), _yStep(yStep)
  {
    assert(_x0 >= 0 && _x0 < _image.width());
    assert(_y0 >= 0 && _y0 < _image.height());
    assert(_length > 0 && _length <= _image.width());
    assert(_yEnd >= 0 && _yEnd <= _y0);
    assert(_yStep > 0);
  }
  Points2i_t detectLeftEdge() const
  {
    assert(_x0 + _length <= _image.width());
    return detectVerticalEdge(probeToRight);
  }
  Points2i_t detectRightEdge() const
  {
    assert(_x0 - _length >= -1);
    return detectVerticalEdge(probeToLeft);
  }

private:
  using Probe_t = function<bool(const uint8_t*, int&)>;

  Points2i_t detectVerticalEdge(const Probe_t &probeHorizontal) const
  {
    Points2i_t points;
    const int n = (_y0 - _yEnd) / _yStep + 1;
    assert(n > 0);
    points.reserve(n);

    const uint8_t *ptr = _image.ptr(_x0, _y0);
    const int step = _image.step() * _yStep;
    int px;
    for(int y = _y0; y >= _yEnd; y -= _yStep, ptr -= step)
      if(probeHorizontal(ptr, px))
        points.emplace_back(px, y);

    return points;
  }
  Probe_t probeToRight = [this](const uint8_t *ptr, int &x)
  {
    for(int i = _length; i > 0; i--)
    {
      if(*ptr++)
      {
        x = _x0 + _length - i;
        return true;
      }
    }
    return false;
  };
  Probe_t probeToLeft = [this](const uint8_t *ptr, int &x)
  {
    for(int i = _length; i > 0; i--)
    {
      if(*ptr--)
      {
        x = _x0 - _length + i;
        return true;
      }
    }
    return false;
  };
}; // class VerticalEdgePointsDetector

constexpr double __tan30 = numbers::sqrt3 / 3;
constexpr double __tan60 = numbers::sqrt3;

template<typename CoordinateType> class Line;
using Linei_t = Line<int>;
using Lined_t = Line<double>;

template<typename CoordinateType>
class Line : LineCoordinates<CoordinateType>
{
  using Base_t = LineCoordinates<CoordinateType>;
  using Base_t::Base_t;
  using Base_t::_a;
  using Base_t::_b;
  using Base_t::_c;
  friend class Line<int>;
  friend class Line<double>;

public:
  template<typename SrcCoordinateType>
  Line(const Line<SrcCoordinateType> &srcLine);

  using Coordinates_t = array<CoordinateType, 3>;
  Coordinates_t getCoordinates() const
  {
    return{ _a, _b, _c };
  }
  template<typename PointType>
  CoordinateType comparableDistance(const PointType &p) const
  {
    return comparableDistance(p.x, p.y);
  }
  double hypot() const
  {
    return std::hypot(_a, _b);
  }
  optional<Point2> intersection(const Line<CoordinateType> &other) const
  {
    // nearly perpendicular? -> abs(angle) > 60°
    const CoordinateType numerator = this->det(other);
    const CoordinateType denominator = _a * other._a + _b * other._b;
    if(abs(numerator) > abs(denominator) * __tan60)
    {
      const double det = numerator;
      return Point2(this->detx(other) / det,
                    this->dety(other) / det);
    }
    return nullopt;
  }
  Line<CoordinateType> reverse() const
  {
    return{ -_a, -_b, -_c };
  }
  template<typename PointType>
  Line<CoordinateType> parallel(const PointType &p) const
  {
    return{ _a, _b, -_a * p.x - _b * p.y };
  }
  template<typename PointType>
  Line<CoordinateType> perpendicular(const PointType &p) const
  {
    return{ -_b, _a, _b * p.x - _a * p.y };
  }
  Lined_t slopeCorrection(double correctionFactor) const
  {
    return{ _a * correctionFactor, _b, _c };
  }
  Lined_t normalized() const
  {
    const double h = hypot();
    return{ _a / h, _b / h, _c / h };
  }
  Lined_t angleBisector(const Line<CoordinateType> &other) const
  {
    const Lined_t n = normalized();
    const Lined_t o = other.normalized();

    return{ n._a + o._a,
            n._b + o._b,
            n._c + o._c };
  }

private:
  CoordinateType comparableDistance(CoordinateType x, CoordinateType y) const
  {
    return abs(x * _a + y * _b + _c);
  }
}; // class Line

template<> template<>
Line<double>::Line(const Line<int> &srcLine)
: Base_t(srcLine._a, srcLine._b, srcLine._c)
{
}

class ApproximationLine : public Linei_t
{
  double _residual = 0;

public:
  using Base_t = Linei_t;
  using PointsIter_t = Points2i_t::const_iterator;

  ApproximationLine(const PointsIter_t &p, const PointsIter_t &q, const Points2i_t &points)
  : Base_t(*p, *q)
  {
    if(points.size() <= 2)
      return;

    vector<int> dists;
    dists.reserve(points.size() - 2);

    // skip p and q
    for(PointsIter_t pi = points.begin(); pi != p; ++pi)
      dists.push_back(comparableDistance(*pi));
    for(PointsIter_t pi = p + 1; pi != q; ++pi)
      dists.push_back(comparableDistance(*pi));
    for(PointsIter_t pi = q + 1; pi != points.end(); ++pi)
      dists.push_back(comparableDistance(*pi));

    int sum = 0;
    const size_t n = dists.size() > 4 ? (dists.size() - 1) / 2 : 1;
    for(size_t i = n; i > 0; i--)
    {
      const auto min = ranges::min_element(dists);
      sum += *min;
      dists.erase(min);
    }
    _residual = sum / (n * hypot());
  }
  double getResidual() const
  {
    return _residual;
  }
}; // class ApproximationLine

class BestLine : public ApproximationLine::Base_t
{
  using Base_t = ApproximationLine::Base_t;

public:
  BestLine(const Points2i_t &points)
  : Base_t(findBestLine(points))
  {
  }

private:
  static Base_t findBestLine(const Points2i_t &points)
  {
    assert(points.size() >= 2);

    // calculate all lines
    vector<ApproximationLine> lines;
    lines.reserve(binomCoeff2(points.size()));

    for(auto p = points.begin(); p != points.end() - 1; ++p)
      for(auto q = p + 1; q != points.end(); ++q)
        lines.emplace_back(p, q, points);

    const auto bestLine = ranges::min_element(lines, [](const ApproximationLine &lhs, const ApproximationLine &rhs)
                                                     {
                                                       return lhs.getResidual() < rhs.getResidual();
                                                     });
    return *bestLine;
  }

  static size_t binomCoeff2(size_t n)
  {
    if(n <= 2)
      return 1;
    --n;
    return n + binomCoeff2(n);
  }
}; // class BestLine

// never parallel to the y-axis
class FlatLine
{
  const double _m, _n;
  using SrcLine_t = Linei_t;

public:
  FlatLine(const SrcLine_t &line)
  : FlatLine(line.getCoordinates())
  {
  }
  Point2 calcPoint(double x) const
  {
    return{ x, y(x) };
  }

private:
  FlatLine(const SrcLine_t::Coordinates_t &co)
  : FlatLine(co[0], co[1], co[2])
  {
  }
  FlatLine(int a, int b, int c)
  : _m(double(-a) / b),
    _n(double(-c) / b)
  {
  }
  double y(double x) const
  {
    return x * _m + _n;
  }
}; // class FlatLine

struct BoundaryPoints
{
  const Point2 iniPoint{ -1, -1 };
  Point2 inner = iniPoint;
  Point2 outer = iniPoint;

  BoundaryPoints(const Points2i_t &points, const FlatLine &line)
  {
    const int distanceLimit = 10;
    auto calcBound = [&line, distanceLimit](const Point2i &pi, Point2 &bound)
                     {
                       const Point2 pd = line.calcPoint(pi.x);
                       if(abs(pd.y - pi.y) < distanceLimit)
                       {
                         bound = pd;
                         return true;
                       }
                       return false;
                     };

    for(auto pi = points.begin(); pi != points.end(); ++pi)
      if(calcBound(*pi, inner))
        break;

    for(auto pi = points.rbegin(); pi != points.rend(); ++pi)
      if(calcBound(*pi, outer))
        break;

    assert(inner != iniPoint);
    assert(outer != iniPoint);
  }
}; // struct BoundaryPoints

// horizontal: x-direction in the 2D image
class HorizontalEdges
{
  struct LinePoints
  {
    Points2i_t frontLeft, frontRight, backLeft, backRight;

    LinePoints(const Scans_t &scansLeft, const Scans_t &scansRight)
    {
      Scanner::obtainLinePoints(scansLeft, scansRight,
                                { frontLeft, backLeft },
                                { frontRight, backRight });
    }
  };
  const LinePoints _points;

public:
  struct Edge
  {
    const Points2i_t &points;
    const Linei_t line;
    const BoundaryPoints bounds;

    Edge(const Points2i_t &pnts)
    : points(pnts),
      line(BestLine(points)),
      bounds(points, line)
    {
    }
    Edge(const Edge&) = delete;
  }; // struct Edge

  // The front edge of an object appears below the back edge in the 2D image.
  const Edge frontLeft;
  const Edge frontRight;
  const Edge backLeft;
  const Edge backRight;

  HorizontalEdges(const Scans_t &scansLeft, const Scans_t &scansRight)
  : _points(scansLeft, scansRight),
    frontLeft(_points.frontLeft),
    frontRight(_points.frontRight),
    backLeft(_points.backLeft),
    backRight(_points.backRight)
  {
  }
}; // class HorizontalEdges

class HorizontalEdgesDetector
{
public:
  static const int xStep = 25;

  static optional<HorizontalEdges> detect(const Image &image, int minImgYExtent)
  {
    const int xCenter = image.width() / 2;
    const Scanner scanner(image, minImgYExtent);
    const Scans_t scansRight = scanner.scan(xCenter, xStep);
    if(!scansRight.empty())
    {
      const Scans_t scansLeft = scanner.scan(xCenter - xStep, -xStep);
      if(scansLeft.size() + scansRight.size() >= 3)
        return make_optional<HorizontalEdges>(scansLeft, scansRight);
    }

    return nullopt;
  }
}; // class HorizontalEdgesDetector

// vertical: y-direction in the 2D image
struct VerticalEdges
{
  struct Edge
  {
    const Points2i_t points;
    const Lined_t line;
  };
  optional<Edge> left;
  optional<Edge> right;
}; // struct VerticalEdges

class VerticalEdgesDetector
{
  const Image &_image;
  const HorizontalEdges &_hEdges;
  const Lined_t _baseLine;
  const int _xExtension;
  static const int _yStep = 10;

public:
  VerticalEdgesDetector(const Image &image, const HorizontalEdges &hEdges, double xyRatio, int xExtension)
  : _image(image),
    _hEdges(hEdges),
    _baseLine(calcBaseLine(xyRatio)),
    _xExtension(xExtension)
  {
    assert(_xExtension > 0);
  }

  VerticalEdges detect() const
  {
    const VerticalEdges vEdges
    {
      detectEdge(_hEdges.frontLeft.bounds.outer, _hEdges.backLeft.bounds.outer,
                 [this](int left, int right, int yStart, int yEnd)
                 {
                   return VerticalEdgePointsDetector(_image, left, yStart, right - left, yEnd, _yStep).detectLeftEdge();
                 }),
      detectEdge(_hEdges.frontRight.bounds.outer, _hEdges.backRight.bounds.outer,
                 [this](int left, int right, int yStart, int yEnd)
                 {
                   return VerticalEdgePointsDetector(_image, right, yStart, right - left, yEnd, _yStep).detectRightEdge();
                 })
    };
    return vEdges;
  }

private:
  Lined_t calcBaseLine(double xyRatio)
  {
    const Lined_t front = _hEdges.frontLeft.line.reverse().angleBisector(_hEdges.frontRight.line);
    const Lined_t back = _hEdges.backLeft.line.reverse().angleBisector(_hEdges.backRight.line);
    const Lined_t center = front.angleBisector(back);

    return center.slopeCorrection(xyRatio * xyRatio).perpendicular(_hEdges.frontLeft.points.front());
  }

  optional<VerticalEdges::Edge> detectEdge(const Point2 &front, const Point2 &back, const function<Points2i_t(int, int, int, int)> &detectEdgePoints) const
  {
    int left = min(front.x, back.x) - _xExtension;
    int right = max(front.x, back.x) + _xExtension;
    int yStart = front.y - _yStep;
    int yEnd = back.y + _yStep;

    if(left < 0)
      left = 0;
    if(right >= _image.width())
      right = _image.width() - 1;
    if(yStart >= _image.height())
      yStart = _image.height() - 1;
    if(yEnd < 0)
      yEnd = 0;
    if(yStart < yEnd)
      return nullopt;

    Points2i_t points = detectEdgePoints(left, right, yStart, yEnd);
    if(points.empty())
      return nullopt;

    const Point2i bestPoint = findBestPoint(points);

    return make_optional<VerticalEdges::Edge>(move(points), _baseLine.parallel(bestPoint));
  }

  Point2i findBestPoint(const Points2i_t &points) const
  {
    assert(points.size());

    struct PointDist
    {
      const Point2i *point;
      double dist;
    };
    vector<PointDist> dists;
    dists.reserve(points.size());

    ranges::transform(points, back_inserter(dists), [this](const Point2i &p)
                                                    {
                                                      return PointDist{ &p, _baseLine.comparableDistance(p) };
                                                    });
    ranges::sort(dists, {}, &PointDist::dist);
    const size_t best = 2 * dists.size() / 3;

    return *dists[best].point;
  }
}; // class VerticalEdgesDetector

struct Corners
{
  optional<Point2> frontLeft;
  optional<Point2> frontRight;
  optional<Point2> backLeft;
  optional<Point2> backRight;

  Corners(const HorizontalEdges &hEdges, const VerticalEdges &vEdges)
  {
    if(vEdges.left)
    {
      frontLeft = vEdges.left->line.intersection(hEdges.frontLeft.line);
      backLeft = vEdges.left->line.intersection(hEdges.backLeft.line);
    }
    if(vEdges.right)
    {
      frontRight = vEdges.right->line.intersection(hEdges.frontRight.line);
      backRight = vEdges.right->line.intersection(hEdges.backRight.line);
    }
  }
}; // struct Corners

using boost::qvm::operator-;

class Quadrilateral
{
public:
  static bool isConvex(const Quadrilateral_t &q)
  {
    const Vector v[]
    {
      { q[0], q[1] },
      { q[1], q[3] }, // vertices counterclockwise
      { q[3], q[2] },
      { q[2], q[0] }
    };
    const bool positive = v[0].isPositive(v[1]);

    return positive == v[1].isPositive(v[2])
        && positive == v[2].isPositive(v[3])
        && positive == v[3].isPositive(v[0]);
  }

private:
  struct Vector : Point2
  {
    Vector(const Point2 &p, const Point2 &q)
    : Point2(q - p)
    {
    }
    // z-component of cross product
    bool isPositive(const Vector &other) const
    {
      return x * other.y - other.x * y > 0;
    }
  };
}; // class Quadrilateral

#if DEBUG_CV
namespace cvc // OpenCV colors
{
  const uint8_t X = 0xff;
  const Rgb red     { 0, 0, X };
  const Rgb green   { 0, X, 0 };
  const Rgb blue    { X, 0, 0 };
  const Rgb yellow  { 0, X, X };
  const Rgb cyan    { X, X, 0 };
  const Rgb magenta { X, 0, X };
}
namespace cvRgb
{
  Scalar toCvRgb(Rgb c) { return Scalar(c.r, c.g, c.b); }
  const Scalar red     = toCvRgb(cvc::red);
  const Scalar green   = toCvRgb(cvc::green);
  const Scalar blue    = toCvRgb(cvc::blue);
  const Scalar yellow  = toCvRgb(cvc::yellow);
  const Scalar cyan    = toCvRgb(cvc::cyan);
  const Scalar magenta = toCvRgb(cvc::magenta);
}

class Drawing : public Mat
{
public:
  Drawing(Size size)
  : Mat(size, CV_8UC3)
  {
  }
  void drawPoints(const Points2i_t &points, Rgb color = cvc::red)
  {
    for(const Point2i &p : points)
      at(p) = color;
  }
  void drawMarker(const Point2 &pos, const Scalar &color)
  {
    cv::drawMarker(*this, CvPoint2d(pos), color);
  }
  void drawLine(const Point2 &p1, const Point2 &p2, const Scalar &color)
  {
    cv::line(*this, CvPoint2d(p1), CvPoint2d(p2), color);
  }
  void drawQuadrilateral(const Quadrilateral_t &q, const Scalar &color)
  {
    const CvPoint2d p[]{ q[0], q[1], q[3], q[2] }; // vertices counterclockwise
    cv::polylines(*this, to_array<cv::Point2i>({ p[0], p[1], p[2], p[3] }), true, color);
  }
  void drawHorizontalEdge(const HorizontalEdges::Edge &left, const HorizontalEdges::Edge &right)
  {
    drawLine(left.bounds.inner, left.bounds.outer, cvRgb::cyan);
    drawLine(right.bounds.inner, right.bounds.outer, cvRgb::yellow);
    drawPoints(left.points, right.points);
  }
  void drawVerticalEdge(const VerticalEdges::Edge &edge)
  {
    drawVLine(edge.line, edge.points.back().y, edge.points.front().y, cvRgb::yellow);
    drawPoints(edge.points, cvc::magenta);
  }

private:
  struct CvPoint2d : cv::Point2d
  {
    CvPoint2d(const Point2 &p)
    : cv::Point2d(p.x, p.y)
    {
    }
  };
  Rgb &at(const Point2i &p)
  {
    return Mat::at<Rgb>(p.y, p.x);
  }
  void drawPoints(const Points2i_t &left, const Points2i_t &right)
  {
    drawPoints(left);
    drawPoints(right);
    at(right.front()) = cvc::magenta;
  }
  void drawVLine(const Lined_t &line, int top, int bottom, const Scalar &color)
  {
    const int xMax = cols - 1;
    const optional<Point2> topPt = line.intersection({ Point2(0, top), Point2(xMax, top) });
    const optional<Point2> bottomPt = line.intersection({ Point2(0, bottom), Point2(xMax, bottom) });
    if(topPt && bottomPt)
      drawLine(*topPt, *bottomPt, color);
  }
}; // class Drawing
#endif // DEBUG_CV

} // namespace

Segmentation::FrontEdge Segmentation::detectFrontEdge(const Image &image, const string &windowName)
{
  FrontEdge frontEdge;

# if DEBUG_CV
  Drawing drawing(image.size());
  cvtColor(image, drawing, COLOR_GRAY2RGB);
# endif

  morphologyEx(image, image, MORPH_CLOSE, Mat());

  const int xStep = 50;
  const int xCenter = image.width() / 2;
  const BottomScanner scanner(image);
  const Points2i_t points = scanner.scan(xCenter, xStep);
  if(points.size() >= 2)
  {
    const FlatLine edge = BestLine(points);
    const auto [left, right] = ranges::minmax(points, [](const Point2i &p, const Point2i &q)
                                                      {
                                                        return p.x < q.x;
                                                      });
    frontEdge.pointLeft = edge.calcPoint(left.x);
    frontEdge.pointRight = edge.calcPoint(right.x);
    frontEdge.valid = true;

#   if DEBUG_CV
    drawing.drawLine(frontEdge.pointLeft, frontEdge.pointRight, cvRgb::blue);
    drawing.drawPoints(points);
#   endif
  }

# if DEBUG_CV
  imshow(windowName, drawing);
  waitKey(1);
# endif

  return frontEdge;
}

Segmentation::Outline Segmentation::detectOutline(const Image &image, int minImgYExtent, double xyRatio, const string &windowName)
{
  Outline outline;

# if DEBUG_CV
  Drawing drawing(image.size());
  cvtColor(image, drawing, COLOR_GRAY2RGB);
# endif

  morphologyEx(image, image, MORPH_CLOSE, Mat());

  const optional<HorizontalEdges> optHorizontalEdges = HorizontalEdgesDetector::detect(image, minImgYExtent);
  if(optHorizontalEdges)
  {
    const HorizontalEdges &hEdges = *optHorizontalEdges;

    const VerticalEdgesDetector vertEdgesDetector(image, hEdges, xyRatio, HorizontalEdgesDetector::xStep);
    const VerticalEdges vEdges = vertEdgesDetector.detect();
    const Corners corners(hEdges, vEdges);

    outline.quadrilateral =
      {
        corners.frontLeft.value_or(hEdges.frontLeft.bounds.outer),
        corners.frontRight.value_or(hEdges.frontRight.bounds.outer),
        corners.backLeft.value_or(hEdges.backLeft.bounds.outer),
        corners.backRight.value_or(hEdges.backRight.bounds.outer)
      };
    outline.valid = Quadrilateral::isConvex(outline.quadrilateral);

#   if DEBUG_CV
    drawing.drawMarker(hEdges.frontLeft.bounds.outer, cvRgb::green);
    drawing.drawMarker(hEdges.frontRight.bounds.outer, cvRgb::green);
    drawing.drawMarker(hEdges.backLeft.bounds.outer, cvRgb::green);
    drawing.drawMarker(hEdges.backRight.bounds.outer, cvRgb::green);

    drawing.drawQuadrilateral(outline.quadrilateral, cvRgb::blue);

    drawing.drawHorizontalEdge(hEdges.frontLeft, hEdges.frontRight);
    drawing.drawHorizontalEdge(hEdges.backLeft, hEdges.backRight);
    if(vEdges.left)
      drawing.drawVerticalEdge(*vEdges.left);
    if(vEdges.right)
      drawing.drawVerticalEdge(*vEdges.right);
#   endif
  }

# if DEBUG_CV
  imshow(windowName, drawing);
  waitKey(1);
# endif

  return outline;
}

} /* namespace stairs */
