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

#include "quadrilateralTest.h"
#include <algorithm>
#include <cassert>
using namespace std;

namespace stairs
{

QuadrilateralTest::Sector::Sector(Coordinate_t a, Coordinate_t b)
: _lower(a), _upper(a)
{
  expand(b);
}

void QuadrilateralTest::Sector::expand(Coordinate_t c)
{
  if(_lower > c)
    _lower = c;
  else if(_upper < c)
    _upper = c;
}

bool QuadrilateralTest::Sector::overlaps(const Sector &other) const
{
  return _lower < other._upper && _upper > other._lower;
}

bool QuadrilateralTest::Sector::isAbove(const Sector &other) const
{
  return (_lower + _upper) / 2 < other._lower;
}

bool QuadrilateralTest::Sector::isBelow(const Sector &other) const
{
  return (_lower + _upper) / 2 > other._upper;
}

bool QuadrilateralTest::Sector::isWithin(Coordinate_t c) const
{
  return _lower < c && c < _upper;
}

QuadrilateralTest::BBox::BBox(Coordinate_t lowerX,
                              Coordinate_t upperX,
                              Coordinate_t lowerY,
                              Coordinate_t upperY)
: _x(lowerX, upperX),
  _y(lowerY, upperY)
{
}

QuadrilateralTest::BBox::BBox(const Point2 &p, const Point2 &q)
: _x(p.x, q.x),
  _y(p.y, q.y)
{
}

QuadrilateralTest::BBox::BBox(const Quadrilateral_t &q)
: BBox(q[0], q[1])
{
  expand(q[2]);
  expand(q[3]);
}

void QuadrilateralTest::BBox::expand(const Point2 &p)
{
  _x.expand(p.x);
  _y.expand(p.y);
}

bool QuadrilateralTest::BBox::overlaps(const BBox &other) const
{
  return _x.overlaps(other._x) && _y.overlaps(other._y);
}

QuadrilateralTest::BBox::RelativePosition QuadrilateralTest::BBox::getRelativePosition(const BBox &other) const
{
  if(_y.overlaps(other._y))
  {
    if(_x.isAbove(other._x))
      return xAbove;
    if(_x.isBelow(other._x))
      return xBelow;
  }
  if(_x.overlaps(other._x))
  {
    if(_y.isAbove(other._y))
      return yAbove;
    if(_y.isBelow(other._y))
      return yBelow;
  }
  return nowhere;
}

bool QuadrilateralTest::BBox::isWithin(const Point2 &p) const
{
  return _x.isWithin(p.x) && _y.isWithin(p.y);
}

namespace
{

struct Line : LineCoordinates<Coordinate_t>
{
  using Base_t = LineCoordinates<Coordinate_t>;
  using Base_t::Base_t;

  const Coordinate_t &a() const { return _a; }
  const Coordinate_t &b() const { return _b; }
  const Coordinate_t &c() const { return _c; }
};

// never parallel to the y-axis
class FlatLine
{
  Coordinate_t _a, _c;

public:
  FlatLine(const Point2 &p, const Point2 &q)
  {
    const Line l(p, q);
    _a = l.a() / l.b();
    _c = l.c() / l.b();
  }
  bool isPositive(const Point2 &p) const
  {
    return p.x * _a + p.y + _c > 0;
  }
};

// never parallel to the x-axis
class SteepLine
{
  Coordinate_t _b, _c;

public:
  SteepLine(const Point2 &p, const Point2 &q)
  {
    const Line l(p, q);
    _b = l.b() / l.a();
    _c = l.c() / l.a();
  }
  bool isPositive(const Point2 &p) const
  {
    return p.x + p.y * _b + _c > 0;
  }
};

class FlatPositiveSegment : public QuadrilateralTest::Segment
{
  const FlatLine _line;

public:
  FlatPositiveSegment(const Point2 &p, const Point2 &q)
  : Segment(p, q),
    _line(p, q)
  {
  }
  bool isLeft(const Point2 &p) const override
  {
    return _line.isPositive(p);
  }
};

class FlatNegativeSegment : public QuadrilateralTest::Segment
{
  const FlatLine _line;

public:
  FlatNegativeSegment(const Point2 &p, const Point2 &q)
  : Segment(p, q),
    _line(p, q)
  {
  }
  bool isLeft(const Point2 &p) const override
  {
    return !_line.isPositive(p);
  }
};

class SteepPositiveSegment : public QuadrilateralTest::Segment
{
  const SteepLine _line;

public:
  SteepPositiveSegment(const Point2 &p, const Point2 &q)
  : Segment(p, q),
    _line(p, q)
  {
  }
  bool isLeft(const Point2 &p) const override
  {
    return !_line.isPositive(p);
  }
};

class SteepNegativeSegment : public QuadrilateralTest::Segment
{
  const SteepLine _line;

public:
  SteepNegativeSegment(const Point2 &p, const Point2 &q)
  : Segment(p, q),
    _line(p, q)
  {
  }
  bool isLeft(const Point2 &p) const override
  {
    return _line.isPositive(p);
  }
};

} // namespace

QuadrilateralTest::Segment::Segment(const Point2 &p, const Point2 &q)
: _box(p, q)
{
}

unique_ptr<QuadrilateralTest::Segment> QuadrilateralTest::Segment::create(const Point2 &p, const Point2 &q)
{
  const Coordinate_t dx = q.x - p.x;
  const Coordinate_t dy = q.y - p.y;

  if(abs(dx) < abs(dy))
  {
    if(dy > 0)
      return make_unique<SteepPositiveSegment>(p, q);
    else
      return make_unique<SteepNegativeSegment>(p, q);
  }
  else
  {
    if(dx > 0)
      return make_unique<FlatPositiveSegment>(p, q);
    else
      return make_unique<FlatNegativeSegment>(p, q);
  }
}

/*
 * Order of the vertices and directed line segments
 *
 * vertices (constructor parameter): default order
 * line segments (class member): counterclockwise
 *
 *     2       3
 *     ----<----
 *     |   2   |
 *     Y 3   1 ^
 *     |   0   |
 *     ---->----
 *     0       1
 */
QuadrilateralTest::QuadrilateralTest(const Quadrilateral_t &q)
: _totalBox(q),
  _segments{ Segment::create(q[0], q[1]),
             Segment::create(q[1], q[3]),
             Segment::create(q[3], q[2]),
             Segment::create(q[2], q[0]) },
  _insideIsLeft(_segments[0]->isLeft(q[3]))
{
  if(_insideIsLeft != _segments[1]->isLeft(q[2]) ||
     _insideIsLeft != _segments[2]->isLeft(q[0]) ||
     _insideIsLeft != _segments[3]->isLeft(q[1]))
  {
    throw invalid_argument("Quadrilateral is not convex.");
  }

  typedef size_t SegmentId_t;
  typedef vector<SegmentId_t> SegmentIds_t;
  struct SegmentsMapCell
  {
    Coordinate_t upperX;
    SegmentIds_t segmentIds;
    bool neighborExists[BBox::relPosMax];
  };
  typedef vector<SegmentsMapCell> SegmentsMapCells_t;
  struct SegmentsMapRow
  {
    const Coordinate_t upperY;
    SegmentsMapCells_t cells;
  };
  vector<SegmentsMapRow> segmentsMap;
  segmentsMap.reserve(3);

  typedef array<Coordinate_t, 4> Coordinates4_t;
  Coordinates4_t xs{ q[0].x, q[1].x, q[2].x, q[3].x };
  Coordinates4_t ys{ q[0].y, q[1].y, q[2].y, q[3].y };
  ranges::sort(xs);
  ranges::sort(ys);
  Coordinate_t lowerY = ys.front();

  for(size_t yi = 1; yi < ys.size(); yi++)
  {
    if(lowerY < ys[yi])
    {
      segmentsMap.push_back({ ys[yi] });
      SegmentsMapRow &row = segmentsMap.back();
      row.cells.reserve(3);
      Coordinate_t lowerX = xs.front();

      for(size_t xi = 1; xi < xs.size(); xi++)
      {
        if(lowerX < xs[xi])
        {
          row.cells.push_back({ xs[xi] });
          SegmentsMapCell &cell = row.cells.back();
          cell.segmentIds.reserve(2);
          const BBox cellbox(lowerX, cell.upperX, lowerY, row.upperY);

          for(size_t si = 0; si < _segments.size(); si++)
          {
            const BBox &segmentBox = _segments[si]->box();
            if(cellbox.overlaps(segmentBox))
              cell.segmentIds.push_back(si);
            if(cell.segmentIds.empty())
              cell.neighborExists[cellbox.getRelativePosition(segmentBox)] = true;
          }
          lowerX = cell.upperX;
        }
      }
      lowerY = row.upperY;
    }
  }

  if(segmentsMap.empty())
    throw invalid_argument("Quadrilateral has no extent along Y-axis.");

  for(SegmentsMapRow &row : segmentsMap)
  {
    if(row.cells.empty())
      throw invalid_argument("Quadrilateral has no extent along X-axis.");

    for(SegmentsMapCell &cell : row.cells)
    {
      if(cell.segmentIds.size() > 2)
        throw invalid_argument("Quadrilateral's segment map has a cell with more than 2 segments.");
    }
  }

  for(SegmentsMapRow &row : segmentsMap)
  {
    for(auto cellIt = row.cells.begin(); cellIt != row.cells.end() - 1; )
    {
      const SegmentIds_t &currSegs = cellIt->segmentIds;
      const SegmentIds_t &nextSegs = (cellIt + 1)->segmentIds;

      if(currSegs.empty() && nextSegs.empty())
        throw invalid_argument("Quadrilateral's segment map has 2 empty cells next to each other.");
      if(currSegs.size() > 1 && nextSegs.size() > 1)
        throw invalid_argument("Quadrilateral's segment map has 2 double occupied cells next to each other.");

      if(currSegs == nextSegs)
        cellIt = row.cells.erase(cellIt);
      else
        ++cellIt;
    }
    assert(row.cells.size() >= 1 && row.cells.size() <= 3);
  }
  assert(segmentsMap.size() >= 1 && segmentsMap.size() <= 3);

  const auto setCellTester = [this](CellTester &cellTester, const SegmentsMapCell &cell)
  {
    switch(cell.segmentIds.size())
    {
      case 0:
        cellTester.set0Segments(cell.neighborExists[BBox::xAbove] &&
                                cell.neighborExists[BBox::xBelow] &&
                                cell.neighborExists[BBox::yAbove] &&
                                cell.neighborExists[BBox::yBelow]);
        break;
      case 1:
        cellTester.set1Segment(*_segments[cell.segmentIds[0]],
                               _insideIsLeft);
        break;
      case 2:
        cellTester.set2Segments(*_segments[cell.segmentIds[0]],
                                *_segments[cell.segmentIds[1]],
                                _insideIsLeft);
        break;
    }
  };
  using namespace std::placeholders;
  const auto setCellSelector = [&setCellTester](CellSelector &cellSelector, const SegmentsMapCells_t &cells)
  {
    switch(cells.size())
    {
      case 1:
        cellSelector.set1Cell(std::bind(setCellTester, _1, cells[0]));
        break;
      case 2:
        cellSelector.set2Cells(cells[0].upperX,
                               std::bind(setCellTester, _1, cells[0]),
                               std::bind(setCellTester, _1, cells[1]));
        break;
      case 3:
        cellSelector.set3Cells(cells[0].upperX, cells[1].upperX,
                               std::bind(setCellTester, _1, cells[0]),
                               std::bind(setCellTester, _1, cells[1]),
                               std::bind(setCellTester, _1, cells[2]));
        break;
    }
  };

  switch(segmentsMap.size())
  {
    case 1:
      _rowSelector.set1Row(std::bind(setCellSelector, _1, segmentsMap[0].cells));
      break;
    case 2:
      _rowSelector.set2Rows(segmentsMap[0].upperY,
                            std::bind(setCellSelector, _1, segmentsMap[0].cells),
                            std::bind(setCellSelector, _1, segmentsMap[1].cells));
      break;
    case 3:
      _rowSelector.set3Rows(segmentsMap[0].upperY, segmentsMap[1].upperY,
                            std::bind(setCellSelector, _1, segmentsMap[0].cells),
                            std::bind(setCellSelector, _1, segmentsMap[1].cells),
                            std::bind(setCellSelector, _1, segmentsMap[2].cells));
      break;
  }
}

bool QuadrilateralTest::isPointWithin(const Point2 &point) const
{
  if(_totalBox.isWithin(point))
    return _rowSelector.select(point);

  return false;
}

QuadrilateralTest::CellTester::CellTester()
: _isPointWithinTrue([this](const Point2&)
                     {
                       return true;
                     }),
  _isPointWithinFalse([this](const Point2&)
                      {
                        return false;
                      }),
  _isPointWithin1Seg([this](const Point2 &p)
                     {
                       return _segment1->isLeft(p) == _insideIsLeft;
                     }),
  _isPointWithin2Seg([this](const Point2 &p)
                     {
                       return _segment1->isLeft(p) == _insideIsLeft &&
                              _segment2->isLeft(p) == _insideIsLeft;
                     })
{
}

void QuadrilateralTest::CellTester::set0Segments(bool within)
{
  isPointWithin = within ? _isPointWithinTrue : _isPointWithinFalse;
}

void QuadrilateralTest::CellTester::set1Segment(const Segment &segment, bool insideIsLeft)
{
  _segment1 = &segment;
  _insideIsLeft = insideIsLeft;
  isPointWithin = _isPointWithin1Seg;
}

void QuadrilateralTest::CellTester::set2Segments(const Segment &segment1, const Segment &segment2, bool insideIsLeft)
{
  _segment1 = &segment1;
  _segment2 = &segment2;
  _insideIsLeft = insideIsLeft;
  isPointWithin = _isPointWithin2Seg;
}

QuadrilateralTest::CellSelector::CellSelector()
: _select0Trans([this](const Point2 &p)
                {
                  return _cellTester1.isPointWithin(p);
                }),
  _select1Trans([this](const Point2 &p)
                {
                  if(p.x < _xTrans1)
                    return _cellTester1.isPointWithin(p);

                  return _cellTester2.isPointWithin(p);
                }),
  _select2Trans([this](const Point2 &p)
                {
                  if(p.x < _xTrans1)
                    return _cellTester1.isPointWithin(p);
                  if(p.x < _xTrans2)
                    return _cellTester2.isPointWithin(p);

                  return _cellTester3.isPointWithin(p);
                })
{
}

void QuadrilateralTest::CellSelector::set1Cell(const SetCellTester_t &setCellTester1)
{
  setCellTester1(_cellTester1);
  select = _select0Trans;
}

void QuadrilateralTest::CellSelector::set2Cells(Coordinate_t xTrans1,
                                                const SetCellTester_t &setCellTester1,
                                                const SetCellTester_t &setCellTester2)
{
  _xTrans1 = xTrans1;
  setCellTester1(_cellTester1);
  setCellTester2(_cellTester2);
  select = _select1Trans;
}

void QuadrilateralTest::CellSelector::set3Cells(Coordinate_t xTrans1, Coordinate_t xTrans2,
                                                const SetCellTester_t &setCellTester1,
                                                const SetCellTester_t &setCellTester2,
                                                const SetCellTester_t &setCellTester3)
{
  _xTrans1 = xTrans1;
  _xTrans2 = xTrans2;
  setCellTester1(_cellTester1);
  setCellTester2(_cellTester2);
  setCellTester3(_cellTester3);
  select = _select2Trans;
}

QuadrilateralTest::RowSelector::RowSelector()
: _select0Trans([this](const Point2 &p)
                {
                  return _cellSelector1.select(p);
                }),
  _select1Trans([this](const Point2 &p)
                {
                  if(p.y < _yTrans1)
                    return _cellSelector1.select(p);

                  return _cellSelector2.select(p);
                }),
  _select2Trans([this](const Point2 &p)
                {
                  if(p.y < _yTrans1)
                    return _cellSelector1.select(p);
                  if(p.y < _yTrans2)
                    return _cellSelector2.select(p);

                  return _cellSelector3.select(p);
                })
{
}

void QuadrilateralTest::RowSelector::set1Row(const SetCellSelector_t &setCellSelector1)
{
  setCellSelector1(_cellSelector1);
  select = _select0Trans;
}

void QuadrilateralTest::RowSelector::set2Rows(Coordinate_t yTrans1,
                                              const SetCellSelector_t &setCellSelector1,
                                              const SetCellSelector_t &setCellSelector2)
{
  _yTrans1 = yTrans1;
  setCellSelector1(_cellSelector1);
  setCellSelector2(_cellSelector2);
  select = _select1Trans;
}

void QuadrilateralTest::RowSelector::set3Rows(Coordinate_t yTrans1, Coordinate_t yTrans2,
                                              const SetCellSelector_t &setCellSelector1,
                                              const SetCellSelector_t &setCellSelector2,
                                              const SetCellSelector_t &setCellSelector3)
{
  _yTrans1 = yTrans1;
  _yTrans2 = yTrans2;
  setCellSelector1(_cellSelector1);
  setCellSelector2(_cellSelector2);
  setCellSelector3(_cellSelector3);
  select = _select2Trans;
}

} /* namespace stairs */
