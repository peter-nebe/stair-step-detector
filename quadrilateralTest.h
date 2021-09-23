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

#ifndef QUADRILATERALTEST_H_
#define QUADRILATERALTEST_H_

#include "types.h"
#include <memory>
#include <functional>
#include <limits>

namespace stairs
{

class QuadrilateralTest
{
public:
  QuadrilateralTest(const Quadrilateral_t &q);
  bool isPointWithin(const Point2 &point) const;

  class Sector
  {
    Coordinate_t _lower, _upper;

  public:
    Sector(Coordinate_t a, Coordinate_t b);
    void expand(Coordinate_t c);
    bool overlaps(const Sector &other) const;
    bool isAbove(const Sector &other) const;
    bool isBelow(const Sector &other) const;
    bool isWithin(Coordinate_t c) const;
  }; // class Sector

  class BBox
  {
    Sector _x, _y;

  public:
    BBox(Coordinate_t lowerX, Coordinate_t upperX, Coordinate_t lowerY, Coordinate_t upperY);
    BBox(const Point2 &p, const Point2 &q);
    BBox(const Quadrilateral_t &q);
    void expand(const Point2 &p);
    bool overlaps(const BBox &other) const;

    enum RelativePosition
    {
      nowhere,
      xAbove,
      xBelow,
      yAbove,
      yBelow,
      relPosMax
    };
    RelativePosition getRelativePosition(const BBox &other) const;
    bool isWithin(const Point2 &p) const;
  }; // class BBox

  class Segment
  {
    const BBox _box;

  protected:
    Segment(const Point2 &p, const Point2 &q);

  public:
    static std::unique_ptr<Segment> create(const Point2 &p, const Point2 &q);
    virtual ~Segment(){}
    virtual bool isLeft(const Point2 &p) const = 0;
    const BBox &box() const { return _box; }
  }; // class Segment

private:
  const BBox _totalBox;
  const std::array<std::unique_ptr<Segment>, 4> _segments;
  const bool _insideIsLeft;

  typedef std::function<bool(const Point2 &p)> TestPoint_t;

  class CellTester
  {
  public:
    CellTester();
    void set0Segments(bool within);
    void set1Segment(const Segment &segment, bool insideIsLeft);
    void set2Segments(const Segment &segment1, const Segment &segment2, bool insideIsLeft);

    TestPoint_t isPointWithin;

  private:
    const TestPoint_t _isPointWithinTrue;
    const TestPoint_t _isPointWithinFalse;
    const TestPoint_t _isPointWithin1Seg;
    const TestPoint_t _isPointWithin2Seg;
    const Segment *_segment1 = nullptr;
    const Segment *_segment2 = nullptr;
    bool _insideIsLeft = false;
  }; // class CellTester

  class CellSelector
  {
  public:
    CellSelector();

    typedef std::function<void(CellTester&)> SetCellTester_t;

    void set1Cell(const SetCellTester_t &setCellTester1);
    void set2Cells(Coordinate_t xTrans1,
                   const SetCellTester_t &setCellTester1,
                   const SetCellTester_t &setCellTester2);
    void set3Cells(Coordinate_t xTrans1, Coordinate_t xTrans2,
                   const SetCellTester_t &setCellTester1,
                   const SetCellTester_t &setCellTester2,
                   const SetCellTester_t &setCellTester3);

    TestPoint_t select;

  private:
    const TestPoint_t _select0Trans;
    const TestPoint_t _select1Trans;
    const TestPoint_t _select2Trans;
    Coordinate_t _xTrans1 = std::numeric_limits<Coordinate_t>::signaling_NaN();
    Coordinate_t _xTrans2 = std::numeric_limits<Coordinate_t>::signaling_NaN();
    CellTester _cellTester1, _cellTester2, _cellTester3;
  }; // class CellSelector

  class RowSelector
  {
  public:
    RowSelector();

    typedef std::function<void(CellSelector&)> SetCellSelector_t;

    void set1Row(const SetCellSelector_t &setCellSelector1);
    void set2Rows(Coordinate_t yTrans1,
                  const SetCellSelector_t &setCellSelector1,
                  const SetCellSelector_t &setCellSelector2);
    void set3Rows(Coordinate_t yTrans1, Coordinate_t yTrans2,
                  const SetCellSelector_t &setCellSelector1,
                  const SetCellSelector_t &setCellSelector2,
                  const SetCellSelector_t &setCellSelector3);

    TestPoint_t select;

  private:
    const TestPoint_t _select0Trans;
    const TestPoint_t _select1Trans;
    const TestPoint_t _select2Trans;
    Coordinate_t _yTrans1 = std::numeric_limits<Coordinate_t>::signaling_NaN();
    Coordinate_t _yTrans2 = std::numeric_limits<Coordinate_t>::signaling_NaN();
    CellSelector _cellSelector1, _cellSelector2, _cellSelector3;
  } // class RowSelector
  _rowSelector;
}; // class QuadrilateralTest

} // namespace stairs

#endif // QUADRILATERALTEST_H_
