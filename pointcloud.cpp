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

#include "pointcloud.h"
#include "configuration.h"
#include "transformation.h"
#include "window.h"
#include "image.h"
#include "segmentation.h"
#include "quadrilateralTest.h"
#include "stairs.h"
#include "drawing.h"
#include "qvmTraits.h"
#include "boost/qvm/vec_operations.hpp"
#include "boost/qvm/vec_mat_operations.hpp"
#include <algorithm>
#include <span>
#include <numeric>
#include <iostream>
using namespace std;

namespace stairs
{

namespace
{

using Vertices_t = vector<rs2::vertex>;
using Points3_t = vector<Point3>;
using Quadrilateral3_t = Quadrilateral_<Point3>;

using PointIndex_t = uint32_t;
using Height_t = uint16_t; // histogram index
struct PointHt
{
  PointIndex_t pointIdx;
  Height_t height;
};
using PointsHt_t = vector<PointHt>;

using NumPoints_t = uint32_t;
using Histogram_t = vector<NumPoints_t>;
using Peaks_t = vector<Height_t>;

class Projection2D
{
  const Size2i _imageSize;
  const Configuration::MeasuringRange &_worldRange;
  const Coordinate_t _xToImage,
                     _yToImage,
                     _xToWorld,
                     _yToWorld;

public:
  Projection2D(const Configuration &config)
  : _imageSize(config.streams.depth.width, config.streams.depth.height),
    _worldRange(config.measuringRange),
    _xToImage(_imageSize.width / (_worldRange.x.max - _worldRange.x.min)),
    _yToImage(_imageSize.height / (_worldRange.y.max - _worldRange.y.min)),
    _xToWorld(1 / _xToImage),
    _yToWorld(1 / _yToImage)
  {
  }
  Point2i worldToImage(const Point3 &point) const
  {
    return{ static_cast<int>((point.x - _worldRange.x.min) * _xToImage),
            static_cast<int>((_worldRange.y.max - point.y) * _yToImage) };
  }
  Point2 imageToWorld(const Point2 &point) const
  {
    return{ _worldRange.x.min + point.x * _xToWorld,
            _worldRange.y.max - point.y * _yToWorld };
  }
  const Size2i &getImageSize() const
  {
    return _imageSize;
  }
  Coordinate_t getXYRatio() const
  {
    return _xToImage / _yToImage;
  }
}; // class Projection2D

struct ProcessingConfiguration : Configuration
{
  const Coordinate_t heightIntervalReciprocal = 1.0 / heightInterval;
  const Height_t minHeight = (minHeightAboveGround - measuringRange.z.min) * heightIntervalReciprocal;
  const int minImgYExtent = minStepDepth * streams.depth.height / (measuringRange.y.max - measuringRange.y.min);
  const Projection2D projection{ *this };
};
const ProcessingConfiguration __processingConfiguration;

struct PointsTransformation
{
  static Points3_t transformToWorld(const Vertices_t &cameraPoints, const CameraToWorld &cameraToWorld)
  {
    Points3_t worldPoints;
    worldPoints.reserve(cameraPoints.size());

    ranges::transform(cameraPoints, back_inserter(worldPoints), [&cameraToWorld](const rs2::vertex &vertex)
                                                                {
                                                                  return cameraToWorld(vertex);
                                                                });
    return worldPoints;
  }

  static Quadrilateral3_t transformToCamera(const Quadrilateral3_t &worldPoints, const WorldToCamera &worldToCamera)
  {
    Quadrilateral3_t cameraPoints;
    ranges::transform(worldPoints, cameraPoints.begin(), [&worldToCamera](const Point3 &point)
                                                         {
                                                           return worldToCamera(point);
                                                         });
    return cameraPoints;
  }

  static Quadrilateral3_t transformToExternalWorld(const Quadrilateral3_t &worldPoints, const GeometricTransformation &trans)
  {
    Quadrilateral3_t extWorldPoints;
    ranges::transform(worldPoints, extWorldPoints.begin(), [&trans](const Point3 &point)
                                                           {
                                                             return trans.toExternalWorld(point);
                                                           });
    return extWorldPoints;
  }
}; // struct PointsTransformation

class PointsExtraction
{
  const rs2::pointcloud &_pointcloud;
  const Camera::DepthFrame &_frame;
  const CameraToWorld &_cameraToWorld;

public:
  PointsExtraction(const rs2::pointcloud &pointcloud, const Camera::DepthFrame &frame, const CameraToWorld &cameraToWorld)
  : _pointcloud(pointcloud),
    _frame(frame),
    _cameraToWorld(cameraToWorld)
  {
  }

  void extract(Points3_t &points, PointsHt_t &pointsHt) const
  {
    const Vertices_t cameraPoints = getNonZeroPoints();
    const Points3_t worldPoints = PointsTransformation::transformToWorld(cameraPoints, _cameraToWorld);

    const auto &config = __processingConfiguration;
    points = getPointsInRange(worldPoints, config.measuringRange);
    pointsHt = calcHeights(points, config.measuringRange.z.min, config.heightIntervalReciprocal);
  }

private:
  Vertices_t getNonZeroPoints() const
  {
    const rs2::points points = _pointcloud.calculate(_frame);

    Vertices_t vertices;
    vertices.reserve(points.size());

    ranges::copy_if(span(points.get_vertices(), points.size()), back_inserter(vertices), [](const rs2::vertex &vertex)
                                                                                         {
                                                                                           return vertex.z > 0;
                                                                                         });
    return vertices;
  }

  Points3_t getPointsInRange(const Points3_t &points, const Configuration::MeasuringRange &range) const
  {
    Points3_t pointsInRange;
    pointsInRange.reserve(points.size());

    ranges::copy_if(points, back_inserter(pointsInRange), [&range](const Point3 &p)
                                                          {
                                                            return p.x > range.x.min &&
                                                                   p.x < range.x.max &&
                                                                   p.y > range.y.min &&
                                                                   p.y < range.y.max &&
                                                                   p.z > range.z.min &&
                                                                   p.z < range.z.max;
                                                          });
    return pointsInRange;
  }

  PointsHt_t calcHeights(const Points3_t &points, Coordinate_t referenceHeight, Coordinate_t intervalReciprocal) const
  {
    PointsHt_t pointsHt;
    pointsHt.reserve(points.size());

    for(PointIndex_t pointIdx = 0; pointIdx < points.size(); pointIdx++)
      pointsHt.push_back(PointHt{
                                  pointIdx,
                                  static_cast<Height_t>((points[pointIdx].z - referenceHeight) * intervalReciprocal)
                                });
    return pointsHt;
  }
}; // class PointsExtraction

class HeightsHistogram
{
public:
  static void calcHistogram(const PointsHt_t &points, Histogram_t &hist, Peaks_t &peaks)
  {
    const auto &config = __processingConfiguration;
    const auto zrange = config.measuringRange.z;

    hist = calcHist(points, zrange.max - zrange.min, config.heightIntervalReciprocal);
    peaks = detectPeaks(hist);
  }

private:
  static Histogram_t calcHist(const PointsHt_t &points, Coordinate_t maxRelHeight, Coordinate_t intervalReciprocal)
  {
    Histogram_t hist(static_cast<size_t>(maxRelHeight * intervalReciprocal) + 1);

    // possibly use execution policy
    ranges::for_each(points, [&hist](const PointHt &point)
                             {
                               ++hist[point.height];
                             });
    return hist;
  }

  static Peaks_t detectPeaks(const Histogram_t &hist)
  {
    const Peaks_t peaks = findPeaks(hist);
    const Peaks_t filtered = filterPeaks(peaks, hist);

    return filtered;
  }

  static Peaks_t findPeaks(const Histogram_t &hist)
  {
    Peaks_t peaks;
    peaks.reserve(hist.size() / 2);

    const size_t n = hist.size() - 1;
    bool ascending = false;

    for(Height_t i = 0; i < n; i++)
    {
      const NumPoints_t curr = hist[i];
      const NumPoints_t succ = hist[i + 1];

      if(curr < succ)
      {
        ascending = true;
        continue;
      }
      if(curr > succ)
      {
        if(ascending)
          peaks.push_back(i);

        ascending = false;
      }
    }
    return peaks;
  }

  static Peaks_t filterPeaks(const Peaks_t &peaks, const Histogram_t &hist)
  {
    Peaks_t filtered;
    filtered.reserve(peaks.size());

    ranges::copy_if(peaks, back_inserter(filtered), [&hist](Height_t i)
                                                    {
                                                      const NumPoints_t numPoints = hist[i];
                                                      if(numPoints < 2000)
                                                        return false;
                                                      return (numPoints * 2 - hist[i - 1] - hist[i + 1]) * 2 > numPoints;
                                                    });
    return filtered;
  }
}; // class HeightsHistogram

struct Plateau
{
  Height_t height;
  PointsHt_t plateauPoints;
  Quadrilateral_t quadriWorld2D; // only x, y
  bool valid = false;
};
using Plateaus_t = vector<Plateau>;

class PlateausExtraction
{
  const Histogram_t &_heightsHistogram;
  const Peaks_t &_histogramPeaks;

public:
  PlateausExtraction(const Histogram_t &heightsHistogram, const Peaks_t &histogramPeaks)
  : _heightsHistogram(heightsHistogram),
    _histogramPeaks(histogramPeaks)
  {
  }

  Plateaus_t extractPlateaus(PointsHt_t &pointsHt) const
  {
    Plateaus_t plateaus;
    plateaus.reserve(_histogramPeaks.size());

    PointsHt_t remainder; // pointsHt minus points of all plateaus
    remainder.reserve(pointsHt.size());

    for(Height_t peak : _histogramPeaks)
      plateaus.emplace_back(peak, extractPlateauPoints(peak, pointsHt, remainder));

    ranges::copy(pointsHt, back_inserter(remainder));
    pointsHt.clear();

    // TODO use remainder to detect vertical faces

    return plateaus;
  }

private:
  PointsHt_t extractPlateauPoints(Height_t height, PointsHt_t &pointsHt, PointsHt_t &remainder) const
  {
    // plateau: [heightMin, heightMax] = [height - 1, height] or [height, height + 1]

    Height_t heightMin, heightMax;
    const Height_t pred = height - 1;
    const Height_t succ = height + 1;
    if(_heightsHistogram[pred] > _heightsHistogram[succ])
    {
      heightMin = pred;
      heightMax = height;
    }
    else
    {
      heightMin = height;
      heightMax = succ;
    }
    const size_t plateauSize = _heightsHistogram[heightMin] + _heightsHistogram[heightMax];

    PointsHt_t upper;
    upper.reserve(pointsHt.size());

    // remainder: < heightMin
    // upper: >= heightMin (plateau + above)
    split(pointsHt, heightMin - 1, remainder, upper);

    PointsHt_t plateauPoints;
    plateauPoints.reserve(plateauSize);
    pointsHt.clear();

    // plateauPoints: <= heightMax
    // pointsHt: > heightMax
    split(upper, heightMax, plateauPoints, pointsHt);

    return plateauPoints;
  }

  static void split(const PointsHt_t &points, Height_t heightThreshold, PointsHt_t &lower, PointsHt_t &upper)
  {
    ranges::partition_copy(points, back_inserter(upper), back_inserter(lower), [heightThreshold](const PointHt &point)
                                                                               {
                                                                                 return heightThreshold < point.height;
                                                                               });
  }
}; // class PlateausExtraction

class StairsDetector
{
  const Window &_window;
  const GeometricTransformation &_transformation;
  const Camera::DepthFrame &_frame;
  const Points3_t &_points;

public:
  StairsDetector(const Window &window, const GeometricTransformation &trans, const Camera::DepthFrame &frame, const Points3_t &points)
  : _window(window),
    _transformation(trans),
    _frame(frame),
    _points(points)
  {
  }

  Stairs detectStairs(Plateaus_t &plateaus) const
  {
    _window.setViewport(viewportId::depth);

    const StairSteps_t stairSteps = detectStairSteps(plateaus);
    for(const Quadrilateral3_t &quadri : stairSteps)
      drawStairStep(quadri);

    Stairs stairs;
    stairs.stairSteps.reserve(stairSteps.size());

    ranges::transform(stairSteps, back_inserter(stairs.stairSteps), [this](const Quadrilateral3_t &quadri)
                                                                    {
                                                                      const Quadrilateral3_t extWorld
                                                                          = PointsTransformation::transformToExternalWorld(quadri, _transformation);
                                                                      return Stairs::StairStep{ .height = extWorld[0].z,
                                                                                                .quadrilateral{ extWorld[0],
                                                                                                                extWorld[1],
                                                                                                                extWorld[2],
                                                                                                                extWorld[3] }};
                                                                    });
    _window.setViewport(viewportId::infrared);

    auto extWorld = stairs.stairSteps.begin();
    for(const Quadrilateral3_t &quadri : stairSteps)
    {
      drawStairStep(quadri, extWorld->quadrilateral, extWorld->height);
      ++extWorld;
    }

    return stairs;
  }

private:
  using StairSteps_t = vector<Quadrilateral3_t>;

  StairSteps_t detectStairSteps(Plateaus_t &plateaus) const
  {
    const auto &config = __processingConfiguration;
    size_t maxGround = 0;
    int groundInd = -1;
    int firstValidInd = -1;

    size_t i = 0;
    for( ; i < plateaus.size(); i++)
    {
      Plateau &plateau = plateaus[i];
      if(plateau.height >= config.minHeight)
        break;

      if(maxGround < plateau.plateauPoints.size())
      {
        maxGround = plateau.plateauPoints.size();
        groundInd = i;
      }
    }
    for( ; i < plateaus.size(); i++)
    {
      Plateau &plateau = plateaus[i];
      const Image plateauImage = projectToBinaryImage(plateau.plateauPoints);
      const Segmentation::Outline outline = Segmentation::detectOutline(plateauImage, config.minImgYExtent, config.projection.getXYRatio(),
                                                                        "Height " + to_string(plateau.height));
      plateau.quadriWorld2D = imgPointsToWorld(outline.quadrilateral);
      plateau.valid = outline.valid;
      if(plateau.valid && firstValidInd < 0)
        firstValidInd = i;
    }

    StairSteps_t stairSteps;
    stairSteps.reserve(plateaus.size());

    if(firstValidInd >= 0)
    {
      if(groundInd >= 0)
      {
        Plateau &ground = plateaus[groundInd];
        ground.quadriWorld2D = calcGroundQuadrilateral(plateaus[firstValidInd].quadriWorld2D);
        ground.valid = true;

        stairSteps.push_back(calcGround(ground));
      }

      for(size_t i = firstValidInd; i < plateaus.size(); i++)
      {
        const Plateau &plateau = plateaus[i];
        if(!plateau.valid)
          continue;

        stairSteps.push_back(calcStairStep(plateau));
      }
    }

    return stairSteps;
  }

  Image projectToBinaryImage(const PointsHt_t &pointsHt) const
  {
    const Projection2D &projection = __processingConfiguration.projection;
    Image image(projection.getImageSize());

    // possibly use execution policy
    ranges::for_each(pointsHt, [this, &projection, &image](const PointHt &pointHt)
                               {
                                 const Point3 &worldPoint = _points[pointHt.pointIdx];
                                 const Point2i imagePoint = projection.worldToImage(worldPoint);
                                 *image.ptr(imagePoint) = 0xff;
                               });
    return image;
  }

  template<size_t N>
  using Points2_ = array<Point2, N>;

  template<size_t N>
  static Points2_<N> imgPointsToWorld(const Points2_<N> &imgPoints)
  {
    const Projection2D &projection = __processingConfiguration.projection;
    Points2_<N> worldPoints;

    ranges::transform(imgPoints, worldPoints.begin(), [&projection](const Point2 &imgPoint)
                                                      {
                                                        return projection.imageToWorld(imgPoint);
                                                      });
    return worldPoints;
  }

  static Quadrilateral_t calcGroundQuadrilateral(const Quadrilateral_t &q)
  {
    const Coordinate_t yMin = __processingConfiguration.measuringRange.y.min;
    auto calcDx = [yMin](const Point2 &p, const Point2 &q)
                  {
                    return (q.y - yMin) * (q.y - p.y) / (q.x - p.x);
                  };
    Quadrilateral_t gq;

    if(q[0].y < q[1].y)
    {
      gq[0] = Point2(q[0].x, yMin);
      gq[1] = Point2(q[1].x + calcDx(q[0], q[1]), yMin);
    }
    else
    {
      gq[0] = Point2(q[0].x + calcDx(q[1], q[0]), yMin);
      gq[1] = Point2(q[1].x, yMin);
    }
    gq[2] = q[0];
    gq[3] = q[1];

    return gq;
  }

  class Line : LineCoordinates<double>
  {
    using Base_t = LineCoordinates<double>;
    using Base_t::Base_t;

  public:
    Point2 intersection(const Line &other) const
    {
      const double d = det(other);
      return{ detx(other) / d,
              dety(other) / d };
    }
  };

  Quadrilateral3_t calcGround(const Plateau &ground) const
  {
    const PointsHt_t pointsInQuadri = getPointsInQuadrilateral(ground.plateauPoints, ground.quadriWorld2D);
    const Image groundImage = projectToBinaryImage(pointsInQuadri);
    const Segmentation::FrontEdge frontEdge = Segmentation::detectFrontEdge(groundImage, "Ground");
    if(frontEdge.valid)
    {
      const Points2_<2> frontPoints = imgPointsToWorld<2>({ frontEdge.pointLeft, frontEdge.pointRight });
      const Line frontLine(frontPoints[0], frontPoints[1]);
      const Point2 frontLeft = frontLine.intersection({ ground.quadriWorld2D[0], ground.quadriWorld2D[2] });
      const Point2 frontRight = frontLine.intersection({ ground.quadriWorld2D[1], ground.quadriWorld2D[3] });
      const Coordinate_t averageZ = calcAverageZ(pointsInQuadri);

      return{{{ frontLeft, averageZ },
              { frontRight, averageZ },
              { ground.quadriWorld2D[2], averageZ },
              { ground.quadriWorld2D[3], averageZ }}};
    }
    return{};
  }

  Quadrilateral3_t calcStairStep(const Plateau &plateau) const
  {
    const PointsHt_t pointsInQuadri = getPointsInQuadrilateral(plateau.plateauPoints, plateau.quadriWorld2D);
    const Coordinate_t averageZ = calcAverageZ(pointsInQuadri);

    return{{{ plateau.quadriWorld2D[0], averageZ },
            { plateau.quadriWorld2D[1], averageZ },
            { plateau.quadriWorld2D[2], averageZ },
            { plateau.quadriWorld2D[3], averageZ }}};
  }

  PointsHt_t getPointsInQuadrilateral(const PointsHt_t &pointsHt, const Quadrilateral_t &quadrilateral) const
  {
    const QuadrilateralTest quadriTest(quadrilateral);
    PointsHt_t pointsInQuadri;
    pointsInQuadri.reserve(pointsHt.size());

    ranges::copy_if(pointsHt, back_inserter(pointsInQuadri), [this, &quadriTest](const PointHt &pointHt)
                                                             {
                                                               const Point3 &point = _points[pointHt.pointIdx];
                                                               return quadriTest.isPointWithin(point);
                                                             });
    return pointsInQuadri;
  }

  Coordinate_t calcAverageZ(const PointsHt_t &pointsHt) const
  {
    const Coordinate_t sum = accumulate(pointsHt.begin(), pointsHt.end(), Coordinate_t{}, [this](Coordinate_t accu, const PointHt &pointHt)
                                                                                          {
                                                                                            return accu + _points[pointHt.pointIdx].z;
                                                                                          });
    return sum / pointsHt.size();
  }

  void drawStairStep(const Quadrilateral3_t &quadriWorld) const
  {
    drawStairStep(quadriWorld, { quadriWorld[0], quadriWorld[1] }, quadriWorld[0].z);
  }

  void drawStairStep(const Quadrilateral3_t &quadriWorld, const Quadrilateral_t &labeling, Coordinate_t zLabel) const
  {
    const Quadrilateral3_t quadriCam = PointsTransformation::transformToCamera(quadriWorld, _transformation.worldToCamera());
    Quadrilateralf_t quadriProj;
    _frame.project(quadriCam, quadriProj);

    drawQuadrilateral(quadriProj, labeling, zLabel);
  }
}; // class StairsDetector

} // namespace

Pointcloud::Pointcloud(const Window &window, const GeometricTransformation &trans)
: _window(window),
  _transformation(trans)
{
}

void Pointcloud::process(const Camera::DepthFrame &frame) const
{
  Points3_t points;
  PointsHt_t pointsHt;

  const PointsExtraction pointsEx(_pointcloud, frame, _transformation.cameraToWorld());
  pointsEx.extract(points, pointsHt);

  Histogram_t heightsHistogram;
  Peaks_t histogramPeaks;
  HeightsHistogram::calcHistogram(pointsHt, heightsHistogram, histogramPeaks);

  const PlateausExtraction plateausEx(heightsHistogram, histogramPeaks);
  Plateaus_t plateaus = plateausEx.extractPlateaus(pointsHt);

  const StairsDetector detector(_window, _transformation, frame, points);
  const Stairs stairs = detector.detectStairs(plateaus);
  cout << stairs.serialize() << endl;
}

} /* namespace stairs */
