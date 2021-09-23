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

#ifndef TRANSFORMATION_H_
#define TRANSFORMATION_H_

#include "types.h" // Coordinate_t, Point_
#include "boost/qvm/mat.hpp"
#include "boost/qvm/mat_operations.hpp"
#include "boost/qvm/vec_mat_operations.hpp"
#include "boost/qvm/vec_operations.hpp"

namespace stairs
{

template<int Dim>
using Vector_ = Point_<Dim>;

template<int Dim>
using Matrix_ = boost::qvm::mat<Coordinate_t, Dim, Dim>;

template<int Dim>
using ReferencePoints_ = std::array<Point_<Dim>, Dim>;

template<int Dim>
class Transformation_
{
public:
  using RefPoints = ReferencePoints_<Dim>;
  using Point = Point_<Dim>;
  using Mat = Matrix_<Dim>;
  using Vec = Vector_<Dim>;

  Transformation_()
  : _a(boost::qvm::identity_mat<Coordinate_t, Dim>()),
    _aInv(_a)
  {
  }
  Transformation_(const RefPoints &rp, const RefPoints &rpMapping);
  Transformation_(const RefPoints &triangleInPlane);

  template<typename SrcPointType>
  Point transform(const SrcPointType &x) const
  {
    using boost::qvm::operator+;
    return _a * x + _b;
  }

  Point transformInv(const Point &x) const
  {
    return _aInv * (x - _b);
  }

private:
  Mat _a, _aInv;
  Vec _b;
}; // class Transformation_

using Transformation = Transformation_<3>;
using Transformation2D = Transformation_<2>;

struct CameraToWorld
{
  template<typename SrcPointType>
  Point3 operator()(const SrcPointType &p) const
  {
    return _camera.transform(p);
  }
  const Transformation &_camera;
};

struct WorldToCamera
{
  Point3 operator()(const Point3 &p) const;
  const Transformation &_camera;
};

struct ToExternalWorld
{
  Point3 operator()(const Point3 &p) const;
  const Transformation2D _world;
  const Coordinate_t _worldZ = 0;
};

class GeometricTransformation
{
public:
  using RefPoints = Transformation::RefPoints;

  GeometricTransformation(){}
  GeometricTransformation(const RefPoints &worldPoints, const RefPoints &cameraPoints);
  const CameraToWorld &cameraToWorld() const { return _cameraToWorld; }
  const WorldToCamera &worldToCamera() const { return _worldToCamera; }
  const ToExternalWorld &toExternalWorld() const { return _toExternalWorld; }

private:
  GeometricTransformation(const GeometricTransformation&) = delete;

  // transformation between camera coordinates and camera-dependent world coordinates
  const Transformation _camera;
  const CameraToWorld _cameraToWorld{ _camera };
  const WorldToCamera _worldToCamera{ _camera };

  // transformation into external world coordinates
  const ToExternalWorld _toExternalWorld;

  // only for debugging and testing
  const Transformation _affinity;
}; // class GeometricTransformation

} // namespace stairs

#endif // TRANSFORMATION_H_
