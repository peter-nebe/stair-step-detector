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

#include "transformation.h"
#include "qvmTraits.h"
#include "boost/qvm/vec_operations.hpp"
#include "boost/qvm/vec_mat_operations.hpp"
#include "boost/qvm/map_mat_vec.hpp"
#include "boost/qvm/map_mat_mat.hpp"
#include <tuple>
using namespace boost::qvm;

namespace stairs
{

namespace
{

template<class T>
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type
almost_equal(T x, T y, int ulp = 2)
{
  const auto abs = std::fabs(x - y);

  // the machine epsilon has to be scaled to the magnitude of the values used
  // and multiplied by the desired precision in ULPs (units in the last place)
  return abs <= std::numeric_limits<T>::epsilon() * std::fabs(x + y) * ulp
      // unless the result is subnormal
      || abs < std::numeric_limits<T>::min();
}

template<int Dim>
bool isMagOne(const Vector_<Dim> &v)
{
  return almost_equal(mag(v), 1.0);
}

template<int Dim>
bool isDotZero(const Vector_<Dim> &a, const Vector_<Dim> &b)
{
  return std::fabs(dot(a, b)) < 1e-15;
}

template<int Dim>
bool isDetOne(const Matrix_<Dim> &m)
{
  return almost_equal(determinant(m), 1.0, 3);
}

Matrix_<2> makeRotation(const ReferencePoints_<2> &rp, const ReferencePoints_<2> &rpMapping)
{
  auto normalizedVector = [](const ReferencePoints_<2> &rp)
                          {
                            const Vector_<2> v = normalized(rp[1] - rp[0]);
                            return std::make_tuple(v.x, v.y);
                          };

  const auto [dx, dy] = normalizedVector(rp);
  const auto [dxm, dym] = normalizedVector(rpMapping);
  const auto xBaseX = dx * dxm + dy * dym;
  const auto xBaseY = dy * dxm - dx * dym;

  const Vector_<2> xBase{ xBaseX, xBaseY };
  const Vector_<2> yBase{ -xBaseY, xBaseX };
  assert(isMagOne(xBase));
  assert(isMagOne(yBase));
  assert(isDotZero(xBase, yBase));

  Matrix_<2> rot;
  col<0>(rot) = xBase;
  col<1>(rot) = yBase;
  assert(isDetOne(rot));

  return rot;
}

} // namespace

template<>
Transformation_<2>::Transformation_(const RefPoints &rp, const RefPoints &rpMapping)
{
  // set rotation matrix
  _a = makeRotation(rp, rpMapping);

  // rotation matrix: inverse = transpose
  _aInv = transposed(_a);
  assert(isDetOne(_aInv));

  // set translation vector
  _b = rp.front() - _a * rpMapping.front();
}

template<>
Transformation_<3>::Transformation_(const RefPoints &triangleInPlane)
{
  // calculate orientation of the plane in mapping coordinates

  const Vec p = triangleInPlane[0];
  const Vec u = triangleInPlane[1] - p;
  const Vec v = triangleInPlane[2] - p;

  const Vec n0 = normalized(cross(u, v));
  assert(isMagOne(n0));
  const Vec zBase = -n0;

  // dot(yBase, zBase) = 0
  // ybx zbx + yby zby + ybz zbz = 0
  const Coordinate_t ybx = 0;
  const Coordinate_t ybz = 1;
  // yby zby + zbz = 0
  // yby zby = -zbz
  // yby = -zbz / zby
  const Coordinate_t yby = -zBase.z / zBase.y;

  const Vec yBase = normalized(Vec{ ybx, yby, ybz });
  const Vec xBase = cross(yBase, zBase);
  assert(isMagOne(xBase));
  assert(isMagOne(yBase));
  assert(isMagOne(zBase));
  assert(isDotZero(xBase, yBase));
  assert(isDotZero(yBase, zBase));
  assert(isDotZero(zBase, xBase));

  Mat rot;
  col<0>(rot) = xBase;
  col<1>(rot) = yBase;
  col<2>(rot) = zBase;

  // set inverse rotation matrix
  _aInv = rot;

  // rotation matrix: inverse = transpose
  _a = transposed(_aInv);
  assert(isDetOne(_a));
  assert(isDetOne(_aInv));

  const Coordinate_t distFromOrigin = dot(p, n0);
  assert(distFromOrigin > 0);

  // set translation vector
  _b = Vec{ 0, 0, distFromOrigin };
}

template<>
Transformation_<3>::Transformation_(const RefPoints &rp, const RefPoints &rpMapping)
{
  auto makeLinearTransformation = [](const RefPoints &threePoints)
                                  {
                                    const Vec p = threePoints[0];
                                    const Vec u = threePoints[1] - p;
                                    const Vec v = threePoints[2] - p;

                                    Mat m;
                                    col<0>(m) = u;
                                    col<1>(m) = v;
                                    col<2>(m) = cross(u, v);

                                    return m;
                                  };
  const Mat m = makeLinearTransformation(rp);
  const Mat mm = makeLinearTransformation(rpMapping);

  _a = m * inverse(mm);
  _aInv = inverse(_a);

  // set translation vector
  _b = rp.front() - _a * rpMapping.front();
}

Point3 WorldToCamera::operator()(const Point3 &p) const
{
  return _camera.transformInv(p);
}

GeometricTransformation::GeometricTransformation(const RefPoints &worldPoints, const RefPoints &cameraPoints)
: _camera(cameraPoints), // camera coordinates
  _world({
           worldPoints[0], // external world coordinates
           worldPoints[1]
         },
         {
           _cameraToWorld(cameraPoints[0]), // camera-dependent world coordinates
           _cameraToWorld(cameraPoints[1])
         }),
  _worldZ(worldPoints[0].z),
  _affinity(worldPoints, cameraPoints)
{
}

Point3 GeometricTransformation::toExternalWorld(const Point3 &p) const
{
  const Point2 p2 = p;
  return{ _world.transform(p2), _worldZ + p.z };
}

} /* namespace stairs */
