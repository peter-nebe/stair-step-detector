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

#ifndef QVMTRAITS_H_
#define QVMTRAITS_H_

#include "types.h" // Point_
#include "librealsense2/hpp/rs_frame.hpp" // rs2::vertex
#include "boost/qvm/vec_traits_defaults.hpp"
#include "boost/qvm/deduce_vec.hpp"

namespace boost { namespace qvm
{

/*
 * stairs::Point_
 */
template<int Dim>
struct vec_traits<stairs::Point_<Dim>> : vec_traits_defaults<stairs::Point_<Dim>, stairs::Coordinate_t, Dim>
{
  template<int I>
  static stairs::Coordinate_t &write_element(stairs::Point_<Dim>&);
};
template<> template<>
inline stairs::Coordinate_t &vec_traits<stairs::Point2>::write_element<0>(stairs::Point2 &p)
{
  return p.x;
}
template<> template<>
inline stairs::Coordinate_t &vec_traits<stairs::Point2>::write_element<1>(stairs::Point2 &p)
{
  return p.y;
}
template<> template<>
inline stairs::Coordinate_t &vec_traits<stairs::Point3>::write_element<0>(stairs::Point3 &p)
{
  return p.x;
}
template<> template<>
inline stairs::Coordinate_t &vec_traits<stairs::Point3>::write_element<1>(stairs::Point3 &p)
{
  return p.y;
}
template<> template<>
inline stairs::Coordinate_t &vec_traits<stairs::Point3>::write_element<2>(stairs::Point3 &p)
{
  return p.z;
}

/*
 * rs2::vertex
 */
template<>
struct vec_traits<rs2::vertex> : vec_traits_defaults<rs2::vertex, float, 3>
{
  template<int I>
  static float &write_element(rs2::vertex&);
};
template<>
inline float &vec_traits<rs2::vertex>::write_element<0>(rs2::vertex &v)
{
  return v.x;
}
template<>
inline float &vec_traits<rs2::vertex>::write_element<1>(rs2::vertex &v)
{
  return v.y;
}
template<>
inline float &vec_traits<rs2::vertex>::write_element<2>(rs2::vertex &v)
{
  return v.z;
}

template<class, int, int> struct mat;

}} // namespace boost::qvm

namespace stairs
{
  template<int Dim>
  using Matrix_ = boost::qvm::mat<Coordinate_t, Dim, Dim>;
}

namespace boost { namespace qvm
{

/*
 * specify result type of matrix-vector operations
 */
template<typename PointType, int Dim>
struct deduce_vec2<stairs::Matrix_<Dim>, PointType, Dim>
{
  typedef stairs::Point_<Dim> type;
};

}} // namespace boost::qvm

#endif // QVMTRAITS_H_
