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

#include "stairs.h"
#include <iostream>
#include <iomanip>
using namespace std;

namespace stairs
{

namespace
{

using StairStep = Stairs::StairStep;

ostream &operator<<(ostream &os, const Point2 &p)
{
  os << fixed << setprecision(3)
     << '[' << p.x << ',' << p.y << ']';
  return os;
}

ostream &operator<<(ostream &os, const StairStep &s)
{
  os << fixed << setprecision(3)
     << "[[\"height\"," << s.height
     << "],[\"quadrilateral\","
     << s.quadrilateral[0] << ','
     << s.quadrilateral[1] << ','
     << s.quadrilateral[2] << ','
     << s.quadrilateral[3] << "]]";
  return os;
}

} // namespace

string Stairs::serialize() const
{
  ostringstream os;
  os << "[\"stairs\",[\"stairSteps\"," << stairSteps.size() << ']';

  if(!stairSteps.empty())
  {
    os << ",[";
    for(auto it = stairSteps.begin(); it != stairSteps.end() - 1; ++it)
      os << *it << ',';
    os << stairSteps.back() << ']';
  }
  os << ']';

  return os.str();
}

} /* namespace stairs */
