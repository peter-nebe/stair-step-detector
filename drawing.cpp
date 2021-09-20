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

#include "drawing.h"
#include "../third-party/stb_easy_font.h"
#include "GL/gl.h"
#include <sstream>
#include <iomanip>
using namespace std;

namespace stairs
{

namespace
{

const int osPrecision = 3;
const int osWidth = 6;

struct CoordinateOsstream : ostringstream
{
  CoordinateOsstream &operator<<(Coordinate_t coordinate)
  {
    static_cast<ostream&>(*this)
      << std::fixed << setprecision(osPrecision)
      << setw(osWidth) << coordinate;
    return *this;
  }
  CoordinateOsstream &operator<<(const Point2 &p)
  {
    static_cast<ostream&>(*this)
      << std::fixed << setprecision(osPrecision)
      << setw(osWidth) << p.x << endl
      << setw(osWidth) << p.y;
    return *this;
  }
  CoordinateOsstream &operator<<(const Point3f &p)
  {
    static_cast<ostream&>(*this)
      << std::fixed << setprecision(osPrecision)
      << setw(osWidth) << p.x << endl
      << setw(osWidth) << p.y << endl
      << setw(osWidth) << p.z;
    return *this;
  }
  void clear()
  {
    str(string());
  }
}; // struct CoordinateOsstream

} // namespace

void setColor(Rgb c)
{
  glColor3ub(c.r, c.g, c.b);
}

void drawRectangle(int x, int y, int w, int h)
{
  const int quad[4][2]{{ x, y },
                       { x + w, y },
                       { x + w, y + h }, // vertices clockwise
                       { x, y + h }};
  drawQuadrilateral(quad);
}

// order of vertices clockwise or counterclockwise
void drawQuadrilateral(const int v[4][2])
{
  glBegin(GL_LINE_LOOP);
  glVertex2iv(v[0]);
  glVertex2iv(v[1]);
  glVertex2iv(v[2]);
  glVertex2iv(v[3]);
  glEnd();
}

void drawDot(int x, int y)
{
  glBegin(GL_QUADS);
  glVertex2i(x - 1, y - 1);
  glVertex2i(x + 2, y - 1);
  glVertex2i(x + 2, y + 2);
  glVertex2i(x - 1, y + 2);
  glEnd();
}

void drawCross(int x, int y)
{
  const int l = 10;

  glBegin(GL_LINES);
  glVertex2i(x - l - 1, y);
  glVertex2i(x + l, y);
  glVertex2i(x, y - l - 1);
  glVertex2i(x, y + l);
  glEnd();
}

void drawText(int x, int y, const char *text)
{
  const int bufsize = 10000;
  char *const buffer = new char[bufsize];
  const int numQuads = stb_easy_font_print(x, y, const_cast<char*>(text), nullptr, buffer, bufsize);

  glEnableClientState(GL_VERTEX_ARRAY);
  glVertexPointer(2, GL_FLOAT, 16, buffer);
  glDrawArrays(GL_QUADS, 0, numQuads * 4);
  glDisableClientState(GL_VERTEX_ARRAY);

  delete[] buffer;
}

const int drawOffset = 1;

void setDrawOffset(int x, int y)
{
  glPushMatrix();
  glTranslatef(x + drawOffset, y + drawOffset, 0);
}

void resetDrawOffset()
{
  glPopMatrix();
}

void drawSubframeRect(const Rect2i &rect, Rgb color)
{
  setColor(color);

  Rect2i r = rect;
  if(r.x == 0)
  {
    r.x = drawOffset;
    r.width -= drawOffset;
  }
  if(r.y == 0)
  {
    r.y = drawOffset;
    r.height -= drawOffset;
  }

  drawRectangle(r);
}

void drawMarker(const Point2 &center, const Contour_t &contour)
{
  setColor(blue);
  drawPolygon(contour);
  setColor(green);
  drawCross(center);
}

void drawMarker(const Point2 &center, const Point3f &coord)
{
  const int cx = toInt(center.x);
  const int cy = toInt(center.y);
  setColor(blue);
  drawCross(cx, cy);

  CoordinateOsstream os;
  os << coord;
  drawText(cx + 11, cy - 17, os.str().c_str());
}

void drawPoints(const Points2i_t &points)
{
  glBegin(GL_POINTS);
  for(const Point2i &p : points)
    glVertex2i(p.x, p.y);
  glEnd();
}

void drawLineStrip(const Points2i_t &points)
{
  glBegin(GL_LINE_STRIP);
  for(const Point2i &p : points)
    glVertex2i(p.x, p.y);
  glEnd();
}

void drawPolygon(const Points2i_t &points)
{
  glBegin(GL_LINE_LOOP);
  for(const Point2i &p : points)
    glVertex2i(p.x, p.y);
  glEnd();
}

void drawRectangle(const Rect2i &rect)
{
  drawRectangle(rect.x, rect.y, rect.width, rect.height);
}

void drawOctagon(const Point2f &center, float radius)
{
  const int cx = toInt(center.x);
  const int cy = toInt(center.y);
  const int r = toInt(radius);
  const int r45 = toInt(radius * 0.7071f);
  const Points2i_t points
  {
    { cx, cy - r },
    { cx + r45, cy - r45 },
    { cx + r, cy },
    { cx + r45, cy + r45 },
    { cx, cy + r },
    { cx - r45, cy + r45 },
    { cx - r, cy },
    { cx - r45, cy - r45 }
  };
  drawPolygon(points);
}

void drawQuadrilateral(const Quadrilateralf_t &co, const Quadrilateral_t &labeling, Coordinate_t zLabel)
{
  const int xLeft = toInt(co[0].x);
  const int yLeft = toInt(co[0].y);
  const int xRight = toInt(co[1].x);
  const int yRight = toInt(co[1].y);
  const int quad[4][2]{{ xLeft, yLeft },
                       { xRight, yRight },
                       { toInt(co[3].x), toInt(co[3].y) }, // vertices counterclockwise
                       { toInt(co[2].x), toInt(co[2].y) }};
  setColor(green);
  drawQuadrilateral(quad);

  int xOffsCenter = -16;
  int yOffsCenter = 2;
  int xOffsLeft = xOffsCenter;
  int xOffsRight = xOffsCenter;
  int yOffsSide = yOffsCenter;

  int viewport[4];
  glGetIntegerv(GL_VIEWPORT, viewport);
  const int textHeight = 19;
  const int yMax = viewport[3] - textHeight - yOffsSide;
  if(yLeft > yMax || yRight > yMax)
  {
    xOffsLeft = -32;
    xOffsRight = 0;
    yOffsCenter = -9;
    yOffsSide = -textHeight;
  }

  CoordinateOsstream os;
  os << labeling[0];
  drawText(xLeft + xOffsLeft, yLeft + yOffsSide, os.str().c_str());

  os.clear();
  os << labeling[1];
  drawText(xRight + xOffsRight, yRight + yOffsSide, os.str().c_str());

  os.clear();
  os << zLabel;
  drawText((xLeft + xRight) / 2 + xOffsCenter, (yLeft + yRight) / 2 + yOffsCenter, os.str().c_str());
}

} /* namespace stairs */
