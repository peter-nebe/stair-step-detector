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

#include "calibrationMark.h"
#include "image.h"
#include "drawing.h" // Rgb
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <thread>
using namespace std;
using namespace cv;

#define DEBUG_CV 0

namespace stairs
{

namespace
{

#if DEBUG_CV
namespace cvc // OpenCV colors
{
  const uint8_t X = 0xff;
  const Rgb red     { 0, 0, X };
  const Rgb green   { 0, X, 0 };
  const Rgb blue    { X, 0, 0 };
}
namespace cvRgb
{
  Scalar toCvRgb(Rgb c) { return Scalar(c.r, c.g, c.b); }
  const Scalar red     = toCvRgb(cvc::red);
  const Scalar green   = toCvRgb(cvc::green);
  const Scalar blue    = toCvRgb(cvc::blue);
}

Mat &getDrawingImg(int width = 0, int height = 0)
{
  static Mat drawingImg = Mat::zeros(height, width, CV_8UC3);
  return drawingImg;
}

inline Mat getNextDrawingQuadrant(const Image &img)
{
  int posX, posY;
  static const uint8_t *ptr0;
  static int count = 0;

  switch(count++)
  {
  case 0:
    posX = posY = 0;
    ptr0 = img.ptr();
    break;
  case 1:
    posX = img.width();
    posY = 0;
    break;
  case 2:
    posX = img.ptr() - img.height() * img.step() - ptr0;
    posY = img.height();
    /* no break */
  default:
    count = 0;
  }

  return Mat(getDrawingImg(img.width() * 2, img.height() * 2), { posX, posY, img.width(), img.height() });
}

void showDrawing()
{
  imshow("Calibration Marks", getDrawingImg());
  waitKey(1);
}
#endif // DEBUG_CV

using NumPixels_t = uint32_t;
using Histogram_t = array<NumPixels_t, numeric_limits<uint8_t>::max() + 1>;

Histogram_t calcHist(const Image &im)
{
  Histogram_t hist{};
  const uint8_t *row = im.pixels();

  for(int i = im.height(); i > 0; i--)
  {
    const uint8_t *p = row;
    for(int j = im.width(); j > 0; j--)
      ++hist[*p++];
    row += im.step();
  }

  return hist;
}

int detectThreshold(const Image &image)
{
  int threshold = 176; // default
  const NumPixels_t minNumPixels = 100; // minimum area of calibration mark
  const int maxAccumulationWidth = 16; // maximum gray value range of calibration mark
  const Histogram_t hist = calcHist(image);

  assert(!hist.empty());
  int gval = hist.size() - 1;
  while(hist[gval] == 0)
    if(--gval < 0)
      return threshold; // dummy (image is empty)

  const int accumulationEnd = gval - maxAccumulationWidth;
  if(accumulationEnd <= maxAccumulationWidth)
    return threshold; // dummy (image is too dark)

  NumPixels_t calibMarkPixels = 0;
  while(true)
  {
    calibMarkPixels += hist[gval];
    if(calibMarkPixels >= minNumPixels)
      break;
    if(--gval <= accumulationEnd)
      return threshold; // default (calibration mark too small or blurred)
  }

  int slidingBegin = accumulationEnd - maxAccumulationWidth + 1;
  int slidingEnd = accumulationEnd;
  NumPixels_t accu = 0;
  for(gval = slidingBegin; gval <= slidingEnd; gval++)
    accu += hist[gval];

  while(accu < minNumPixels)
  {
    if(--slidingBegin < 0)
      break;
    accu += hist[slidingBegin];
    accu -= hist[slidingEnd--];
  }

  threshold = (slidingEnd + accumulationEnd) / 2;

  return threshold;
}

// binarize the image with the bright calibration mark
void binarize(const Image &image, Image &binary, Mat &drawing)
{
  const int thresh = detectThreshold(image);
  threshold(image, binary, thresh, 0xff, THRESH_BINARY);

# if DEBUG_CV
  cvtColor(binary, drawing, COLOR_GRAY2RGB);
# if DEBUG_CV == 2
  showDrawing();
  std::this_thread::sleep_for(std::chrono::seconds(1));
# endif
# endif
}

} // namespace

CalibrationMark::Detections CalibrationMark::detect(const Image &image)
{
  const size_t maxDetections = 4;
  Detections detections;
  detections.reserve(maxDetections);

# if DEBUG_CV
  Mat drawing = getNextDrawingQuadrant(image);
# else
  Mat drawing;
# endif

  Image binary;
  binarize(image, binary, drawing);

  vector<Contour_t> contours;
  findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  for(Contour_t &contour : contours)
  {
    Contour_t polygon;
    approxPolyDP(contour, polygon, 3, true);

#   if DEBUG_CV
    for(const Point2i &p : contour)
      drawing.at<Rgb>(p.y, p.x) = cvc::red;
    polylines(drawing, polygon, true, cvRgb::blue);
#   endif

    const size_t numPoints = polygon.size();
    if(numPoints < 4 || numPoints > 12)
      continue;

    const cv::Rect bbox = boundingRect(polygon);
    const int ratio = bbox.width * 100 / bbox.height;
    if(ratio < 60 || ratio > 300)
      continue;

    const double area = contourArea(polygon);
    if(area < 80 || area > 1500)
      continue;

    const bool convex = isContourConvex(polygon);
    if(!convex)
      continue;

    cv::Point2f center;
    float radius;
    minEnclosingCircle(polygon, center, radius);
    if(radius < 8 || radius > 30)
      continue;

    Moments m = moments(contour);
    Point2 centroid(m.m10 / m.m00, m.m01 / m.m00);

#   if DEBUG_CV
    cv::drawMarker(drawing, cv::Point2d(centroid.x, centroid.y), cvRgb::green);
#   endif

    detections.push_back({ centroid, move(contour) });
    if(detections.size() == maxDetections)
      break;
  }

# if DEBUG_CV
  showDrawing();
# endif

  return detections;
}

} /* namespace stairs */

namespace cv { namespace traits
{
  template<>
  struct Type<stairs::Point2i> : Type<Point2i>
  {
  };
}}
