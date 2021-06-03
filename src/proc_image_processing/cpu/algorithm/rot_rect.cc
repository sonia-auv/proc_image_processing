/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
/// \section LICENSE
/// This file is part of S.O.N.I.A. software.
///
/// S.O.N.I.A. software is free software: you can redistribute it and/or modify
/// it under the terms of the GNU General Public License as published by
/// the Free Software Foundation, either version 3 of the License, or
/// (at your option) any later version.
///
/// S.O.N.I.A. software is distributed in the hope that it will be useful,
/// but WITHOUT ANY WARRANTY; without even the implied warranty of
/// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
/// GNU General Public License for more details.
///
/// You should have received a copy of the GNU General Public License
/// along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.

#include <cpu/algorithm/rot_rect.h>

namespace proc_image_processing {

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
RotRect::RotRect(const std::vector<cv::Point> &edges)
    : cv::RotatedRect(cv::minAreaRect(edges)) {
  setValues();
  points(pts_);
}

//------------------------------------------------------------------------------
//
RotRect::RotRect() : cv::RotatedRect() {}

//------------------------------------------------------------------------------
//
RotRect::RotRect(const cv::RotatedRect &rotRect) : cv::RotatedRect(rotRect) {
  setValues();
  points(pts_);
}

//------------------------------------------------------------------------------
//
RotRect::RotRect(const RotRect &a) : cv::RotatedRect(a) {
  for (int i = 0; i < 4; i++) pts_[i] = a.pts_[i];
}

//------------------------------------------------------------------------------
//
RotRect::~RotRect() {}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
void RotRect::drawRect(cv::Mat &out, cv::Scalar color, int thickness) {
  cv::line(out, pts_[0], pts_[1], color, thickness);
  cv::line(out, pts_[1], pts_[2], color, thickness);
  cv::line(out, pts_[2], pts_[3], color, thickness);
  cv::line(out, pts_[3], pts_[0], color, thickness);
}

//------------------------------------------------------------------------------
//
void RotRect::setValues() {
  float in_angle = angle;
  float out_angle = in_angle;
  // angle is consider in of height in opencv
  // since height always is not always on the longest size,
  // make sure always return the longest size in the height
  // and makes the angle follow.
  if (size.width > size.height) {
    std::swap(size.width, size.height);
    if (in_angle < 0)
      out_angle = 90 + in_angle;
    else
      out_angle = -(90 - in_angle);
  }
  angle = out_angle;
}

//------------------------------------------------------------------------------
//
void RotRect::swap(RotRect &a) {
  std::swap(a.angle, angle);
  std::swap(a.center, center);
  std::swap(a.pts_, pts_);
  std::swap(a.size, size);
}

//------------------------------------------------------------------------------
//
RotRect &RotRect::operator=(RotRect rotRect) {
  swap(rotRect);
  return *this;
}

//------------------------------------------------------------------------------
//
RotRect &RotRect::operator=(cv::RotatedRect rotRect) {
  RotRect slRotRect(rotRect);
  swap(slRotRect);
  return *this;
}

//------------------------------------------------------------------------------
//
bool RotRect::operator==(const RotRect &rotRect) {
  bool result = true;
  if (rotRect.center != center) {
    result = false;
  }
  if (rotRect.size != size) {
    result = false;
  }
  if (std::abs(rotRect.angle - rotRect.angle) > 0.5) {
    result = false;
  }
  return result;
}

//------------------------------------------------------------------------------
//
cv::Point2f *RotRect::getCorners() { return pts_; }

}  // namespace proc_image_processing
