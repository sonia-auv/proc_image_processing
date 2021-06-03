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

#include <proc_image_processing/algorithm/moments.h>

namespace proc_image_processing {

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
Moments::Moments(cv::Mat image, bool binary) {
  if (image.channels() != 1) {
    cv::Mat image2;
    cv::cvtColor(image, image2, CV_BGR2GRAY);
    image2.copyTo(image);
  }

  // Gets the moment by opencv moment function
  cv_moments_ = cv::moments(image, binary);

  mass_center_ = cv::Point2f(cv_moments_.m10 / cv_moments_.m00,
                             cv_moments_.m01 / cv_moments_.m00);

  // Here, remember that the mome are calculated on an image. If the image
  // was extract from a rotatedRect, it means the coordinate are in the angle
  // of the rotatedRect. X and Y axis of the image are rotated of angle degree
  real_center_ = cv::Point(image.cols / 2, image.rows / 2);

  x_distance_from_center_ = mass_center_.x - real_center_.x;
  y_distance_from_center_ = mass_center_.y - real_center_.y;
}

//------------------------------------------------------------------------------
//
Moments::Moments() {
  cv_moments_ = cv::Moments();
  mass_center_ = cv::Point(-1, -1);
  real_center_ = cv::Point(-1, -1);
  x_distance_from_center_ = 0.0f;
  y_distance_from_center_ = 0.0f;
}

//------------------------------------------------------------------------------
//
Moments::~Moments() {}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
void Moments::operator=(Moments moments) {
  cv_moments_ = moments.cv_moments_;
  mass_center_ = moments.mass_center_;
  real_center_ = moments.real_center_;
  x_distance_from_center_ = moments.x_distance_from_center_;
  y_distance_from_center_ = moments.y_distance_from_center_;
}

}  // namespace proc_image_processing
