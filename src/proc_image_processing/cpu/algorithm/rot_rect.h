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

#ifndef PROVIDER_VISION_ALGORITHM_ROT_RECT_H_
#define PROVIDER_VISION_ALGORITHM_ROT_RECT_H_

#include "../../../../../../.cache/JetBrains/CLion2021.1/.remote/127.0.0.1_2222/9683959d-c64a-40ab-a3b2-ad89a608eb90/usr/include/c++/7/math.h"
#include "../../../../../../.cache/JetBrains/CLion2021.1/.remote/127.0.0.1_2222/9683959d-c64a-40ab-a3b2-ad89a608eb90/usr/include/c++/7/memory"
#include "../../../../../../.cache/JetBrains/CLion2021.1/.remote/127.0.0.1_2222/9683959d-c64a-40ab-a3b2-ad89a608eb90/usr/local/include/opencv2/core/core.hpp"
#include "../../../../../../.cache/JetBrains/CLion2021.1/.remote/127.0.0.1_2222/9683959d-c64a-40ab-a3b2-ad89a608eb90/usr/local/include/opencv2/opencv.hpp"

namespace proc_image_processing {

#define PROVIDER_VISION_TOP_LEFT 0
#define PROVIDER_VISION_TOP_RIGHT 4
#define PROVIDER_VISION_BOTTOM_LEFT 1
#define PROVIDER_VISION_BOTTOM_RIGHT 2

// Rotated rect ensure that the height member is the longest one
// and the angle is in the direction of the height
// it also contains usefull method to play with rotated rectangle
class RotRect : public cv::RotatedRect {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<RotRect>;

  //============================================================================
  // P U B L I C   C / D T O R S

  // Constructor/destructor
  RotRect(const std::vector<cv::Point> &edges);

  RotRect(const cv::RotatedRect &rotRect);

  RotRect(const RotRect &a);

  RotRect();

  ~RotRect();

  //============================================================================
  // P U B L I C   O P E R A T O R S

  RotRect &operator=(RotRect rotRect);

  RotRect &operator=(cv::RotatedRect rotRect);

  bool operator==(const RotRect &rotRect);

  //============================================================================
  // P U B L I C   M E T H O D S

  void drawRect(cv::Mat &out, cv::Scalar color, int thickness = 1);

  // Create the class with another rotated rect
  void swap(RotRect &a);

  cv::Point2f *getCorners();

 private:
  //============================================================================
  // P R I V A T E   M E T H O D S

  // Set height to the longest side of the rectangle and
  void setValues();

  //============================================================================
  // P R I V A T E   M E M B E R S

  cv::Point2f pts_[4];
};

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_ALGORITHM_ROT_RECT_H_
