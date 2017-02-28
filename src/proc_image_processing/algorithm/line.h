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

#ifndef PROVIDER_VISION_ALGORITHM_LINE_H_
#define PROVIDER_VISION_ALGORITHM_LINE_H_

#include <math.h>
#include <stdlib.h>
#include <memory>
#include <opencv2/imgproc.hpp>

namespace proc_image_processing {

/**
 * Basic definitnion of a line
 */
class Line {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Line>;

  //============================================================================
  // P U B L I C   C / D T O R S

  Line(const cv::Point &start, const cv::Point &end);

  //============================================================================
  // P U B L I C   M E T H O D S

  // Debug
  void Draw(cv::Mat &img, cv::Scalar color);

  // Getters
  cv::Point GetCenter();

  cv::Point GetStart();

  cv::Point GetEnd();

  float GetAngle();

  float GetLength();

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  // start point is leftmosst point, end id farrigth point
  cv::Point center_;

  cv::Point start_;

  cv::Point end_;

  // Degree
  float angle_;

  float length_;
};

bool lengthSort(Line a, Line b);

bool centerXSort(Line a, Line b);

bool centerYSort(Line a, Line b);

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_ALGORITHM_LINE_H_
