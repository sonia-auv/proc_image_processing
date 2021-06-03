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

#include <cpu/algorithm/line.h>

namespace proc_image_processing {

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
Line::Line(const cv::Point &start, const cv::Point &end)
    : start_(start), end_(end), angle_(0), isSwap_(false) {
  if (start_.x > end_.x)
  {
      isSwap_ = true;
      std::swap(start_, end_);
  }

  int yDiff = abs(start_.y - end_.y);
  int xDiff = abs(start_.x - end_.x);

  if (start_.y > end_.y)
    center_.y = end_.y + yDiff / 2;
  else
    center_.y = start_.y + yDiff / 2;

  if (start_.x > end_.x)
    center_.x = end_.x + xDiff / 2;
  else
    center_.x = start_.x + xDiff / 2;

  // inversion in the start_ end_ for x y is because y is positive
  // downward.
  float at = atan2(static_cast<double>(start_.y - end_.y),
                   static_cast<double>(end_.x - start_.x));
  angle_ = at / (2 * M_PI) * 360;

  length_ = sqrt(pow((start_.y - end_.y), 2) + pow((start_.y - end_.y), 2));
}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
void Line::Draw(cv::Mat &img, cv::Scalar color) {
  cv::line(img, start_, end_, color, 4, 8);
}

//------------------------------------------------------------------------------
//
cv::Point Line::PerpendicularLine()
{
  std::vector<cv::Point> secondLine;

  cv::Point vector1;
  Eigen::Vector3d vector2;
  vector1 = end_ - start_;

  Eigen::Vector3d vector(vector1.x, vector1.y, 1);

  double min = fabs(vector.x());
  Eigen::Vector3d cardinalAxis;
  cardinalAxis << 1, 0, 1;

  if (fabs(vector.y()) < min)
  {
    cardinalAxis << 0, 1, 1;
  }

  vector2 = vector.cross(cardinalAxis);

  return cv::Point(vector2.x(), vector2.y());
}

//------------------------------------------------------------------------------
//
std::vector<cv::Point> Line::GenerateLine(cv::Mat &img)
{
    std::vector<cv::Point> line;
    cv::LineIterator it(img, start_, end_);

    for (int i = 0; i < it.count; i++, ++it)
    {
        cv::Point p = it.pos();
        line.push_back(p);
    }

    return line;
}

//------------------------------------------------------------------------------
//
cv::Point Line::GetCenter() { return center_; }

//------------------------------------------------------------------------------
//
cv::Point Line::GetStart() { return start_; }

//------------------------------------------------------------------------------
//
cv::Point Line::GetEnd() { return end_; }

//------------------------------------------------------------------------------
//
float Line::GetAngle() { return angle_; }

//------------------------------------------------------------------------------
//
float Line::GetLength() { return length_; }

//------------------------------------------------------------------------------
//
bool lengthSort(Line a, Line b) { return a.GetLength() > b.GetLength(); }

//------------------------------------------------------------------------------
//
bool centerXSort(Line a, Line b) { return a.GetCenter().x > b.GetCenter().x; }

//------------------------------------------------------------------------------
//
bool centerYSort(Line a, Line b) { return a.GetCenter().y > b.GetCenter().y; }

}  // namespace proc_image_processing
