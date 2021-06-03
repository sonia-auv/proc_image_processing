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

#include <proc_image_processing/cpu/algorithm/object_full_data.h>

namespace proc_image_processing {

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
ObjectFullData::ObjectFullData(const cv::Mat &originalImage,
                               const cv::Mat &binaryImage,
                               const Contour &contour)
    : ObjectBasicData(originalImage, binaryImage, contour) {}


cv::Point ObjectVecMedian(ObjectFullData::FullObjectPtrVec objVec) {
  std::vector<float> xVec;
  std::vector<float> yVec;
  for (auto &elem:objVec) {
    xVec.insert(xVec.begin(), elem->GetCenter().x);
    yVec.insert(yVec.begin(), elem->GetCenter().y);
  }
  return cv::Point((int)(Median(xVec)),
                   (int)(Median(yVec)));
}

}  // namespace proc_image_processing
