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

#ifndef PROVIDER_VISION_ALGORITHM_TYPE_AND_CONST_H_
#define PROVIDER_VISION_ALGORITHM_TYPE_AND_CONST_H_

#include "../../../../../../.cache/JetBrains/CLion2021.1/.remote/127.0.0.1_2222/9683959d-c64a-40ab-a3b2-ad89a608eb90/usr/include/c++/7/memory"
#include "../../../../../../.cache/JetBrains/CLion2021.1/.remote/127.0.0.1_2222/9683959d-c64a-40ab-a3b2-ad89a608eb90/usr/local/include/opencv2/opencv.hpp"

namespace proc_image_processing {

typedef std::vector<cv::Point> contour_t;
typedef std::vector<contour_t> contourList_t;
typedef std::vector<cv::Vec4i> hierachy_t;

#define NEXT_CTR 0
#define PREV_CTR 1
#define FIRST_CHILD_CTR 2
#define PARENT_CTR 3

typedef std::vector<cv::Vec4i> defectuosity_t;

// Enum for the rotation function
enum rotationType { R_NONE = 0, R_90, R_180, R_270 };

enum symmetryType { S_NONE = 0, S_X_AXIS, S_Y_AXIS, S_BOTH };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_ALGORITHM_TYPE_AND_CONST_H_
