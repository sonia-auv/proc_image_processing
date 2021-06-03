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

#include "contour.h"

namespace proc_image_processing {

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
Contour::Contour(const std::vector<cv::Point> &ctr) : contour_(ctr) {}

//------------------------------------------------------------------------------
//
Contour::Contour(const cv::RotatedRect &rect) {
  cv::Point2f pts[4];
  rect.points(pts);

  for (int j = 0; j < 4; j++) {
    contour_.push_back(pts[j]);
  }
}

}  // namespace proc_image_processing
