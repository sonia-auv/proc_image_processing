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

#ifndef PROVIDER_VISION_FILTERS_LAPLACIAN_H_
#define PROVIDER_VISION_FILTERS_LAPLACIAN_H_

#include "filter.h"
#include <memory>

namespace proc_image_processing {

class Laplacian : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Laplacian>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit Laplacian(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        enable_("Enable", false, &parameters_),
        convert_to_uchar_("Convert_to_uchar", true, &parameters_),
        delta_("Delta", 0, 0, 255, &parameters_),
        scale_("Scale", 1, 0, 255, &parameters_),
        size_("Size", 2, 1, 20, &parameters_) {
    SetName("Laplacian");
  }

  virtual ~Laplacian() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void Execute(cv::Mat &image) {
    if (enable_()) {
      if (image.channels() > 1) {
        cv::cvtColor(image, image, CV_BGR2GRAY);
      }
      int size = size_() * 2 + 1;

      if (convert_to_uchar_()) {
        cv::Laplacian(image, image, CV_8U, size, scale_(), delta_(),
                      cv::BORDER_DEFAULT);
      } else {
        cv::Laplacian(image, image, CV_32F, size, scale_(), delta_(),
                      cv::BORDER_DEFAULT);
      }
    }
  }

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  Parameter<bool> enable_, convert_to_uchar_;
  RangedParameter<double> delta_, scale_;
  RangedParameter<int> size_;
};

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_SOBEL_H_
