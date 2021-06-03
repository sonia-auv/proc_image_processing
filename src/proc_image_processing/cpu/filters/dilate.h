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

#ifndef PROVIDER_VISION_FILTERS_DILATE_H_
#define PROVIDER_VISION_FILTERS_DILATE_H_

#include "filter.h"
#include <memory>

namespace proc_image_processing {

class Dilate : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Dilate>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit Dilate(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        enable_("Enable", false, &parameters_),
        use_square_kernel_("Square_kernel", true, &parameters_),
        kernel_type_("Kernel_type", 0, 0, 2, &parameters_),
        kernel_size_x_("Width", 1, 0, 20, &parameters_),
        kernel_size_y_("Height", 1, 0, 20, &parameters_),
        iteration_("Iteration", 1, 0, 20, &parameters_),
        anchor_(-1, -1) {
    SetName("Dilate");
  }

  virtual ~Dilate() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void Execute(cv::Mat &image) {
    if (enable_()) {
      int kernel_type = 0;
      switch (kernel_type_()) {
        case 0:
          kernel_type = cv::MORPH_RECT;
          break;
        case 1:
          kernel_type = cv::MORPH_ELLIPSE;
          break;
        case 2:
          kernel_type = cv::MORPH_CROSS;
          break;
      }

      cv::Size size(kernel_size_x_() * 2 + 1,
                    (use_square_kernel_() ? kernel_size_x_() * 2 + 1
                                          : kernel_size_y_() * 2 + 1));
      cv::Mat kernel = cv::getStructuringElement(kernel_type, size, anchor_);

      cv::dilate(image, image, kernel, anchor_, iteration_());
    }
  }

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  Parameter<bool> enable_, use_square_kernel_;
  RangedParameter<int> kernel_type_;
  RangedParameter<int> kernel_size_x_, kernel_size_y_;
  RangedParameter<int> iteration_;

  const cv::Point anchor_;
};

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_DILATE_H_
