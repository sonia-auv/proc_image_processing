/// \author	Pierluc Bédard <pierlucbed@gmail.com>
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

#ifndef PROVIDER_VISION_FILTERS_IMAGE_CROPPER_H_
#define PROVIDER_VISION_FILTERS_IMAGE_CROPPER_H_

#include <proc_image_processing/filters/filter.h>
#include <memory>

namespace proc_image_processing {

class ImageCropper : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Blurr>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit ImageCropper(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        enable_("Enable", false, &parameters_),
        x_offset_("X Offset", 0, 0, 2000, &parameters_),
        y_offset_("Y Offset", 0, 0, 2000, &parameters_),
        x_reduction_("X Reduction", 0, 0, 2000, &parameters_),
        y_reduction_("Y Reduction", 0, 0, 2000, &parameters_) {
    SetName("ImageCropper");
  }

  virtual ~ImageCropper() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void Execute(cv::Mat &image) {
    if (enable_()) {
      if ((x_offset_() + x_reduction_() < image.size[1]) |
            (y_offset_() + y_reduction_() < image.size[0])) {
        image = image(cv::Rect(x_offset_(), y_offset_(),
                               image.size[1] - x_reduction_() - x_offset_(),
                               image.size[0] - y_reduction_() - y_offset_()));
      }
    }
  }

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  Parameter<bool> enable_;
  RangedParameter<int> x_offset_, y_offset_, x_reduction_, y_reduction_;
};

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_IMAGE_CROPPER_H_
