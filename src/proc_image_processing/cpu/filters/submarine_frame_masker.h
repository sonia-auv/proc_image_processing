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

#ifndef PROVIDER_VISION_FILTERS_SUBMARINE_FRAME_MASKER_H_
#define PROVIDER_VISION_FILTERS_SUBMARINE_FRAME_MASKER_H_

#include "filter.h"
#include <memory>

namespace proc_image_processing {

class SubmarineFrameMasker : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<SubmarineFrameMasker>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit SubmarineFrameMasker(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        enable_("Enable", false, &parameters_),
        rotate_type_("Rotation_type", 0, 0, 3, &parameters_,
                     "Rotate type: 0=NONE, 1=x axis, 2=y axis, 3=all axis"),
        prev_rot_value_(0) {
    SetName("SubmarineFrameMasker");
    std::string mask_name =
        std::string(getenv("SONIA_WORKSPACE_ROOT")) +
        std::string("/ros/src/vision_server/config/bottom_mask.jpg");
    bottom_mask_ = cv::imread(mask_name, CV_LOAD_IMAGE_GRAYSCALE);
  }

  virtual ~SubmarineFrameMasker() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void Execute(cv::Mat &image) {
    if (enable_()) {
      if (prev_rot_value_ != rotate_type_()) {
        prev_rot_value_ = rotate_type_();
        switch (rotate_type_()) {
          case 1:
            cv::flip(bottom_mask_, bottom_mask_, 0);
            break;
          case 2:
            cv::flip(bottom_mask_, bottom_mask_, 1);
            break;
          case 3:
            cv::flip(bottom_mask_, bottom_mask_, -1);
            break;
        }
      }
      if (image.size() == bottom_mask_.size())
        cv::bitwise_and(image, bottom_mask_, image);
    }
  }

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  Parameter<bool> enable_;
  RangedParameter<int> rotate_type_;
  cv::Mat bottom_mask_;
  int prev_rot_value_;
};

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_SUBMARINE_FRAME_MASKER_H_
