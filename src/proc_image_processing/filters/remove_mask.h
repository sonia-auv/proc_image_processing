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

#ifndef PROVIDER_VISION_FILTERS_REMOVE_MASK_H_
#define PROVIDER_VISION_FILTERS_REMOVE_MASK_H_

#include <proc_image_processing/filters/filter.h>
#include <memory>

namespace proc_image_processing {

// see http://docs.opencv.org/modules/imgproc/doc/filtering.html
// for more detail on how and why
//
// This is a program to execute image filter other than erode, dilate and
// morphologicalEx. Those are more blur function than pixelizer
// settings are for the differents type of filters, and does not apply to all
class RemoveMask : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Blurr>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit RemoveMask(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        enable_("Enable", false, &parameters_),
        type_("Type", 2, 0, 3, &parameters_,
              "1=Blur, 2=GaussianBlur, 3=MedianBlur"),
        kernel_size_("Kernel_size", 1, 0, 35, &parameters_),
        anchor_(-1, -1) {
    SetName("RemoveMask");
  }

  virtual ~RemoveMask() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void Execute(cv::Mat &image) {
    if (enable_()) {
        global_params_.getOriginalImage().copyTo(image,image);
    }
  }

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  Parameter<bool> enable_;
  RangedParameter<int> type_, kernel_size_;

  const cv::Point anchor_;
};

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_REMOVE_MASK_H_
