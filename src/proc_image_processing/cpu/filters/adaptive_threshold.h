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

#ifndef PROVIDER_VISION_FILTERS_ADAPTIVE_THRESHOLD_H_
#define PROVIDER_VISION_FILTERS_ADAPTIVE_THRESHOLD_H_

#include <proc_image_processing/filters/filter.h>
#include <memory>

namespace proc_image_processing {

class AdaptiveThreshold : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<AdaptiveThreshold>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit AdaptiveThreshold(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        enable_("Enable", false, &parameters_),
        method_("Method", 0, 0, 1, &parameters_, "0=Gaussian 1=Mean"),
        threshold_type_("Threshold_type", 0, 0, 1, &parameters_,
                        "0=BIN, 1=BIN_INV"),
        _block_size("Size", 1, 1, 40, &parameters_),
        c_param_("C_param", 0.0f, -255.0f, 255.0f, &parameters_) {
    SetName("AdaptiveThreshold");
  }

  virtual ~AdaptiveThreshold() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void Execute(cv::Mat &image) {
    if (enable_()) {
      if (image.channels() > 1) {
        cv::cvtColor(image, image, CV_BGR2GRAY);
      }
      int size = _block_size() * 2 + 1;
      int method = method_() == 0 ? cv::ADAPTIVE_THRESH_GAUSSIAN_C
                                  : cv::ADAPTIVE_THRESH_MEAN_C;
      int type =
          threshold_type_() == 0 ? cv::THRESH_BINARY : cv::THRESH_BINARY_INV;
      cv::adaptiveThreshold(image, image, 255, method, type, size, c_param_());
    }
  }

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  Parameter<bool> enable_;

  RangedParameter<int> method_, threshold_type_, _block_size;

  RangedParameter<double> c_param_;
};

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_INRANGE_H_
