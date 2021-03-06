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

#ifndef PROVIDER_VISION_FILTERS_SHCARR_H_
#define PROVIDER_VISION_FILTERS_SHCARR_H_

#include <proc_image_processing/filters/filter.h>
#include <memory>

namespace proc_image_processing {

class Scharr : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Scharr>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit Scharr(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        enable_("Enable", false, &parameters_),
        convert_to_uchar_("Convert_to_uchar", true, &parameters_),
        use_pixel_intensity_correction_("use_pixel_intensity_correction", false,
                                        &parameters_),
        delta_("Delta", 0, 0, 255, &parameters_),
        scale_("Scale", 1, 0, 255, &parameters_),
        power_pixel_correction_("pixel_correction_power", 1, -10, 10,
                                &parameters_) {
    SetName("Scharr");
  }

  virtual ~Scharr() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void Execute(cv::Mat &image) {
    if (enable_()) {
      if (image.channels() > 1) {
        cv::cvtColor(image, image, CV_BGR2GRAY);
      }
      cv::Mat scharrX, scharrY;
      cv::Scharr(image, scharrX, CV_32F, 1, 0, scale_(), delta_(),
                 cv::BORDER_REPLICATE);
      cv::Scharr(image, scharrY, CV_32F, 0, 1, scale_(), delta_(),
                 cv::BORDER_REPLICATE);
      cv::absdiff(scharrX, 0, scharrX);
      cv::absdiff(scharrY, 0, scharrY);

      cv::addWeighted(scharrX, 0.5, scharrY, 0.5, 0, image, CV_32F);

      if (use_pixel_intensity_correction_()) {
        for (int y = 0; y < image.rows; y++) {
          float *ptr = image.ptr<float>(y);
          for (int x = 0; x < image.cols; x++) {
            ptr[x] = pow(ptr[x], power_pixel_correction_());
          }
        }
      }

      if (convert_to_uchar_()) {
        cv::convertScaleAbs(image, image);
      }
    }
  }

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  Parameter<bool> enable_, convert_to_uchar_, use_pixel_intensity_correction_;
  RangedParameter<double> delta_, scale_, power_pixel_correction_;
};

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_SHCARR_H_
