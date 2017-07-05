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

#ifndef PROVIDER_VISION_FILTERS_CONTRAST_BRIGHTNESS_H_
#define PROVIDER_VISION_FILTERS_CONTRAST_BRIGHTNESS_H_

#include <proc_image_processing/algorithm/general_function.h>
#include <proc_image_processing/filters/filter.h>
#include <memory>

namespace proc_image_processing {

// Filter showing planes of different analysis (gray, _hsi, _bgr)
// No threshold
class ContrastBrightness : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<ContrastBrightness>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit ContrastBrightness(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        enable_("enable", false, &parameters_),
        contrast_("Contrast", 0, 0, 256, &parameters_,
                 "Contrast"),
        brightness_("Brightness", 0, 0, 256, &parameters_,
                 "Set Brightness"),
        rows_(0),
        cols_(0) {
    SetName("ContrastBrightness");
  }

  virtual ~ContrastBrightness() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void Execute(cv::Mat &image) {
    if (enable_()) {
      if (CV_MAT_CN(image.type()) != 3) {
        return;
      }

      rows_ = image.rows;
      cols_ = image.cols;
      // Set final matrices

      cv::Mat final = cv::Mat::zeros(rows_, cols_, CV_8UC1);

      // Replace with new images

      for( int y = 0; y < image.rows; y++ )
      { for( int x = 0; x < image.cols; x++ )
        {
          final.at<uchar>(y,x) = cv::saturate_cast<uchar>( contrast_()*( image.at<uchar>(y,x) ) + brightness_() );

        }
      }

      final.copyTo(image);
    }
  }

 private:
  //============================================================================
  // P R I V A T E   M E T H O D S


  Parameter<bool> enable_;
  RangedParameter<double> contrast_, brightness_;
  // Color matrices

  int rows_;
  int cols_;
};

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_CONTRAST_BRIGHTNESS_H_
