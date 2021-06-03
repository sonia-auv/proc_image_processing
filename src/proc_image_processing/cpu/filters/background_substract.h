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

#ifndef PROVIDER_VISION_FILTERS_BACKGROUND_SUBSTRACT_H_
#define PROVIDER_VISION_FILTERS_BACKGROUND_SUBSTRACT_H_

#include <filters/filter.h>
#include <memory>

namespace proc_image_processing {

// see http://docs.opencv.org/modules/imgproc/doc/filtering.html
// for more detail on how and why
//
// This is a program to execute image filter other than erode, dilate and
// morphologicalEx. Those are more blur function than pixelizer
// settings are for the differents type of filters, and does not apply to all
class BackgroundSubstract : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<BackgroundSubstract>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit BackgroundSubstract(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        enable_("Enable", false, &parameters_),
        show_blurred_("Show_blurred", false, &parameters_),
        blur_size_("Blur_size", 255, 0, 1000, &parameters_),
        sigma_("Sigma", 10, 0, 100, &parameters_) {
    SetName("BackgroundSubstract");
  }

  virtual ~BackgroundSubstract() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void Execute(cv::Mat &image) {
    if (enable_()) {
      std::vector<cv::Mat> channels;
      split(image, channels);
      cv::Mat b = channels[0];
      cv::Mat g = channels[1];
      cv::Mat r = channels[2];
      cv::Mat blurB, blurG, blurR;
      cv::blur(b, blurB,
               cv::Size(blur_size_.GetValue(), blur_size_.GetValue()));
      cv::blur(g, blurG,
               cv::Size(blur_size_.GetValue(), blur_size_.GetValue()));
      cv::blur(r, blurR,
               cv::Size(blur_size_.GetValue(), blur_size_.GetValue()));
      if (show_blurred_()) {
        b = blurB;
        g = blurG;
        r = blurR;
      } else {
        b = b - blurB;
        g = g - blurG;
        r = r - blurR;
      }
      channels[0] = b;
      channels[1] = g;
      channels[2] = r;
      cv::merge(channels, image);
    }
  }

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  Parameter<bool> enable_, show_blurred_;
  RangedParameter<int> blur_size_, sigma_;
};

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_BLURR_H_
