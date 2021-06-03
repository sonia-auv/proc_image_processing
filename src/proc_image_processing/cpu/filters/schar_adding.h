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

#ifndef PROVIDER_VISION_FILTERS_SCHARR_ADDING_H_
#define PROVIDER_VISION_FILTERS_SCHARR_ADDING_H_

#include <algorithm/general_function.h>
#include <filters/filter.h>
#include <memory>

namespace proc_image_processing {

class ScharrAdding : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<ScharrAdding>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit ScharrAdding(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        enable_("Enable", false, &parameters_),
        run_small_image_("Run_small_image", true, &parameters_,
                         "Resize image to run on smaller image"),
        convert_to_uchar_("Convert_to_uchar", false, &parameters_),
        delta_("Delta", 0, 0, 255, &parameters_),
        scale_("Scale", 1, 0, 255, &parameters_),
        mean_multiplier_("Mean_multiplier", 1.0f, 0.0f, 10.0f, &parameters_),
        plane_blue_("Blue", false, &parameters_),
        plane_green_("Green", false, &parameters_),
        plane_red_("Red", false, &parameters_),
        plane_hue_("Hue", false, &parameters_),
        plane_saturation_("Saturation", false, &parameters_),
        plane_intensity_("Intensity", false, &parameters_),
        plane_gray_("Gray", false, &parameters_) {
    SetName("ScharrAdding");
  }

  virtual ~ScharrAdding() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void Execute(cv::Mat &image) {
    if (enable_()) {
      if (image.channels() != 3) return;
      if (run_small_image_()) {
        cv::resize(image, image, cv::Size(image.cols / 2, image.rows / 2));
      }

      std::vector<cv::Mat> colorPlanes = GetColorPlanes(image);
      cv::Mat sum = cv::Mat::zeros(image.rows, image.cols, CV_32FC1);

      if (plane_blue_()) cv::add(calcScharr(colorPlanes[0]), sum, sum);
      if (plane_green_()) cv::add(calcScharr(colorPlanes[1]), sum, sum);
      if (plane_red_()) cv::add(calcScharr(colorPlanes[2]), sum, sum);
      if (plane_hue_()) cv::add(calcScharr(colorPlanes[3]), sum, sum);
      if (plane_saturation_()) cv::add(calcScharr(colorPlanes[4]), sum, sum);
      if (plane_intensity_()) cv::add(calcScharr(colorPlanes[5]), sum, sum);
      if (plane_gray_()) cv::add(calcScharr(colorPlanes[6]), sum, sum);

      sum.copyTo(image);
      if (run_small_image_()) {
        cv::resize(image, image, cv::Size(image.cols * 2, image.rows * 2));
      }

      if (convert_to_uchar_() && image.channels() < 3) {
        cv::cvtColor(image, image, CV_GRAY2BGR);
      }
    }
  }

 private:
  //============================================================================
  // P R I V A T E   M E T H O D S

  cv::Mat calcScharr(const cv::Mat &img) {
    cv::Mat abs_x, scharrX, abs_y, scharrY, diff;

    cv::Scharr(img, scharrX, CV_32F, 1, 0, scale_(), delta_(),
               cv::BORDER_REPLICATE);
    cv::Scharr(img, scharrY, CV_32F, 0, 1, scale_(), delta_(),
               cv::BORDER_REPLICATE);
    cv::absdiff(scharrX, 0, scharrX);
    cv::absdiff(scharrY, 0, scharrY);

    cv::addWeighted(scharrX, 0.5, scharrY, 0.5, 0, diff, CV_32F);

    cv::Scalar mean = cv::mean(diff);
    cv::threshold(diff, diff, (mean[0] * mean_multiplier_()), 0,
                  CV_THRESH_TOZERO);

    return diff;
  }

  //============================================================================
  // P R I V A T E   M E M B E R S

  // _run_small_image accelerate the pipeline by
  // reducing the image size by two (in each direction)
  // so that the scharr computation does not take to much time
  // when multiple images.
  Parameter<bool> enable_, run_small_image_, convert_to_uchar_;
  // _mean_multiplier act as threshold for noise.
  // When set, it remove everything under the mean to keep only
  // proeminent contours.
  RangedParameter<double> delta_, scale_, mean_multiplier_;
  Parameter<bool> plane_blue_, plane_green_, plane_red_;
  Parameter<bool> plane_hue_, plane_saturation_, plane_intensity_;
  Parameter<bool> plane_gray_;
};

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_SCHARR_ADDING_H_
