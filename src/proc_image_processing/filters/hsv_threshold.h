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

#ifndef PROVIDER_VISION_FILTERS_HSV_THRESHOLD_H_
#define PROVIDER_VISION_FILTERS_HSV_THRESHOLD_H_

#include <proc_image_processing/algorithm/general_function.h>
#include <proc_image_processing/filters/filter.h>
#include <memory>

namespace proc_image_processing {

// Filter showing planes of different analysis (gray, _hsi, _bgr)
// No threshold
class HSVThreshold : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<HSVThreshold>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit HSVThreshold(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        enable_("enable", false, &parameters_),
        hue_min_("Hue Min", 0, 0, 255, &parameters_,
                 "Minimum Hue to threshold. Keep values higher or equal to this value."),
        hue_max_("Hue Max", 255, 0, 255, &parameters_,
                 "Maximum Hue to threshold. Keep values lower or equal to this value."),
        saturation_min_("saturation Min", 0, 0, 255, &parameters_,
                 "Minimum saturation to threshold. Keep values higher or equal to this value."),
        saturation_max_("saturation Max", 255, 0, 255, &parameters_,
                 "Maximum saturation to threshold. Keep values lower or equal to this value."),
        value_min_("value Min", 0, 0, 255, &parameters_,
                 "Minimum value to threshold. Keep values higher or equal to this value."),
        value_max_("value Max", 255, 0, 255, &parameters_,
                 "Maximum value to threshold. Keep values lower or equal to this value."),
        rows_(0),
        cols_(0) {
    SetName("HSVThreshold");
  }

  virtual ~HSVThreshold() {}

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
      cv::Mat hue = cv::Mat::zeros(rows_, cols_, CV_8UC1);
      cv::Mat hue_res1 = cv::Mat::zeros(rows_, cols_, CV_8UC1);
      cv::Mat hue_res2 = cv::Mat::zeros(rows_, cols_, CV_8UC1);
      cv::Mat hue_res = cv::Mat::zeros(rows_, cols_, CV_8UC1);
      cv::Mat saturation = cv::Mat::zeros(rows_, cols_, CV_8UC1);
      cv::Mat saturation_res = cv::Mat::zeros(rows_, cols_, CV_8UC1);
      cv::Mat value = cv::Mat::zeros(rows_, cols_, CV_8UC1);
      cv::Mat value_res = cv::Mat::zeros(rows_, cols_, CV_8UC1);
      cv::Mat final = cv::Mat::zeros(rows_, cols_, CV_8UC1);

      // Replace with new images

      channel_vec_ = GetColorPlanes(image);
        set_image(H_INDEX, hue);
        set_image(S_INDEX, saturation);
        set_image(V_INDEX, value);

      cv::threshold(hue,hue_res1,hue_min_(),255,cv::THRESH_BINARY);
      cv::threshold(hue,hue_res2,hue_max_(),255,cv::THRESH_BINARY_INV);
      cv::bitwise_and(hue_res1, hue_res2, hue_res);
      cv::inRange(saturation,cv::Scalar(saturation_min_()), cv::Scalar(saturation_max_()), saturation_res);
      cv::inRange(value,cv::Scalar(value_min_()), cv::Scalar(value_max_()), value_res);

      cv::bitwise_and(hue_res, saturation_res, final);
      cv::bitwise_and(final, value_res, final);



      final.copyTo(image);
    }
  }

 private:
  //============================================================================
  // P R I V A T E   M E T H O D S

  void set_image(const int choice, cv::Mat &out) {
    // Thightly couple with parameter, but putting safety...
    int index = choice < 0 ? 0 : (choice > 6 ? 6 : choice);
    channel_vec_[index].copyTo(out);

  }

  //============================================================================
  // P R I V A T E   M E M B E R S

  Parameter<bool> enable_;
  RangedParameter<int> hue_min_, hue_max_, saturation_min_, saturation_max_, value_min_, value_max_;
  // Color matrices
  std::vector<cv::Mat> channel_vec_;

  int rows_;
  int cols_;
    const int H_INDEX = 4;
    const int S_INDEX = 5;
    const int V_INDEX = 6;
};

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_HSV_THRESHOLD_H_
