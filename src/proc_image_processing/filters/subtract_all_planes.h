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

#ifndef PROVIDER_VISION_FILTERS_SUBTRACT_ALL_PLANES_H_
#define PROVIDER_VISION_FILTERS_SUBTRACT_ALL_PLANES_H_

#include <proc_image_processing/algorithm/general_function.h>
#include <proc_image_processing/filters/filter.h>
#include <memory>

namespace proc_image_processing {

// Filter showing planes of different analysis (gray, _hsi, _bgr)
// No threshold
class SubtractAllPlanes : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<SubtractAllPlanes>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit SubtractAllPlanes(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        enable_("enable", false, &parameters_),
        plane_one_("Plane_1", 1, 0, 7, &parameters_,
                   "0=None, 1=Blue, 2=Green, 3=Red, 4=Hue, 5=Saturation, "
                   "6=Intensity, 7=Gray"),
        plane_two_("Plane_2", 1, 0, 7, &parameters_,
                   "0=None, 1=Blue, 2=Green, 3=Red, 4=Hue, 5=Saturation, "
                   "6=Intensity, 7=Gray"),
        plane_three_("Plane_3", 1, 0, 7, &parameters_,
                     "0=None, 1=Blue, 2=Green, 3=Red, 4=Hue, 5=Saturation, "
                     "6=Intensity, 7=Gray"),
        invert_one_("Invert_plane_1", false, &parameters_),
        invert_two_("Invert_plane_2", false, &parameters_),
        invert_three_("Invert_plane_3", false, &parameters_),
        weight_one_("Weight_Plane_1", 1.0, 0.0, 10.0, &parameters_),
        weight_two_("Weight_Plane_2", 1.0, 0.0, 10.0, &parameters_),
        weight_three_("Weight_Plane_3", 1.0, 0.0, 10.0, &parameters_),
        rows_(0),
        cols_(0) {
    SetName("SubtractAllPlanes");
  }

  virtual ~SubtractAllPlanes() {}

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
      cv::Mat zero = cv::Mat::zeros(rows_, cols_, CV_8UC1);
      cv::Mat one = cv::Mat::zeros(rows_, cols_, CV_8UC1);
      cv::Mat two = cv::Mat::zeros(rows_, cols_, CV_8UC1);
      cv::Mat three = cv::Mat::zeros(rows_, cols_, CV_8UC1);
      cv::Mat final = cv::Mat::zeros(rows_, cols_, CV_8UC1);

      // Replace with new images

      channel_vec_ = GetColorPlanes(image);

      // Set subtraction
      if (plane_one_() != 0)
        set_image(plane_one_() - 1, one, weight_one_(), invert_one_());

      if (plane_two_() != 0)
        set_image(plane_two_() - 1, two, weight_two_(), invert_two_());

      if (plane_three_() != 0)
        set_image(plane_three_() - 1, three, weight_three_(), invert_three_());

      cv::subtract(one, two, final);
      cv::subtract(final, three, final);

      final.copyTo(image);
    }
  }

 private:
  //============================================================================
  // P R I V A T E   M E T H O D S

  void set_image(const int choice, cv::Mat &out, const double weight,
                 const bool inverse) {
    cv::Mat two_five_five(rows_, cols_, CV_16SC1, cv::Scalar(255));
    cv::Mat one(rows_, cols_, CV_16SC1, cv::Scalar(1));

    // Thightly couple with parameter, but putting safety...
    int index = choice < 0 ? 0 : (choice > 6 ? 6 : choice);
    channel_vec_[index].copyTo(out);

    if (inverse) {
      InverseImage(out, out);
    }
    cv::multiply(out, one, out, weight, CV_8UC1);
  }

  //============================================================================
  // P R I V A T E   M E M B E R S

  Parameter<bool> enable_;
  RangedParameter<int> plane_one_, plane_two_, plane_three_;
  Parameter<bool> invert_one_, invert_two_, invert_three_;
  RangedParameter<double> weight_one_, weight_two_, weight_three_;

  // Color matrices
  std::vector<cv::Mat> channel_vec_;

  int rows_;
  int cols_;
};

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_SUBTRACT_ALL_PLANES_H_
