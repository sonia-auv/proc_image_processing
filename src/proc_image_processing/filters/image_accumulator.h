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

#ifndef PROVIDER_VISION_FILTERS_IMAGE_ACCUMULATOR_H_
#define PROVIDER_VISION_FILTERS_IMAGE_ACCUMULATOR_H_

#include <proc_image_processing/algorithm/image_accumulator_buffer.h>
#include <proc_image_processing/algorithm/performance_evaluator.h>
#include <proc_image_processing/filters/filter.h>
#include <memory>

namespace proc_image_processing {

class ImageAccumulator : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<ImageAccumulator>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit ImageAccumulator(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        accumulator_(3, cv::Size(0, 0), CV_8UC1),
        enable_("Enable", false, &parameters_),
        nb_image_("NB_of_images", 3, 1, 20, &parameters_),
        method_("Method_to_use", 1, 0, 2, &parameters_,
                "Method: 1=SameWeight, 2=Adding50Percent, 3=Adjusted"),
        last_size_(0, 0),
        last_method_(CV_8UC1),
        last_type_(0),
        last_nb_image_(3) {
    SetName("ImageAccumulator");
  }

  virtual ~ImageAccumulator() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void Execute(cv::Mat &image) {
    if (enable_()) {
      // Is there any change in the type of images
      // we input to the accumulator?
      // If yes, reset it.
      if (last_type_ != image.type() || last_method_ != method_() ||
          last_nb_image_ != nb_image_() || last_size_ != image.size()) {
        accumulator_.ResetBuffer(nb_image_(), image.size(), image.type());

        last_nb_image_ = nb_image_();
        last_size_ = image.size();

        switch (method_()) {
          case 0:
            accumulator_.SetAverageMethod(
                ImageAccumulatorBuffer::ACC_ALL_SAME_WEIGHT);
            break;
          case 1:
            accumulator_.SetAverageMethod(
                ImageAccumulatorBuffer::ACC_50_PERCENT);
            break;
          case 2:
            accumulator_.SetAverageMethod(
                ImageAccumulatorBuffer::ACC_ADJUST_WEIGHT);
            break;
          default:
            accumulator_.SetAverageMethod(
                ImageAccumulatorBuffer::ACC_ALL_SAME_WEIGHT);
            break;
        }
        last_method_ = method_();
        last_type_ = image.type();
      }
      // Add the newest frame
      accumulator_.AddImage(image);
      // Change the input for the newest averaging.
      accumulator_.GetImage(image);
    }
  }

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  ImageAccumulatorBuffer accumulator_;
  Parameter<bool> enable_;
  RangedParameter<int> nb_image_, method_;
  // Here we need some sorte of remembering
  // so we can reset the accumulator on
  // param changing.
  cv::Size last_size_;
  int last_method_, last_type_, last_nb_image_;
};

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_IMAGE_ACCUMULATOR_H_
