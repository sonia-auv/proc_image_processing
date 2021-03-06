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

#ifndef PROVIDER_VISION_FILTERS_MORPHOLOGY_H_
#define PROVIDER_VISION_FILTERS_MORPHOLOGY_H_

#include <proc_image_processing/filters/filter.h>
#include <memory>

namespace proc_image_processing {

class Morphology : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Morphology>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit Morphology(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        enable_("Enable", false, &parameters_),
        morph_type_("Morphology_type", 0, 0, 4, &parameters_,
                    "0=Gradient, 1=TopHat, 2=BlackHat, 3=Opening, 4=Closing"),
        kernel_type_("Kernel_type", 0, 0, 2, &parameters_,
                     "0=Rect, 1=Elipse, 2=Cross"),
        iteration_("Iteration", 1, 1, 20, &parameters_),
        kernel_size_("Kernel_size", 1, 1, 40, &parameters_),
        anchor_(-1, -1) {
    SetName("Morphology");
  }

  virtual ~Morphology() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void Execute(cv::Mat &image) {
    if (enable_()) {
      if (image.channels() > 1) {
        cv::cvtColor(image, image, CV_BGR2GRAY);
      }

      // Kernel selection
      int kernelType;
      switch (kernel_type_()) {
        case 0:
          kernelType = cv::MORPH_RECT;
          break;
        case 1:
          kernelType = cv::MORPH_ELLIPSE;
          break;
        case 2:
          kernelType = cv::MORPH_CROSS;
          break;
        default:
          kernelType = cv::MORPH_RECT;
          break;
      }

      // Creating the kernel
      cv::Mat kernel = cv::getStructuringElement(
          kernelType, cv::Size(kernel_size_() * 2 + 1, kernel_size_() * 2 + 1),
          anchor_);

      // Selecting with _morph_type wich operation to use
      switch (morph_type_()) {
        case 0:
          cv::morphologyEx(image, image, cv::MORPH_GRADIENT, kernel, anchor_,
                           iteration_(), CV_8U);
          break;
        case 1:
          cv::morphologyEx(image, image, cv::MORPH_TOPHAT, kernel, anchor_,
                           iteration_(), CV_8U);
          break;
        case 2:
          cv::morphologyEx(image, image, cv::MORPH_BLACKHAT, kernel, anchor_,
                           iteration_(), CV_8U);
          break;
        case 3:
          cv::morphologyEx(image, image, cv::MORPH_OPEN, kernel, anchor_,
                           iteration_(), CV_8U);
          break;
        case 4:
          cv::morphologyEx(image, image, cv::MORPH_CLOSE, kernel, anchor_,
                           iteration_(), CV_8U);
          break;
      }
    }
  }

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  Parameter<bool> enable_;
  RangedParameter<int> morph_type_, kernel_type_, iteration_, kernel_size_;
  const cv::Point anchor_;
};

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_MORPHOLOGY_H_
