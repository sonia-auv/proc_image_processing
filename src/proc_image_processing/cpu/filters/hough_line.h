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

#ifndef PROVIDER_VISION_FILTERS_HOUGH_LINE_H_
#define PROVIDER_VISION_FILTERS_HOUGH_LINE_H_

#include "filter.h"
#include <memory>

namespace proc_image_processing {

class HoughLine : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<HoughLine>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit HoughLine(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        enable_("Enable", false, &parameters_),
        rho_("Rho", 1.0f, 0.0f, 1000.0f, &parameters_),
        theta_("Theta", 1.0f, 0.0f, 1000.0f, &parameters_),
        min_length_("Min_length", 1, 0, 1000, &parameters_),
        max_gap_("Max_gap", 1, 0, 1000, &parameters_),
        threshold_("Threshold", 1, 0, 1000, &parameters_) {
    SetName("HoughLine");
  }

  virtual ~HoughLine() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void Execute(cv::Mat &image) {
    if (enable_()) {
      if (image.channels() > 1) {
        cv::cvtColor(image, image, CV_BGR2GRAY);
      }

      std::vector<cv::Vec4i> lines;
      cv::HoughLinesP(image, lines, rho_(), theta_(), threshold_(),
                      min_length_(), max_gap_());

      cv::Mat drawing_image(image.rows, image.cols, CV_8UC3,
                            cv::Scalar::all(0));
      for (const auto &line : lines) {
        cv::line(drawing_image, cv::Point(line[0], line[1]),
                 cv::Point(line[2], line[3]), cv::Scalar(255, 255, 255), 3);
      }
      cv::cvtColor(drawing_image, image, CV_BGR2GRAY);
    }
  }

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  Parameter<bool> enable_;
  RangedParameter<double> rho_, theta_, min_length_, max_gap_;
  RangedParameter<int> threshold_;
};

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_SOBEL_H_
