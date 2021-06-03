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

#ifndef PROVIDER_VISION_FILTERS_STATS_THRESHOLD_H_
#define PROVIDER_VISION_FILTERS_STATS_THRESHOLD_H_

#include <filters/filter.h>
#include <memory>

namespace proc_image_processing {

class StatsThreshold : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<StatsThreshold>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit StatsThreshold(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        enable_("Enable", false, &parameters_),
        min_thresh_("Min_thresh", 0, 0, 255, &parameters_),
        mean_multiplier_("Mean_multiplier", 1, -10, 10, &parameters_),
        std_dev_multiplier_("Standard_deviation_multiplier", 1, -10, 10,
                            &parameters_) {
    SetName("StatsThreshold");
  }

  virtual ~StatsThreshold() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void Execute(cv::Mat &image) {
    if (enable_()) {
      if (image.channels() > 1) {
        cv::cvtColor(image, image, CV_BGR2GRAY);
      }
      if (image.depth() != CV_8U) {
        image.convertTo(image, CV_8U);
      }
      cv::Scalar mean, stdDev;

      cv::meanStdDev(image, mean, stdDev);
      int thresh_val =
          mean[0] * mean_multiplier_() + stdDev[0] * std_dev_multiplier_();
      thresh_val = thresh_val < min_thresh_() ? min_thresh_() : thresh_val;
      cv::threshold(image, image, thresh_val, 255, CV_THRESH_BINARY);
    }
  }

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  Parameter<bool> enable_;
  RangedParameter<int> min_thresh_;
  RangedParameter<double> mean_multiplier_, std_dev_multiplier_;
};

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_STATS_THRESHOLD_H_
