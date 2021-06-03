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

#ifndef PROVIDER_VISION_FILTERS_TEST_FILTER_H_
#define PROVIDER_VISION_FILTERS_TEST_FILTER_H_

#include <filters/filter.h>
#include <server/target.h>
#include <memory>

namespace proc_image_processing {

class TestFilter : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<TestFilter>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit TestFilter(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        enable_("Enable", true, &parameters_),
        x_("X", 0, -512, 512, &parameters_),
        y_("Y", 0, -512, 512, &parameters_),
        w_("Width", 200, 0, 1024, &parameters_),
        h_("Height", 200, 0, 1024, &parameters_),
        angle_("Angle", 0, 0, 360, &parameters_),
        header_("Header", "test", &parameters_),
        specField1_("SpecialField_1", "sf1", &parameters_),
        specField2_("SpecialField_2", "sf2", &parameters_) {
    SetName("TestFilter");
  }

  virtual ~TestFilter() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void init() {}

  virtual void Execute(cv::Mat &image) {
    if (enable_()) {
      target_.SetTarget("test_filter", x_.GetValue() - 1000 / 2, y_.GetValue(),
                        w_.GetValue(), h_.GetValue(), angle_.GetValue(), 1000,
                        -1000 - (1000 / 2), specField1_.GetValue(),
                        specField2_.GetValue());

      NotifyTarget(target_);
    }
  }

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  Parameter<bool> enable_;
  RangedParameter<int> x_, y_, w_, h_, angle_;
  Parameter<std::string> header_, specField1_, specField2_;

  Target target_;
};

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_TEST_FILTER_H_
