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

#ifndef PROVIDER_VISION_FILTER_H_
#define PROVIDER_VISION_FILTER_H_

#include <proc_image_processing/cpu/server/global_param_handler.h>
#include <proc_image_processing/cpu/server/parameter.h>
#include <proc_image_processing/cpu/server/ranged_parameter.h>
#include <proc_image_processing/cpu/server/target.h>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace proc_image_processing {

class Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Filter>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit Filter(const GlobalParamHandler &globalParams);

  virtual ~Filter() = default;

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void Execute(cv::Mat &image) = 0;

  // Name of the filter handlers
  inline const std::string GetName();

  inline void SetName(const std::string &name);

  const std::vector<ParameterInterface *> &GetParameters() const;

  std::string GetParameterValue(const std::string &name);

  void SetParameterValue(const std::string &name, std::string value);

  // Wrapper for a call to _globalParms
  // NotifyTarget, to be put on the result topic
  void NotifyTarget(const Target &target);

  void GlobalParamInteger(const std::string &name, const int value,
                          const int min, const int max);

  void GlobalParamDouble(const std::string &name, const double value,
                         const double min, const double max);

  void GlobalParamBoolean(const std::string &name, const bool value);

  void GlobalParamString(const std::string &name, const std::string &value);

 protected:
  //============================================================================
  // P R O T E C T E D   M E M B E R S

  GlobalParamHandler &global_params_;

  std::vector<ParameterInterface *> parameters_;

  // Useful to identify the filter.
  std::string name_;
};

}  // namespace proc_image_processing

#include "filter_inl.h"

#endif  // PROVIDER_VISION_FILTER_H_
