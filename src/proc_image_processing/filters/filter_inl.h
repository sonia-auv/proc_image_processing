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
#error This file may only be included from filter.h
#endif

#include <sonia_common/macros.h>
#include <proc_image_processing/server/target.h>

namespace proc_image_processing {

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
ATLAS_INLINE Filter::Filter(const GlobalParamHandler &globalParams)
    : global_params_(const_cast<GlobalParamHandler &>(globalParams)),
      // enable_("Enable", false, &parameters_),
      // Explicit construction not needed here... Just reminder it exist.
      parameters_() {}

//------------------------------------------------------------------------------
//
ATLAS_INLINE const std::vector<ParameterInterface *> &Filter::GetParameters()
    const {
  return parameters_;
}

//------------------------------------------------------------------------------
//
ATLAS_INLINE std::string Filter::GetParameterValue(const std::string &name) {
  std::string returnString("");
  for (int i = 0; i < int(parameters_.size()); i++) {
    // Here we give it a local value to limit the
    // access to the vector (optimisation)
    ParameterInterface *param = parameters_[i];

    // nullptr element should never append since the vector's object
    // are added by the said object (which cannot be null if
    // it gets to the constructor) and which are member of
    // the filter class (which mean they are alive as long as the
    // filter is alive)...
    if (param != nullptr) {
      // Is it the param we are searching
      if (param->GetName() == name) {
        returnString = param->ToString();
      }
    }
  }
  return returnString;
}

//------------------------------------------------------------------------------
//
ATLAS_INLINE void Filter::SetParameterValue(const std::string &name,
                                            std::string value) {
  for (int i = 0; i < int(parameters_.size()); i++) {
    // Here we give it a local value to limit the
    // access to the vector (optimisation)
    ParameterInterface *param = parameters_[i];

    if (param != nullptr && param->GetName() == name) {
      param->SetStringValue(value);
    }
  }
}

//------------------------------------------------------------------------------
//
ATLAS_INLINE const std::string Filter::GetName() { return name_; }

//------------------------------------------------------------------------------
//
ATLAS_INLINE void Filter::SetName(const std::string &name) { name_ = name; }

//------------------------------------------------------------------------------
//
ATLAS_INLINE void Filter::NotifyTarget(const Target &target) {
  global_params_.addTarget(target);
}

//------------------------------------------------------------------------------
//
ATLAS_INLINE void Filter::GlobalParamInteger(const std::string &name,
                                             const int value, const int min,
                                             const int max) {
  global_params_.addParam(
      new RangedParameter<int>(name, value, max, min, &parameters_));
}

//------------------------------------------------------------------------------
//
ATLAS_INLINE void Filter::GlobalParamDouble(const std::string &name,
                                            const double value,
                                            const double min,
                                            const double max) {
  global_params_.addParam(
      new RangedParameter<double>(name, value, max, min, &parameters_));
}

//------------------------------------------------------------------------------
//
ATLAS_INLINE void Filter::GlobalParamBoolean(const std::string &name,
                                             const bool value) {
  global_params_.addParam(new Parameter<bool>(name, value, &parameters_));
}

//------------------------------------------------------------------------------
//
ATLAS_INLINE void Filter::GlobalParamString(const std::string &name,
                                            const std::string &value) {
  global_params_.addParam(
      new Parameter<std::string>(name, value, &parameters_));
}

}  // namespace proc_image_processing
