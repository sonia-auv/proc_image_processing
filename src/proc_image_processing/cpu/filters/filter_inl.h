/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROVIDER_VISION_FILTER_H_
#error This file may only be included from filter.h
#endif

#include <sonia_common/macros.h>
#include <proc_image_processing/cpu/server/target.h>

namespace proc_image_processing {

  ATLAS_INLINE Filter::Filter(const GlobalParamHandler& globalParams)
    : global_params_(const_cast<GlobalParamHandler&>(globalParams)),
    // enable_("Enable", false, &parameters_),
    // Explicit construction not needed here... Just reminder it exist.
    parameters_() {
  }

  ATLAS_INLINE const std::vector<ParameterInterface*>& Filter::GetParameters()
    const {
    return parameters_;
  }

  ATLAS_INLINE std::string Filter::GetParameterValue(const std::string& name) {
    std::string returnString("");
    for (int i = 0; i < int(parameters_.size()); i++) {
      // Here we give it a local value to limit the
      // access to the vector (optimisation)
      ParameterInterface* param = parameters_[i];

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

  ATLAS_INLINE void Filter::SetParameterValue(const std::string& name,
    std::string value) {
    for (int i = 0; i < int(parameters_.size()); i++) {
      // Here we give it a local value to limit the
      // access to the vector (optimisation)
      ParameterInterface* param = parameters_[i];

      if (param != nullptr && param->GetName() == name) {
        param->SetStringValue(value);
      }
    }
  }

  ATLAS_INLINE const std::string Filter::GetName() { return name_; }

  ATLAS_INLINE void Filter::SetName(const std::string& name) { name_ = name; }

  ATLAS_INLINE void Filter::NotifyTarget(const Target& target) {
    global_params_.addTarget(target);
  }

  ATLAS_INLINE void Filter::GlobalParamInteger(const std::string& name,
    const int value, const int min,
    const int max) {
    global_params_.addParam(
      new RangedParameter<int>(name, value, max, min, &parameters_));
  }

  ATLAS_INLINE void Filter::GlobalParamDouble(const std::string& name,
    const double value,
    const double min,
    const double max) {
    global_params_.addParam(
      new RangedParameter<double>(name, value, max, min, &parameters_));
  }

  ATLAS_INLINE void Filter::GlobalParamBoolean(const std::string& name,
    const bool value) {
    global_params_.addParam(new Parameter<bool>(name, value, &parameters_));
  }

  ATLAS_INLINE void Filter::GlobalParamString(const std::string& name,
    const std::string& value) {
    global_params_.addParam(
      new Parameter<std::string>(name, value, &parameters_));
  }

}  // namespace proc_image_processing
