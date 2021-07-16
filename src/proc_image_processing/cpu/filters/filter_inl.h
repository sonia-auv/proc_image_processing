/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROC_IMAGE_PROCESSING_FILTER_H_
#error This file may only be included from filter.h
#endif

#include <sonia_common/macros.h>
#include <proc_image_processing/cpu/server/target.h>

namespace proc_image_processing {

    ATLAS_INLINE Filter::Filter(const GlobalParamHandler &globalParams)
            : global_params_(const_cast<GlobalParamHandler &>(globalParams)),
            // enable_("Enable", false, &parameters_),
            // Explicit construction not needed here... Just reminder it exist.
              parameters_() {
    }

    ATLAS_INLINE const std::vector<ParameterInterface *> &Filter::getParameters()
    const {
        return parameters_;
    }

    ATLAS_INLINE std::string Filter::getParameterValue(const std::string &name) {
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
                if (param->getName() == name) {
                    returnString = param->toString();
                }
            }
        }
        return returnString;
    }

    ATLAS_INLINE void Filter::setParameterValue(const std::string &name,
                                                std::string value) {
        for (int i = 0; i < int(parameters_.size()); i++) {
            // Here we give it a local value to limit the
            // access to the vector (optimisation)
            ParameterInterface *param = parameters_[i];

            if (param != nullptr && param->getName() == name) {
                param->setStringValue(value);
            }
        }
    }

    ATLAS_INLINE const std::string Filter::getName() { return name_; }

    ATLAS_INLINE void Filter::setName(const std::string &name) { name_ = name; }

    ATLAS_INLINE void Filter::notify(const Target &target) {
        global_params_.addTarget(target);
    }

    ATLAS_INLINE void Filter::addGlobalParameter(const std::string &name,
                                                 const int value, const int min,
                                                 const int max) {
        global_params_.addParameter(
                new RangedParameter<int>(name, value, max, min, &parameters_));
    }

    ATLAS_INLINE void Filter::addGlobalParameter(const std::string &name,
                                                 const double value,
                                                 const double min,
                                                 const double max) {
        global_params_.addParameter(
                new RangedParameter<double>(name, value, max, min, &parameters_));
    }

    ATLAS_INLINE void Filter::addGlobalParameter(const std::string &name,
                                                 const bool value) {
        global_params_.addParameter(new Parameter<bool>(name, value, &parameters_));
    }

    ATLAS_INLINE void Filter::addGlobalParameter(const std::string &name,
                                                 const std::string &value) {
        global_params_.addParameter(
                new Parameter<std::string>(name, value, &parameters_));
    }

}  // namespace proc_image_processing
