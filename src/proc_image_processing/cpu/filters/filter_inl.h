#ifndef PROC_IMAGE_PROCESSING_FILTER_H_
#error This file may only be included from filter.h
#endif

#include <sonia_common/macros.h>
#include <proc_image_processing/cpu/server/target.h>
#include "filter.h"
#include "ros/console.h"


namespace proc_image_processing {

    inline std::string Filter::getName() { return name_; }

    inline const std::vector<ParameterInterface *> &Filter::getParameters() const { return parameters_; }

    inline std::string Filter::getParameterValue(const std::string &name) {
        std::string returnString;
        for (auto param : parameters_) {
            // Here we give it a local value to limit the
            // access to the vector (optimisation)
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

    inline void Filter::notify(const Target &target) { global_param_handler_.addTarget(target); }

    inline void Filter::setName(const std::string &name) { name_ = name; }

    inline void Filter::setParameterValue(const std::string &name, const std::string &value) {
        for (auto param : parameters_) {
            // Here we give it a local value to limit the
            // access to the vector (optimisation)
            if (param != nullptr && param->getName() == name) {
                param->setStringValue(value);
            }
        }
    }

    inline const GlobalParameterHandler &Filter::getGlobalParamHandler() const {
        return global_param_handler_;
    }

}  // namespace proc_image_processing
