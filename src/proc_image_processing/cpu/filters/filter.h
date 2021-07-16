/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROC_IMAGE_PROCESSING_FILTER_H_
#define PROC_IMAGE_PROCESSING_FILTER_H_

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
        using Ptr = std::shared_ptr<Filter>;

        explicit Filter(const GlobalParamHandler &globalParams);

        virtual ~Filter() = default;

        virtual void apply(cv::Mat &image) = 0;

        // Name of the filter handlers
        inline const std::string getName();

        inline void setName(const std::string &name);

        const std::vector<ParameterInterface *> &getParameters() const;

        std::string getParameterValue(const std::string &name);

        void setParameterValue(const std::string &name, std::string value);

        // Wrapper for a call to _globalParms
        // notify, to be put on the result topic
        void notify(const Target &target);

        void addGlobalParameter(const std::string &name, int value, int min, int max);

        void addGlobalParameter(const std::string &name, double value, double min, double max);

        void addGlobalParameter(const std::string &name, bool value);

        void addGlobalParameter(const std::string &name, const std::string &value);

    protected:
        GlobalParamHandler &global_params_;

        std::vector<ParameterInterface *> parameters_;

        // Useful to identify the filter.
        std::string name_;
    };

}  // namespace proc_image_processing

#include "filter_inl.h"

#endif  // PROC_IMAGE_PROCESSING_FILTER_H_
