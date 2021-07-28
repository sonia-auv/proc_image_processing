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

        inline std::string getName();

        inline const std::vector<ParameterInterface *> &getParameters() const;

        inline std::string getParameterValue(const std::string &name);

        inline void addGlobalParameter(const std::string &name, int value, int min, int max);

        inline void addGlobalParameter(const std::string &name, double value, double min, double max);

        inline void addGlobalParameter(const std::string &name, bool value);

        inline void addGlobalParameter(const std::string &name, const std::string &value);

        virtual void apply(cv::Mat &image) = 0;

        inline void notify(const Target &target);

        inline void setName(const std::string &name);

        inline void setParameterValue(const std::string &name, const std::string &value);

    protected:
        GlobalParamHandler &global_param_handler_;
        std::vector<ParameterInterface *> parameters_;
        std::string name_;
    };

}  // namespace proc_image_processing

#include "filter_inl.h"

#endif  // PROC_IMAGE_PROCESSING_FILTER_H_
