#ifndef PROC_IMAGE_PROCESSING_FILTER_H_
#define PROC_IMAGE_PROCESSING_FILTER_H_

#include <proc_image_processing/cpu/server/global_param_handler.h>
#include <proc_image_processing/cpu/server/parameter.h>
#include <proc_image_processing/cpu/server/ranged_parameter.h>
#include <proc_image_processing/cpu/server/target.h>
#include <proc_image_processing/cpu/algorithm/performance_evaluator.h>
#include <ros/console.h>
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

        void execute(cv::Mat &image) {
            if(!enable_()) return;
            
            PerformanceEvaluator timer;

            apply(image);
            
            if(execTime_())
                ROS_INFO("Exec time of %s : %f s", name_.c_str(), timer.getExecutionTime());
        }

        // Name of the filter handlers
        inline std::string getName();

        inline void setName(const std::string &name);

        const std::vector<ParameterInterface *> &getParameters() const;

        std::string getParameterValue(const std::string &name);

        void setParameterValue(const std::string &name, const std::string &value);

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

    private:
        virtual void apply(cv::Mat &image) = 0;

        Parameter<bool> enable_{"Enable", false, &parameters_};
        Parameter<bool> execTime_{"ExecutionTime", false, &parameters_};
    };

}  // namespace proc_image_processing

#include "filter_inl.h"

#endif  // PROC_IMAGE_PROCESSING_FILTER_H_
