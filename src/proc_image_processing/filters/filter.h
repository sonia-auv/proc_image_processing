/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROVIDER_VISION_FILTER_H_
#define PROVIDER_VISION_FILTER_H_

#include <proc_image_processing/server/global_param_handler.h>
#include <proc_image_processing/server/parameter.h>
#include <proc_image_processing/server/ranged_parameter.h>
#include <proc_image_processing/server/target.h>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace proc_image_processing {

  class IFilter {
  public:
    using Ptr = std::shared_ptr<IFilter>;

    explicit IFilter(const GlobalParamHandler& globalParams);

    virtual ~IFilter() = default;

    virtual void Execute(cv::Mat& image) = 0;

    // Name of the filter handlers
    inline const std::string GetName();

    inline void SetName(const std::string& name);

    const std::vector<ParameterInterface*>& GetParameters() const;

    std::string GetParameterValue(const std::string& name);

    void SetParameterValue(const std::string& name, std::string value);

    // Wrapper for a call to _globalParms
    // NotifyTarget, to be put on the result topic
    void NotifyTarget(const Target& target);

    void GlobalParamInteger(const std::string& name, const int value,
      const int min, const int max);

    void GlobalParamDouble(const std::string& name, const double value,
      const double min, const double max);

    void GlobalParamBoolean(const std::string& name, const bool value);

    void GlobalParamString(const std::string& name, const std::string& value);

  protected:
    GlobalParamHandler& global_params_;

    std::vector<ParameterInterface*> parameters_;

    // Useful to identify the filter.
    std::string name_;
  };

}  // namespace proc_image_processing

#include <proc_image_processing/filters/filter_inl.h>

#endif  // PROVIDER_VISION_FILTER_H_
