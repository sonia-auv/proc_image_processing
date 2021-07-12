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

  class Filter {
  public:
    using Ptr = std::shared_ptr<Filter>;

    explicit Filter(const GlobalParamHandler& globalParams);

    virtual ~Filter() = default;

    void execute(cv::Mat& image) {
      if (enable_()){
        apply(image);
      }
    };

    virtual void apply(cv::Mat& image) = 0;

    // Name of the filter handlers
    inline const std::string getName();

    inline void setName(const std::string& name);

    const std::vector<ParameterInterface*>& getParameters() const;

    std::string getParameterValue(const std::string& name);

    void setParameterValue(const std::string& name, std::string value);

    // Wrapper for a call to _globalParms
    // NotifyTarget, to be put on the result topic
    void notifyTarget(const Target& target);

    void globalParamInteger(const std::string& name, const int value,
      const int min, const int max);

    void globalParamDouble(const std::string& name, const double value,
      const double min, const double max);

    void globalParamBoolean(const std::string& name, const bool value);

    void globalParamString(const std::string& name, const std::string& value);

  protected:
    GlobalParamHandler& global_params_;

    std::vector<ParameterInterface*> parameters_;

    // Useful to identify the filter.
    std::string name_;

  private: 
    Parameter<bool> enable_{"Enable", false, &parameters_};
  };
}  // namespace proc_image_processing

#include <proc_image_processing/filters/filter_inl.h>

#endif  // PROVIDER_VISION_FILTER_H_
