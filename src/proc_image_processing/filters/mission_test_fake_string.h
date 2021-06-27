/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROVIDER_VISION_FILTERS_MISSION_TEST_FAKE_STRING_H_
#define PROVIDER_VISION_FILTERS_MISSION_TEST_FAKE_STRING_H_

#include <proc_image_processing/filters/filter.h>
#include <memory>

namespace proc_image_processing {

  class MissionTestFakeString : public AbstractFilter {
  public:
    using Ptr = std::shared_ptr<MissionTestFakeString>;

    explicit MissionTestFakeString(const GlobalParamHandler& globalParams)
      : AbstractFilter(globalParams),
      enable_("Enable", false, &parameters_),
      _string("String_to_return", "test", &parameters_) {
      SetName("MissionTestFakeString");
    }

    virtual ~MissionTestFakeString() {}

    virtual void ProcessImage(cv::Mat& image) {

        NotifyTarget(Target());
    }

  private:
    
    Parameter<std::string> _string;
  };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_MISSION_TEST_FAKE_STRING_H_
