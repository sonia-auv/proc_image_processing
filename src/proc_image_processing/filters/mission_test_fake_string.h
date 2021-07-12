/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>

// FACTORY_GENERATOR_CLASS_NAME=MissionTestFakeString

#ifndef PROVIDER_VISION_FILTERS_MISSION_TEST_FAKE_STRING_H_
#define PROVIDER_VISION_FILTERS_MISSION_TEST_FAKE_STRING_H_

#include <proc_image_processing/filters/filter.h>
#include <memory>

namespace proc_image_processing {

  class MissionTestFakeString : public Filter {
  public:
    using Ptr = std::shared_ptr<MissionTestFakeString>;

    explicit MissionTestFakeString(const GlobalParamHandler& globalParams)
      : Filter(globalParams),
      _string("String_to_return", "test", &parameters_) {
      setName("MissionTestFakeString");
    }

    virtual ~MissionTestFakeString() {}

    void apply(cv::Mat& image) override {
        notifyTarget(Target());
    }

  private:
    Parameter<std::string> _string;
  };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_MISSION_TEST_FAKE_STRING_H_
