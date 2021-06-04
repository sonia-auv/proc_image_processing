/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROVIDER_VISION_FILTERS_MISSION_TEST_FAKE_STRING_H_
#define PROVIDER_VISION_FILTERS_MISSION_TEST_FAKE_STRING_H_

#include <proc_image_processing/filters/filter.h>
#include <memory>

namespace proc_image_processing {

  class MissionTestFakeString : public Filter {
  public:
    //==========================================================================
    // T Y P E D E F   A N D   E N U M

    using Ptr = std::shared_ptr<MissionTestFakeString>;

    //============================================================================
    // P U B L I C   C / D T O R S

    explicit MissionTestFakeString(const GlobalParamHandler& globalParams)
      : Filter(globalParams),
      enable_("Enable", false, &parameters_),
      _string("String_to_return", "test", &parameters_) {
      SetName("MissionTestFakeString");
    }

    virtual ~MissionTestFakeString() {}

    //============================================================================
    // P U B L I C   M E T H O D S

    virtual void Execute(cv::Mat& image) {
      if (enable_()) {
        NotifyTarget(Target());
      }
    }


  private:
    //============================================================================
    // P R I V A T E   M E M B E R S

    Parameter<bool> enable_;
    Parameter<std::string> _string;
  };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_MISSION_TEST_FAKE_STRING_H_
