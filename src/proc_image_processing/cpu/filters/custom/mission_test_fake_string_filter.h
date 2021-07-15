/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>

// FACTORY_GENERATOR_CLASS_NAME=MissionTestFakeStringFilter

#ifndef PROVIDER_VISION_FILTERS_MISSION_TEST_FAKE_STRING_H_
#define PROVIDER_VISION_FILTERS_MISSION_TEST_FAKE_STRING_H_

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {

    class MissionTestFakeStringFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<MissionTestFakeStringFilter>;

        explicit MissionTestFakeStringFilter(const GlobalParamHandler &globalParams)
                : Filter(globalParams),
                  enable_("Enable", false, &parameters_),
                  _string("String_to_return", "test", &parameters_) {
            setName("MissionTestFakeStringFilter");
        }

        virtual ~MissionTestFakeStringFilter() {}

        virtual void apply(cv::Mat &image) {
            if (enable_()) {
                notify(Target());
            }
        }

    private:
        Parameter<bool> enable_;
        Parameter <std::string> _string;
    };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_MISSION_TEST_FAKE_STRING_H_
