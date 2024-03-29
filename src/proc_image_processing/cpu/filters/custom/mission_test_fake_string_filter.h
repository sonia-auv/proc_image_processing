// FACTORY_GENERATOR_CLASS_NAME=MissionTestFakeStringFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_MISSION_TEST_FAKE_STRING_H_
#define PROC_IMAGE_PROCESSING_FILTERS_MISSION_TEST_FAKE_STRING_H_

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {

    class MissionTestFakeStringFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<MissionTestFakeStringFilter>;

        explicit MissionTestFakeStringFilter(const GlobalParameterHandler &globalParams)
                : Filter(globalParams), _string("String to return", "test", &parameters_) {
            setName("MissionTestFakeStringFilter");
        }

        ~MissionTestFakeStringFilter() override = default;

        void apply(cv::Mat &image) override {
            notify(Target());
        }

    private:
        // TODO What is this param used for?!?
        Parameter <std::string> _string;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_MISSION_TEST_FAKE_STRING_H_
