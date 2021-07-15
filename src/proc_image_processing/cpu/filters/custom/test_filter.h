/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>

// FACTORY_GENERATOR_CLASS_NAME=TestFilter

#ifndef PROVIDER_VISION_FILTERS_TEST_FILTER_H_
#define PROVIDER_VISION_FILTERS_TEST_FILTER_H_

#include "proc_image_processing/cpu/filters/filter.h"
#include <proc_image_processing/cpu/server/target.h>
#include <memory>

namespace proc_image_processing {

    class TestFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<TestFilter>;

        explicit TestFilter(const GlobalParamHandler &globalParams)
                : Filter(globalParams),
                  enable_("Enable", true, &parameters_),
                  x_("X", 0, -512, 512, &parameters_),
                  y_("Y", 0, -512, 512, &parameters_),
                  w_("Width", 200, 0, 1024, &parameters_),
                  h_("Height", 200, 0, 1024, &parameters_),
                  angle_("Angle", 0, 0, 360, &parameters_),
                  header_("Header", "test", &parameters_),
                  specField1_("SpecialField_1", "sf1", &parameters_),
                  specField2_("SpecialField_2", "sf2", &parameters_) {
            setName("TestFilter");
        }

        virtual ~TestFilter() {}

        virtual void init() {}

        virtual void apply(cv::Mat &image) {
            if (enable_()) {
                target_.setTarget("test_filter", x_.getValue() - 1000 / 2, y_.getValue(),
                                  w_.getValue(), h_.getValue(), angle_.getValue(), 1000,
                                  -1000 - (1000 / 2), specField1_.getValue(),
                                  specField2_.getValue());

                notify(target_);
            }
        }

    private:
        Parameter<bool> enable_;
        RangedParameter<int> x_, y_, w_, h_, angle_;
        Parameter <std::string> header_, specField1_, specField2_;

        Target target_;
    };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_TEST_FILTER_H_
