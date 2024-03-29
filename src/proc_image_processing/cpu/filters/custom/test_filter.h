// FACTORY_GENERATOR_CLASS_NAME=TestFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_TEST_FILTER_H_
#define PROC_IMAGE_PROCESSING_FILTERS_TEST_FILTER_H_

#include "proc_image_processing/cpu/filters/filter.h"
#include <proc_image_processing/cpu/server/target.h>
#include <memory>

namespace proc_image_processing {

    class TestFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<TestFilter>;

        explicit TestFilter(const GlobalParameterHandler &globalParams)
                : Filter(globalParams),
                  x_("X", 0, -512, 512, &parameters_),
                  y_("Y", 0, -512, 512, &parameters_),
                  w_("Width", 200, 0, 1024, &parameters_),
                  h_("Height", 200, 0, 1024, &parameters_),
                  angle_("Angle", 0, 0, 360, &parameters_),
                  header_("Header", "test_filter", &parameters_),
                  specField1_("Special field 1", "sf1", &parameters_),
                  specField2_("Special field 2", "sf2", &parameters_) {
            setName("TestFilter");
        }

        ~TestFilter() override = default;

        void apply(cv::Mat &image) override {
            target_.setTarget(
                    header_.getValue(),
                    x_.getValue() - 1000 / 2,
                    y_.getValue(),
                    w_.getValue(),
                    h_.getValue(),
                    angle_.getValue(),
                    1000,
                    -1000 - (1000 / 2),
                    specField1_.getValue(),
                    specField2_.getValue()
            );

            notify(target_);
        }

    private:
        Parameter <std::string> header_;
        Parameter <std::string> specField1_;
        Parameter <std::string> specField2_;

        RangedParameter<int> x_;
        RangedParameter<int> y_;
        RangedParameter<int> w_;
        RangedParameter<int> h_;
        RangedParameter<int> angle_;

        Target target_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_TEST_FILTER_H_
