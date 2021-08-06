// FACTORY_GENERATOR_CLASS_NAME=WhiteNoiseRemovalFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_WHITE_NOISE_TAKE_DOWN_H_
#define PROC_IMAGE_PROCESSING_FILTERS_WHITE_NOISE_TAKE_DOWN_H_

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {

    class WhiteNoiseRemovalFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<WhiteNoiseRemovalFilter>;

        explicit WhiteNoiseRemovalFilter(const GlobalParameterHandler &globalParams)
                : Filter(globalParams),
                  enable_("Enable", false, &parameters_),
                  lower_b_("Lower B", 0, 0, 255, &parameters_),
                  upper_b_("Upper B", 0, 0, 255, &parameters_),
                  lower_g_("Lower G", 0, 0, 255, &parameters_),
                  upper_g_("Upper G", 0, 0, 255, &parameters_),
                  lower_r_("Lower R", 0, 0, 255, &parameters_),
                  upper_r_("Upper R", 0, 0, 255, &parameters_),
                  view_channel_("View channel", 0, 0, 3, &parameters_, "0=ALL, 1=Blue, 2=Green, 3=Red") {
            setName("WhiteNoiseRemovalFilter");
        }

        ~WhiteNoiseRemovalFilter() override = default;

        void apply(cv::Mat &image) override {
            if (enable_()) {
                std::vector<cv::Mat> channels;
                cv::Mat original_image(global_param_handler_.getOriginalImage());
                cv::split(original_image, channels);
                cv::inRange(channels[0], lower_b_(), upper_b_(), channels[0]);
                cv::inRange(channels[1], lower_g_(), upper_g_(), channels[1]);
                cv::inRange(channels[2], lower_r_(), upper_r_(), channels[2]);
                cv::Mat result;
                cv::bitwise_or(channels[0], channels[1], result);
                cv::bitwise_or(channels[2], result, result);
                std::vector<cv::Mat> res;

                switch (view_channel_()) {
                    case 1:
                        channels[0].copyTo(image);
                        break;
                    case 2:
                        channels[1].copyTo(image);
                        break;
                    case 3:
                        channels[2].copyTo(image);
                        break;
                    default:
                        if (image.channels() == 3) {
                            res.push_back(result);
                            res.push_back(result);
                            res.push_back(result);
                            cv::merge(res, result);
                            cv::bitwise_and(image, result, image);
                        } else {
                            cv::bitwise_and(image, result, image);
                        }
                        break;
                }
            }
        }

    private:
        Parameter<bool> enable_;
        RangedParameter<int> lower_b_, upper_b_, lower_g_, upper_g_, lower_r_, upper_r_, view_channel_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_WHITE_NOISE_TAKE_DOWN_H_
