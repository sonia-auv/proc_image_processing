/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>

// FACTORY_GENERATOR_CLASS_NAME=WhiteNoiseTakedown

#ifndef PROVIDER_VISION_FILTERS_WHITE_NOISE_TAKEDOWN_H_
#define PROVIDER_VISION_FILTERS_WHITE_NOISE_TAKEDOWN_H_

#include <proc_image_processing/filters/filter.h>
#include <memory>

namespace proc_image_processing {

  class WhiteNoiseTakedown : public Filter {
  public:
    using Ptr = std::shared_ptr<WhiteNoiseTakedown>;

    explicit WhiteNoiseTakedown(const GlobalParamHandler& globalParams)
      : Filter(globalParams),
      low_b_("LowB", 0, 0, 255, &parameters_),
      high_b_("HighB", 0, 0, 255, &parameters_),
      low_g_("LowG", 0, 0, 255, &parameters_),
      high_g_("HighG", 0, 0, 255, &parameters_),
      low_r_("LowR", 0, 0, 255, &parameters_),
      high_r_("HighR", 0, 0, 255, &parameters_),
      view_channel_("Channel_view", 0, 0, 3, &parameters_,
        "0=ALL, 1=Blue, 2=Green, 3=Red") {
      SetName("WhiteNoiseTakedown");
    }

    virtual ~WhiteNoiseTakedown() {}

    virtual void ApplyFilter(cv::Mat& image) {
        std::vector<cv::Mat> channels;
        cv::Mat original_image(global_params_.getOriginalImage());
        cv::split(original_image, channels);
        cv::inRange(channels[0], low_b_(), high_b_(), channels[0]);
        cv::inRange(channels[1], low_g_(), high_g_(), channels[1]);
        cv::inRange(channels[2], low_r_(), high_r_(), channels[2]);
        cv::Mat result;
        cv::bitwise_or(channels[0], channels[1], result);
        cv::bitwise_or(channels[2], result, result);
        std::vector<cv::Mat> res;

        switch (view_channel_()) {
        case 0:
          if (image.channels() == 3) {
            res.push_back(result);
            res.push_back(result);
            res.push_back(result);
            cv::merge(res, result);
            cv::bitwise_and(image, result, image);
          }
          else {
            cv::bitwise_and(image, result, image);
          }
          break;
        case 1:
          channels[0].copyTo(image);
          break;
        case 2:
          channels[1].copyTo(image);
          break;
        case 3:
          channels[2].copyTo(image);
          break;
        }
    }

  private:
    RangedParameter<int> low_b_, high_b_, low_g_, high_g_, low_r_, high_r_;
    RangedParameter<int> view_channel_;
  };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_INRANGE_H_
