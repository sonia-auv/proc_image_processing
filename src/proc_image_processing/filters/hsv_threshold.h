/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>

// FACTORY_GENERATOR_CLASS_NAME=HSVThreshold

#ifndef PROVIDER_VISION_FILTERS_HSV_THRESHOLD_H_
#define PROVIDER_VISION_FILTERS_HSV_THRESHOLD_H_

#include <proc_image_processing/algorithm/general_function.h>
#include <proc_image_processing/filters/filter.h>
#include <memory>

namespace proc_image_processing {

  // Filter showing planes of different analysis (gray, _hsi, _bgr)
  // No threshold
  class HSVThreshold : public Filter {
  public:
    using Ptr = std::shared_ptr<HSVThreshold>;

    explicit HSVThreshold(const GlobalParamHandler& globalParams)
      : Filter(globalParams),
      hue_min_("Hue Min", 0, 0, 256, &parameters_,
        "Minimum Hue to threshold. Keep values higher or equal to this value."),
      hue_max_("Hue Max", 255, 0, 256, &parameters_,
        "Maximum Hue to threshold. Keep values lower or equal to this value."),
      saturation_min_("saturation Min", 0, 0, 256, &parameters_,
        "Minimum saturation to threshold. Keep values higher or equal to this value."),
      saturation_max_("saturation Max", 255, 0, 256, &parameters_,
        "Maximum saturation to threshold. Keep values lower or equal to this value."),
      value_min_("value Min", 0, 0, 256, &parameters_,
        "Minimum value to threshold. Keep values higher or equal to this value."),
      value_max_("value Max", 255, 0, 256, &parameters_,
        "Maximum value to threshold. Keep values lower or equal to this value."),
      rows_(0),
      cols_(0) {
      SetName("HSVThreshold");
    }

    virtual ~HSVThreshold() {}

    void apply(cv::Mat& image) override {
        if (CV_MAT_CN(image.type()) != 3) {
          return;
        }

        rows_ = image.rows;
        cols_ = image.cols;
        // Set final matrices
        cv::Mat hue = cv::Mat::zeros(rows_, cols_, CV_8UC1);
        cv::Mat hue_res1 = cv::Mat::zeros(rows_, cols_, CV_8UC1);
        cv::Mat hue_res2 = cv::Mat::zeros(rows_, cols_, CV_8UC1);
        cv::Mat hue_res = cv::Mat::zeros(rows_, cols_, CV_8UC1);
        cv::Mat saturation = cv::Mat::zeros(rows_, cols_, CV_8UC1);
        cv::Mat saturation_res = cv::Mat::zeros(rows_, cols_, CV_8UC1);
        cv::Mat value = cv::Mat::zeros(rows_, cols_, CV_8UC1);
        cv::Mat value_res = cv::Mat::zeros(rows_, cols_, CV_8UC1);
        cv::Mat final = cv::Mat::zeros(rows_, cols_, CV_8UC1);

        // Replace with new images
        channel_vec_ = GetColorPlanes(image);
        set_image(H_INDEX, hue);
        set_image(S_INDEX, saturation);
        set_image(V_INDEX, value);
        cv::inRange(hue, cv::Scalar(hue_min_()), cv::Scalar(hue_max_()), hue_res);
        //cv::threshold(hue,hue_res1,hue_min_(),255,cv::THRESH_BINARY);
        //cv::threshold(hue,hue_res2,hue_max_(),255,cv::THRESH_BINARY_INV);
        //cv::bitwise_and(hue_res1, hue_res2, hue_res);
        cv::inRange(saturation, cv::Scalar(saturation_min_()), cv::Scalar(saturation_max_()), saturation_res);
        cv::inRange(value, cv::Scalar(value_min_()), cv::Scalar(value_max_()), value_res);

        cv::bitwise_and(hue_res, saturation_res, final);
        cv::bitwise_and(final, value_res, final);

        final.copyTo(image);
    }

  private:
    void set_image(const int choice, cv::Mat& out) {
      // Thightly couple with parameter, but putting safety...
      int index = choice < 0 ? 0 : (choice > 6 ? 6 : choice);
      channel_vec_[index].copyTo(out);

    }

    RangedParameter<int> hue_min_, hue_max_, saturation_min_, saturation_max_, value_min_, value_max_;
    // Color matrices
    std::vector<cv::Mat> channel_vec_;

    int rows_;
    int cols_;
    const int H_INDEX = 4;
    const int S_INDEX = 5;
    const int V_INDEX = 6;
  };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_HSV_THRESHOLD_H_
