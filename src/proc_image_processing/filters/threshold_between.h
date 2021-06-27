/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROVIDER_VISION_FILTERS_THRESHOLD_BETWEEN_H_
#define PROVIDER_VISION_FILTERS_THRESHOLD_BETWEEN_H_

#include <proc_image_processing/filters/filter.h>
#include <memory>

namespace proc_image_processing {

  class ThresholdBetween : public AbstractFilter {
  public:
    using Ptr = std::shared_ptr<Threshold>;

    explicit ThresholdBetween(const GlobalParamHandler& globalParams)
      : AbstractFilter(globalParams),
      type_("Threshold_type_", 1, 0, 5, &parameters_,
        "0=BIN, 1=BIN_INV, 2=TRUNC, 3=TOZERO, 4=TOZERO_INV 5=OTSU"),
      min_1("Min_value_1", 100, 0, 255, &parameters_),
      min_2("Min_value_2", 100, 0, 255, &parameters_) {
      SetName("ThresholdBetween");
    }

    virtual ~ThresholdBetween() {}

    virtual void ProcessImage(cv::Mat& image) {

        if (image.channels() > 1) {
          cv::cvtColor(image, image, CV_BGR2GRAY);
        }
        if (image.depth() != CV_8U) {
          image.convertTo(image, CV_8U);
        }

        int threshold_type = CV_THRESH_BINARY;
        switch (type_()) {
        case 0:
          threshold_type = CV_THRESH_BINARY;
          break;
        case 1:
          threshold_type = CV_THRESH_BINARY_INV;
          break;
        case 2:
          threshold_type = CV_THRESH_TRUNC;
          break;
        case 3:
          threshold_type = CV_THRESH_TOZERO;
          break;
        case 4:
          threshold_type = CV_THRESH_TOZERO_INV;
          break;
        case 5:
          threshold_type = CV_THRESH_OTSU;
          break;
        default:
          threshold_type = CV_THRESH_BINARY;
          break;
        }
        cv::threshold(image, image_1, min_1(), 255, threshold_type);
        cv::threshold(image, image_2, min_2(), 255, threshold_type);
        image = image_2 - image_1;
    }

  private:
    cv::Mat image_1, image_2, image_out;

    
    RangedParameter<int> type_, min_1, min_2;
  };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_THRESHOLD_BETWEEN_H_
