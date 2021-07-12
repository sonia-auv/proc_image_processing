/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>

// FACTORY_GENERATOR_CLASS_NAME=ContrastBrightness

#ifndef PROVIDER_VISION_FILTERS_CONTRAST_BRIGHTNESS_H_
#define PROVIDER_VISION_FILTERS_CONTRAST_BRIGHTNESS_H_

#include <proc_image_processing/algorithm/general_function.h>
#include <proc_image_processing/filters/filter.h>
#include <memory>

namespace proc_image_processing {

  // Filter showing planes of different analysis (gray, _hsi, _bgr)
  // No threshold
  class ContrastBrightness : public Filter {
  public:
    using Ptr = std::shared_ptr<ContrastBrightness>;

    explicit ContrastBrightness(const GlobalParamHandler& globalParams)
      : Filter(globalParams),
      contrast_("Contrast", 0, 0, 256, &parameters_,
        "Contrast"),
      brightness_("Brightness", 0, -256, 256, &parameters_,
        "Set Brightness"),
      rows_(0),
      cols_(0) {
      SetName("ContrastBrightness");
    }

    virtual ~ContrastBrightness() {}

    void apply(cv::Mat& image) override {
        rows_ = image.rows;
        cols_ = image.cols;
        // Set final matrices

        cv::Mat final = cv::Mat::zeros(rows_, cols_, image.type());

        // Replace with new images

        for (int y = 0; y < image.rows; y++) {
          for (int x = 0; x < image.cols; x++) {
            for (int c = 0;c < image.channels(); c++)
              final.at<cv::Vec3b>(y, x)[c] = cv::saturate_cast<uchar>(contrast_() * (image.at<cv::Vec3b>(y, x)[c]) + brightness_());

          }
        }

        final.copyTo(image);
    }


  private:
    RangedParameter<double> contrast_, brightness_;
    // Color matrices
    int rows_;
    int cols_;
  };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_CONTRAST_BRIGHTNESS_H_
