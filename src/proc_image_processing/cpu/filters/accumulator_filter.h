/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>

// FACTORY_GENERATOR_CLASS_NAME=AccumulatorFilter

#ifndef PROVIDER_VISION_FILTERS_IMAGE_ACCUMULATOR_H_
#define PROVIDER_VISION_FILTERS_IMAGE_ACCUMULATOR_H_

#include <proc_image_processing/cpu/algorithm/image_accumulator_buffer.h>
#include <proc_image_processing/cpu/algorithm/performance_evaluator.h>
#include "filter.h"
#include <memory>

namespace proc_image_processing {

    class AccumulatorFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<AccumulatorFilter>;

        explicit AccumulatorFilter(const GlobalParamHandler &globalParams)
                : Filter(globalParams),
                  accumulator_(3, cv::Size(0, 0), CV_8UC1),
                  enable_("Enable", false, &parameters_),
                  nb_image_("NB_of_images", 3, 1, 20, &parameters_),
                  method_("Method_to_use", 1, 0, 2, &parameters_,
                          "Method: 1=SameWeight, 2=Adding50Percent, 3=Adjusted"),
                  last_size_(0, 0),
                  last_method_(CV_8UC1),
                  last_type_(0),
                  last_nb_image_(3) {
            setName("AccumulatorFilter");
    }

        virtual ~AccumulatorFilter() {}

      virtual void apply(cv::Mat &image) {
          if (enable_()) {
              // Is there any change in the type of images
              // we input to the accumulator?
              // If yes, reset it.
              if (last_type_ != image.type() || last_method_ != method_() ||
                  last_nb_image_ != nb_image_() || last_size_ != image.size()) {
                  accumulator_.resetBuffer(nb_image_(), image.size(), image.type());

                  last_nb_image_ = nb_image_();
                  last_size_ = image.size();

          switch (method_()) {
              case 0:
                  accumulator_.setAverageMethod(
                          ImageAccumulatorBuffer::ACC_ALL_SAME_WEIGHT);
            break;
              case 1:
                  accumulator_.setAverageMethod(
                          ImageAccumulatorBuffer::ACC_50_PERCENT);
            break;
              case 2:
                  accumulator_.setAverageMethod(
                          ImageAccumulatorBuffer::ACC_ADJUST_WEIGHT);
            break;
              default:
                  accumulator_.setAverageMethod(
                          ImageAccumulatorBuffer::ACC_ALL_SAME_WEIGHT);
            break;
          }
          last_method_ = method_();
          last_type_ = image.type();
        }
        // Add the newest frame
          accumulator_.addImage(image);
        // Change the input for the newest averaging.
          accumulator_.convertImage(image);
      }
    }

  private:
    ImageAccumulatorBuffer accumulator_;
    Parameter<bool> enable_;
    RangedParameter<int> nb_image_, method_;
    // Here we need some sorte of remembering
    // so we can reset the accumulator on
    // param changing.
    cv::Size last_size_;
    int last_method_, last_type_, last_nb_image_;
  };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_IMAGE_ACCUMULATOR_H_
