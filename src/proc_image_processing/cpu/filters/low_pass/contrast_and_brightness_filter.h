// FACTORY_GENERATOR_CLASS_NAME=ContrastAndBrightnessFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_CONTRAST_BRIGHTNESS_H_
#define PROC_IMAGE_PROCESSING_FILTERS_CONTRAST_BRIGHTNESS_H_

#include <proc_image_processing/cpu/algorithm/general_function.h>
#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {
    /*
    class LambdaBody : public cv::ParallelLoopBody {    
    public:
        typedef void(*LambdaParallelLoopBody)(const cv::Range & range);
        LambdaBody(LambdaParallelLoopBody body){
            _body = body;
        }

        void operator() (const cv::Range & range) const
        {
            _body(range);
        }    
    private:
        LambdaParallelLoopBody _body;
    };
    */
    // Filter showing planes of different analysis (gray, _hsi, _bgr)
    // No threshold
    class ContrastAndBrightnessFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<ContrastAndBrightnessFilter>;

        explicit ContrastAndBrightnessFilter(const GlobalParamHandler &globalParams)
                : Filter(globalParams),
                  enable_("enable", false, &parameters_),
                  contrast_("Contrast", 0, 0, 256, &parameters_,
                            "Contrast"),
                  brightness_("Brightness", 0, -256, 256, &parameters_,
                              "Set Brightness"),
                  rows_(0),
                  cols_(0) {
            setName("ContrastAndBrightnessFilter");
        }

        ~ContrastAndBrightnessFilter() override = default;

        void apply(cv::Mat &image) override {
            if (enable_()) {
                for (int y = 0; y < image.rows; y++) {
                    for (int x = 0; x < image.cols; x++) {
                        for (int c = 0; c < image.channels(); c++)
                            image.at<cv::Vec3b>(y, x)[c] = cv::saturate_cast<uchar>(
                                    contrast_() * (image.at<cv::Vec3b>(y, x)[c]) + brightness_());
                    }
                }
               /*
               // Apply filter to pixels in parallel
               cv::parallel_for_(cv::Range(0, 10), LambdaBody([&](const cv::Range & range){
                   for(int r = range.start; r < range.end; r++) {
                        int y = r / image.cols;
                        int x = r % image.cols;
                        
                        for (int c = 0; c < image.channels(); c++)
                            image.at<cv::Vec3b>(y, x)[c] = cv::saturate_cast<uchar>(
                                    contrast_() * (image.at<cv::Vec3b>(y, x)[c]) + brightness_());
                   }
                }));
                */
            }
        }


    private:
        Parameter<bool> enable_;
        RangedParameter<double> contrast_, brightness_;
        // Color matrices
        int rows_;
        int cols_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_CONTRAST_BRIGHTNESS_H_
