// FACTORY_GENERATOR_CLASS_NAME=SubtractAllPlanesFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_SUBTRACT_ALL_PLANES_H_
#define PROC_IMAGE_PROCESSING_FILTERS_SUBTRACT_ALL_PLANES_H_

#include <proc_image_processing/cpu/algorithm/general_function.h>
#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {

    // Filter showing planes of different analysis (gray, _hsi, _bgr)
    // No threshold
    class SubtractAllPlanesFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<SubtractAllPlanesFilter>;

        explicit SubtractAllPlanesFilter(const GlobalParameterHandler &globalParams)
                : Filter(globalParams),
                  plane_one_("Plane 1", 1, 0, 7, &parameters_,
                             "0=None, 1=Blue, 2=Green, 3=Red, 4=Hue, 5=Saturation, "
                             "6=Intensity, 7=Gray"),
                  plane_two_("Plane 2", 1, 0, 7, &parameters_,
                             "0=None, 1=Blue, 2=Green, 3=Red, 4=Hue, 5=Saturation, "
                             "6=Intensity, 7=Gray"),
                  plane_three_("Plane 3", 1, 0, 7, &parameters_,
                               "0=None, 1=Blue, 2=Green, 3=Red, 4=Hue, 5=Saturation, "
                               "6=Intensity, 7=Gray"),
                  invert_one_("Invert plane 1", false, &parameters_),
                  invert_two_("Invert plane 2", false, &parameters_),
                  invert_three_("Invert plane 3", false, &parameters_),
                  weight_one_("Weight plane 1", 1.0, 0.0, 10.0, &parameters_),
                  weight_two_("Weight plane 2", 1.0, 0.0, 10.0, &parameters_),
                  weight_three_("Weight plane 3", 1.0, 0.0, 10.0, &parameters_),
                  rows_(0),
                  cols_(0) {
            setName("SubtractAllPlanesFilter");
        }

        ~SubtractAllPlanesFilter() override = default;

        void apply(cv::Mat &image) override {
            if (CV_MAT_CN(image.type()) != 3) {
                return;
            }

            rows_ = image.rows;
            cols_ = image.cols;
            // Set result matrices
            cv::Mat zero = cv::Mat::zeros(rows_, cols_, CV_8UC1);
            cv::Mat one = cv::Mat::zeros(rows_, cols_, CV_8UC1);
            cv::Mat two = cv::Mat::zeros(rows_, cols_, CV_8UC1);
            cv::Mat three = cv::Mat::zeros(rows_, cols_, CV_8UC1);
            cv::Mat result = cv::Mat::zeros(rows_, cols_, CV_8UC1);

            // Replace with new images
            channel_vec_ = getColorPlanes(image);

            // Set subtraction
            if (plane_one_() != 0)
                setImage(plane_one_() - 1, one, weight_one_(), invert_one_());

            if (plane_two_() != 0)
                setImage(plane_two_() - 1, two, weight_two_(), invert_two_());

            if (plane_three_() != 0)
                setImage(plane_three_() - 1, three, weight_three_(), invert_three_());

            cv::subtract(one, two, result);
            cv::subtract(result, three, result);

            result.copyTo(image);
        }

    private:
        void setImage(const int choice, cv::Mat &out, const double weight, const bool inverse) {
            cv::Mat two_five_five(rows_, cols_, CV_16SC1, cv::Scalar(255));
            cv::Mat one(rows_, cols_, CV_16SC1, cv::Scalar(1));

            // Thightly couple with parameter, but putting safety...
            int index = choice < 0 ? 0 : (choice > 6 ? 6 : choice);
            channel_vec_[index].copyTo(out);

            if (inverse) {
                inverseImage(out, out);
            }
            cv::multiply(out, one, out, weight, CV_8UC1);
        }

        Parameter<bool> invert_one_;
        Parameter<bool> invert_two_;
        Parameter<bool> invert_three_;

        RangedParameter<int> plane_one_;
        RangedParameter<int> plane_two_;
        RangedParameter<int> plane_three_;
        RangedParameter<double> weight_one_;
        RangedParameter<double> weight_two_;
        RangedParameter<double> weight_three_;

        // Color matrices
        std::vector<cv::Mat> channel_vec_;

        int rows_;
        int cols_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_SUBTRACT_ALL_PLANES_H_
