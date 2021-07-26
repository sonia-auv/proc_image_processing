// FACTORY_GENERATOR_CLASS_NAME=SubtractPlaneAdderFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_SUBTRACT_PLANES_ADDER_H_
#define PROC_IMAGE_PROCESSING_FILTERS_SUBTRACT_PLANES_ADDER_H_

#include <proc_image_processing/cpu/algorithm/general_function.h>
#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {

    // Take the input image as binary, takes the originalImage and process
    // a subtracAllPlane filter, then add both input and computed image together.
    // Most usefull for the purple handle...
    class SubtractPlaneAdderFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<SubtractPlaneAdderFilter>;

        explicit SubtractPlaneAdderFilter(const GlobalParamHandler &globalParams)
                : Filter(globalParams),
                  enable_("enable", false, &parameters_),
                  show_adding_result_("show_adding_result", false, &parameters_),
                  plane_one_("Plane_1", 1, 0, 7, &parameters_,
                             "0=None, 1=Blue, 2=Green, 3=Red, 4=Hue, 5=Saturation, "
                             "6=Intensity, 7=Gray"),
                  plane_two_("Plane_2", 1, 0, 7, &parameters_,
                             "0=None, 1=Blue, 2=Green, 3=Red, 4=Hue, 5=Saturation, "
                             "6=Intensity, 7=Gray"),
                  plane_three_("Plane_3", 1, 0, 7, &parameters_,
                               "0=None, 1=Blue, 2=Green, 3=Red, 4=Hue, 5=Saturation, "
                               "6=Intensity, 7=Gray"),
                  invert_one_("Invert_plane_1", false, &parameters_),
                  invert_two_("Invert_plane_2", false, &parameters_),
                  invert_three_("Invert_plane_3", false, &parameters_),
                  weight_one_("Weight_Plane_1", 1.0, -10.0, 10.0, &parameters_),
                  weight_two_("Weight_Plane_2", 1.0, -10.0, 10.0, &parameters_),
                  weight_three_("Weight_Plane_3", 1.0, -10.0, 10.0, &parameters_),
                  rows_(0),
                  cols_(0) {
            setName("SubtractPlaneAdderFilter");
        }

        ~SubtractPlaneAdderFilter() override = default;

        void apply(cv::Mat &image) override {
            if (enable_()) {
                cv::Mat original = global_params_.getOriginalImage();
                if (CV_MAT_CN(original.type()) != 3) {
                    return;
                }

                rows_ = image.rows;
                cols_ = image.cols;
                // Set final matrices
                cv::Mat zero = cv::Mat::zeros(rows_, cols_, CV_8UC1);
                cv::Mat one = cv::Mat::zeros(rows_, cols_, CV_8UC1);
                cv::Mat two = cv::Mat::zeros(rows_, cols_, CV_8UC1);
                cv::Mat three = cv::Mat::zeros(rows_, cols_, CV_8UC1);
                cv::Mat final = cv::Mat::zeros(rows_, cols_, CV_8UC1);

                // Replace with new images
                channel_vec_ = getColorPlanes(original);

                // Set subtraction
                if (plane_one_() != 0)
                    setImage(plane_one_() - 1, one, weight_one_(), invert_one_());

                if (plane_two_() != 0)
                    setImage(plane_two_() - 1, two, weight_two_(), invert_two_());

                if (plane_three_() != 0)
                    setImage(plane_three_() - 1, three, weight_three_(), invert_three_());

                cv::subtract(one, two, final);
                cv::subtract(final, three, final);

                if (!show_adding_result_()) {
                    cv::add(final, image, final);
                }
                final.copyTo(image);
            }
        }

    private:
        void setImage(const int choice, cv::Mat &out, const double weight,
                      const bool inverse) {
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

        Parameter<bool> enable_, show_adding_result_;
        RangedParameter<int> plane_one_, plane_two_, plane_three_;
        Parameter<bool> invert_one_, invert_two_, invert_three_;
        RangedParameter<double> weight_one_, weight_two_, weight_three_;

        // Color matrices
        std::vector<cv::Mat> channel_vec_;

        int rows_;
        int cols_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_SUBTRACT_PLANES_ADDER_H_