// FACTORY_GENERATOR_CLASS_NAME=ConvexHullFilter

#ifndef PROC_IMAGE_PROCESSING_FILTERS_CONVEX_HULL_H_
#define PROC_IMAGE_PROCESSING_FILTERS_CONVEX_HULL_H_

#include "proc_image_processing/cpu/filters/filter.h"
#include <memory>

namespace proc_image_processing {

    class ConvexHullFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<ConvexHullFilter>;

        explicit ConvexHullFilter(const GlobalParamHandler &globalParams)
                : Filter(globalParams),
                  enable_("Enable", false, &parameters_),
                  mode_("Mode", 0, 0, 3, &parameters_,
                        "0=CV_RETR_EXTERNAL,1=CV_RETR_LIST, 2=CV_RETR_CCOMP, "
                        "3=CV_RETR_TREE"),
                  method_("Method", 0, 0, 3, &parameters_,
                          "0=CV_CHAIN_APPROX_NONE, 1=CV_CHAIN_APPROX_SIMPLE, "
                          "2=CV_CHAIN_APPROX_TC89_L1, "
                          "3=CV_CHAIN_APPROX_TC89_KCOS") {
            setName("ConvexHullFilter");
        }

        ~ConvexHullFilter() override = default;

        void apply(cv::Mat &image) override {
            if (enable_()) {
                int mode, method;
                switch (mode_()) {
                    case 1:
                        mode = char(CV_RETR_LIST);
                        break;
                    case 2:
                        mode = char(CV_RETR_CCOMP);
                        break;
                    case 3:
                        mode = char(CV_RETR_TREE);
                        break;
                    default:
                        mode = char(CV_RETR_EXTERNAL);
                        break;
                }
                switch (method_()) {
                    case 1:
                        method = char(CV_CHAIN_APPROX_SIMPLE);
                        break;
                    case 2:
                        method = char(CV_CHAIN_APPROX_TC89_L1);
                        break;
                    case 3:
                        method = char(CV_CHAIN_APPROX_TC89_KCOS);
                        break;
                    default:
                        method = char(CV_CHAIN_APPROX_NONE);
                        break;
                }
                std::vector<std::vector<cv::Point>> contours;
                std::vector<cv::Vec4i> hierarchy;

                // Find contours
                cv::findContours(image, contours, hierarchy, mode, method,
                                 cv::Point(0, 0));

                // Find the convex hull object for each contour
                std::vector<std::vector<cv::Point>> hull(contours.size());
                for (size_t i = 0; i < contours.size(); i++) {
                    cv::convexHull(cv::Mat(contours[i]), hull[i], false);
                }

                // draw Hull contour
                image = cv::Mat::zeros(image.size(), CV_8UC1);
                for (size_t i = 0; i < contours.size(); i++) {
                    cv::drawContours(image, hull, i, cv::Scalar(255, 255, 255), CV_FILLED);
                }
            }
        }

    private:
        Parameter<bool> enable_;
        RangedParameter<int> mode_, method_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_CONVEX_HULL_H_
