// FACTORY_GENERATOR_CLASS_NAME=SquareDetector

#ifndef PROC_IMAGE_PROCESSING_SQUARE_DETECTOR_H
#define PROC_IMAGE_PROCESSING_SQUARE_DETECTOR_H


#include "proc_image_processing/cpu/filters/filter.h"
#include <cmath>
#include <memory>

namespace proc_image_processing {

    class SquareDetector : public Filter {
    public:
        using Ptr = std::shared_ptr<SquareDetector>;

        explicit SquareDetector(const GlobalParameterHandler &globalParams)
                : Filter(globalParams),
                  N("N", 100, 0, 100, &parameters_),
                  thresh("Threshold", 100, 0, 100, &parameters_) {
            setName("SquareDetector");
        }

        ~SquareDetector() override = default;

        void apply(cv::Mat &image) override {
            std::vector<std::vector<cv::Point> > squares;

            squares.clear();

            // blur will enhance edge detection
            cv::Mat timg(image);
            cv::medianBlur(image, timg, 9);
            cv::Mat gray0(timg.size(), CV_8U), gray;

            std::vector<std::vector<cv::Point> > contours;

            // find squares in every color plane of the image
            for (int c = 0; c < 3; c++) {
                int ch[] = {c, 0};
                cv::mixChannels(&timg, 1, &gray0, 1, ch, 1);

                // try several threshold levels
                for (int l = 0; l < N(); l++) {
                    // hack: use CannyFilter instead of zero threshold level.
                    // CannyFilter helps to catch squares with gradient shading
                    if (l == 0) {
                        // apply CannyFilter. Take the upper threshold from slider
                        // and set the lower to 0 (which forces edges merging)
                        cv::Canny(gray0, gray, 5, thresh(), 5);
                        // dilate canny output to remove potential
                        // holes between edge segments
                        cv::dilate(gray, gray, cv::Mat(), cv::Point(-1, -1));
                    } else {
                        // apply threshold if l!=0:
                        //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
                        gray = gray0 >= (l + 1) * 255 / N();
                    }

                    // find contours and store them all as a list
                    cv::findContours(gray, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

                    std::vector<cv::Point> approx;

                    // test each contour
                    for (auto &contour : contours) {
                        // approximate contour with accuracy proportional
                        // to the contour perimeter
                        cv::approxPolyDP(
                                cv::Mat(contour),
                                approx,
                                cv::arcLength(cv::Mat(contour), true) * 0.02,
                                true);

                        // square contours should have 4 vertices after approximation
                        // relatively large area (to filter out noisy contours)
                        // and be convex.
                        // Note: absolute value of an area is used because
                        // area may be positive or negative - in accordance with the
                        // contour orientation
                        if (approx.size() == 4 &&
                            fabs(cv::contourArea(cv::Mat(approx))) > 1000 &&
                            cv::isContourConvex(cv::Mat(approx))) {
                            double maxCosine = 0;

                            for (int j = 2; j < 5; j++) {
                                // find the maximum cosine of the getAngle between joint edges
                                double cosine = fabs(getAngle(approx[j % 4], approx[j - 2], approx[j - 1]));
                                maxCosine = MAX(maxCosine, cosine);
                            }

                            // if cosines of all angles are small
                            // (all angles are ~90 degree) then write quandrange
                            // vertices to resultant sequence
                            if (maxCosine < 0.3)
                                squares.push_back(approx);
                        }
                    }
                }
            }
            for (auto &square : squares) {
                const cv::Point *p = &square[0];

                int n = (int) square.size();
                //dont detect the border
                if (p->x > 3 && p->y > 3) {
                    cv::polylines(image, &p, &n, 1, true, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
                }
            }
        }

        // TODO Use the one from general function!
        static double getAngle(const cv::Point &pt1, const cv::Point &pt2, const cv::Point &pt0) {
            double dx1 = pt1.x - pt0.x;
            double dy1 = pt1.y - pt0.y;
            double dx2 = pt2.x - pt0.x;
            double dy2 = pt2.y - pt0.y;
            return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
        }

    private:
        cv::Mat output_image_;

        RangedParameter<int> N;
        RangedParameter<int> thresh;

        const cv::Point anchor_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_SQUARE_DETECTOR_H
