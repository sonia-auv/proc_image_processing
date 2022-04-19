#ifndef PROC_IMAGE_PROCESSING_ALGORITHM_CONTOUR_H_
#define PROC_IMAGE_PROCESSING_ALGORITHM_CONTOUR_H_

#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>

namespace proc_image_processing {

    class Contour {
    public:
        using Ptr = std::shared_ptr<Contour>;

        using ContourVec = std::vector<cv::Point>;

        explicit Contour(std::vector<cv::Point> ctr);

        explicit Contour(const cv::RotatedRect &rect);

        cv::Point operator[](unsigned int index);

        /**
         * Approximate contours and merges vertex together
         */
        void approximate(double accuracy);

        void approximateBySize();

        /**
         * draw contour in the image.
         */
        void drawContours(cv::Mat &image, const cv::Scalar &color, int thickness) const;

        size_t getSize() const;

        std::vector<cv::Point> getContour() const;

    private:
        std::vector<cv::Point> contour_;
    };

    inline void Contour::approximate(double accuracy) {
        std::vector<cv::Point> output;
        cv::approxPolyDP(contour_, output, accuracy, false);
        std::swap(contour_, output);
    }

    inline void Contour::approximateBySize() {
        double arc_length = 0.1 * cv::arcLength(contour_, true);
        std::vector<cv::Point> output;
        cv::approxPolyDP(contour_, output, arc_length, false);
        std::swap(contour_, output);
    }

    inline void Contour::drawContours(cv::Mat &image, const cv::Scalar &color, int thickness) const {
        std::vector<ContourVec> contours;
        contours.push_back(contour_);
        cv::drawContours(image, contours, -1, color, thickness);
    }

    inline size_t Contour::getSize() const { return contour_.size(); }

    inline std::vector<cv::Point> Contour::getContour() const { return contour_; }

    inline cv::Point Contour::operator[](unsigned int index) {
        return contour_[index];
    }

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_ALGORITHM_CONTOUR_H_
