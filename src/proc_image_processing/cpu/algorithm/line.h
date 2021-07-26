#ifndef PROC_IMAGE_PROCESSING_ALGORITHM_LINE_H_
#define PROC_IMAGE_PROCESSING_ALGORITHM_LINE_H_

#include <cmath>
#include <cstdlib>
#include <memory>
#include <opencv2/imgproc.hpp>
#include <eigen3/Eigen/Eigen>

namespace proc_image_processing {

    /**
     * Basic definitnion of a line
     */
    class Line {
    public:
        using Ptr = std::shared_ptr<Line>;

        Line(const cv::Point &start, const cv::Point &end);

        // Debug
        void draw(cv::Mat &img, const cv::Scalar &color);

        cv::Point getPerpendicularLine();

        cv::Point getCenter();

        cv::Point getStart();

        cv::Point getEnd();

        std::vector<cv::Point> getPoints(cv::Mat &img);

        float getAngle() const;

        float getLength() const;

    private:
        // start point is leftmost point, end id rightmost point
        cv::Point center_;

        cv::Point start_;

        cv::Point end_;

        // Degree
        float angle_;

        float length_;
        bool isSwap_;
    };

    bool lengthSort(const Line &a, const Line &b);

    bool centerXSort(Line a, Line b);

    bool centerYSort(Line a, Line b);

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_ALGORITHM_LINE_H_
