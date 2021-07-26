#ifndef PROC_IMAGE_PROCESSING_ALGORITHM_ROT_RECT_H_
#define PROC_IMAGE_PROCESSING_ALGORITHM_ROT_RECT_H_

#include <cmath>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

namespace proc_image_processing {

#define PROC_IMAGE_PROCESSING_TOP_LEFT 0
#define PROC_IMAGE_PROCESSING_TOP_RIGHT 4
#define PROC_IMAGE_PROCESSING_BOTTOM_LEFT 1
#define PROC_IMAGE_PROCESSING_BOTTOM_RIGHT 2

    // Rotated rect ensure that the height member is the longest one
    // and the getAngle is in the direction of the height
    // it also contains useful method to play with rotated rectangle
    class RotRect : public cv::RotatedRect {
    public:
        using Ptr = std::shared_ptr<RotRect>;

        explicit RotRect(const std::vector<cv::Point> &edges);

        explicit RotRect(const cv::RotatedRect &rotRect);

        RotRect(const RotRect &a);

        RotRect();

        ~RotRect();

        RotRect &operator=(RotRect rotRect);

        RotRect &operator=(const cv::RotatedRect &rotRect);

        bool operator==(const RotRect &rotRect);

        void drawRect(cv::Mat &out, const cv::Scalar &color, int thickness = 1);

        // Create the class with another rotated rect
        void swap(RotRect &a);

        cv::Point2f *getCorners();

    private:
        // Set height to the longest side of the rectangle and
        void setValues();

        cv::Point2f pts_[4];
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_ALGORITHM_ROT_RECT_H_
