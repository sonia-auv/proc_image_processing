#ifndef PROC_IMAGE_PROCESSING_ALGORITHM_ROT_RECT_H_
#define PROC_IMAGE_PROCESSING_ALGORITHM_ROT_RECT_H_

#include <cmath>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

namespace proc_image_processing {
    // Rotated rect ensure that the height member is the longest one
    // and the getAngle is in the direction of the height
    // it also contains useful method to play with rotated rectangle
    class RotRect : public cv::RotatedRect {
    public:
        static constexpr int TOP_LEFT = 0;
        static constexpr int BOTTOM_LEFT = 1;
        static constexpr int BOTTOM_RIGHT = 2;
        static constexpr int TOP_RIGHT = 4;

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
        void swap(RotRect &a) noexcept;

        cv::Point2f *getCorners();

    private:
        // Set height to the longest side of the rectangle and
        void setValues();

        cv::Point2f pts_[4];
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_ALGORITHM_ROT_RECT_H_
