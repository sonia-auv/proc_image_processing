#ifndef PROC_IMAGE_PROCESSING_ALGORITHM_TYPE_AND_CONST_H_
#define PROC_IMAGE_PROCESSING_ALGORITHM_TYPE_AND_CONST_H_

#include <memory>
#include <opencv2/opencv.hpp>

namespace proc_image_processing {

    constexpr int NEXT_CTR = 0;
    constexpr int PREV_CTR = 1;
    constexpr int FIRST_CHILD_CTR = 2;
    constexpr int PARENT_CTR = 3;

    typedef std::vector<cv::Point> contour_t;
    typedef std::vector<contour_t> contourList_t;
    typedef std::vector<cv::Vec4i> hierarchy_t;
    typedef std::vector<cv::Vec4i> defectuosity_t;

    // Enum for the rotation function
    enum rotationType {
        R_NONE = 0, R_90, R_180, R_270
    };

    enum symmetryType {
        S_NONE = 0, S_X_AXIS, S_Y_AXIS, S_BOTH
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_ALGORITHM_TYPE_AND_CONST_H_
