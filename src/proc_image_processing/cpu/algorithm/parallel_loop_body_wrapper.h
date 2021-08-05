#ifndef PROC_IMAGE_PROCESSING_PARALLEL_LOOP_BODY_WRAPPER_H_
#define PROC_IMAGE_PROCESSING_PARALLEL_LOOP_BODY_WRAPPER_H_

#include "opencv2/opencv.hpp"

namespace proc_image_processing {

    /*
     * This class can be derived to create a "body" for the OpenCV parallel_for_() function
     */
    class ParallelLoopBodyWrapper : public cv::ParallelLoopBody {
    public:
        ParallelLoopBodyWrapper() {}

        virtual ~ParallelLoopBodyWrapper() = default;

        virtual void operator()(const cv::Range &range) const = 0;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_PARALLEL_LOOP_BODY_WRAPPER_H_