#include "performance_evaluator.h"

namespace proc_image_processing {

    PerformanceEvaluator::PerformanceEvaluator()
            : tick_frequency_(cv::getTickFrequency()),
              start_tick_count_(cv::getTickCount()) {
        // Should never occur but...
        if (tick_frequency_ == 0.0f) {
            tick_frequency_ = 1;
        }
    }

}  // namespace proc_image_processing
