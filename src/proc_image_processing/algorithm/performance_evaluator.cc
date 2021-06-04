/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#include <proc_image_processing/algorithm/performance_evaluator.h>

namespace proc_image_processing {

  //==============================================================================
  // C / D T O R   S E C T I O N

  //------------------------------------------------------------------------------
  //
  PerformanceEvaluator::PerformanceEvaluator()
    : tick_frequency_(cv::getTickFrequency()),
    start_tick_count_(cv::getTickCount()) {
    // Should never occur but...
    if (tick_frequency_ == 0.0f) {
      tick_frequency_ = 1;
    }
  }

}  // namespace proc_image_processing
