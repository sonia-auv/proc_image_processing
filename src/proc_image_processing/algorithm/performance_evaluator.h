/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROVIDER_VISION_ALGORITHM_PERFORMANCE_EVALUATOR_H_
#define PROVIDER_VISION_ALGORITHM_PERFORMANCE_EVALUATOR_H_

#include <memory>
#include "opencv2/opencv.hpp"

namespace proc_image_processing {

  /*
   * Class to easily calculate process time
   * At construction, it takes the current tick count
   * and the tick frequency.
   * After that, when you call GetExecTime, it returns
   * the time in millisecond since its construction.
   * you can update the start time (time since construction)
   * by calling UpdateStartTime()
   */
  class PerformanceEvaluator {
  public:
    //==========================================================================
    // T Y P E D E F   A N D   E N U M

    using Ptr = std::shared_ptr<PerformanceEvaluator>;

    //============================================================================
    // P U B L I C   C / D T O R S

    PerformanceEvaluator();

    ~PerformanceEvaluator() {};

    //============================================================================
    // P U B L I C   M E T H O D S

    // Return the time in second since construction or call to UpdateStartTime
    double GetExecTimeSec();

    // Reset the time reference
    void UpdateStartTime();

  private:
    // P R I V A T E   M E M B E R S

    //============================================================================
    double tick_frequency_;

    double start_tick_count_;
  };

  //==============================================================================
  // I N L I N E   F U N C T I O N S   D E F I N I T I O N S

  //------------------------------------------------------------------------------
  //
  inline double PerformanceEvaluator::GetExecTimeSec() {
    return (cv::getTickCount() - start_tick_count_) / tick_frequency_;
  }

  //------------------------------------------------------------------------------
  //
  inline void PerformanceEvaluator::UpdateStartTime() {
    start_tick_count_ = cv::getTickCount();
  }

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_ALGORITHM_PERFORMANCE_EVALUATOR_H_
