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
    using Ptr = std::shared_ptr<PerformanceEvaluator>;

    PerformanceEvaluator();

    ~PerformanceEvaluator() {};

    // Return the time in second since construction or call to UpdateStartTime
    double GetExecTimeSec();

    // Reset the time reference
    void UpdateStartTime();

  private:
    double tick_frequency_;
    double start_tick_count_;
  };

  inline double PerformanceEvaluator::GetExecTimeSec() {
    return (cv::getTickCount() - start_tick_count_) / tick_frequency_;
  }

  inline void PerformanceEvaluator::UpdateStartTime() {
    start_tick_count_ = cv::getTickCount();
  }

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_ALGORITHM_PERFORMANCE_EVALUATOR_H_
