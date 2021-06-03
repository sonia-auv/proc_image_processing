/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
/// \section LICENSE
/// This file is part of S.O.N.I.A. software.
///
/// S.O.N.I.A. software is free software: you can redistribute it and/or modify
/// it under the terms of the GNU General Public License as published by
/// the Free Software Foundation, either version 3 of the License, or
/// (at your option) any later version.
///
/// S.O.N.I.A. software is distributed in the hope that it will be useful,
/// but WITHOUT ANY WARRANTY; without even the implied warranty of
/// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
/// GNU General Public License for more details.
///
/// You should have received a copy of the GNU General Public License
/// along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.

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

  ~PerformanceEvaluator(){};

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
