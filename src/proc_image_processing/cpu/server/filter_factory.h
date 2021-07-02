/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>
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

#ifndef PROVIDER_VISION_FILTER_FACTORY_H_
#define PROVIDER_VISION_FILTER_FACTORY_H_

// <FACTORY_GENERATOR_HEADER_INCLUDES>
#include <proc_image_processing/cpu/filters/adaptive_threshold.h>
#include <proc_image_processing/cpu/filters/background_substract.h>
#include <proc_image_processing/cpu/filters/bilateral_filter.h>
#include <proc_image_processing/cpu/filters/blurr.h>
#include <proc_image_processing/cpu/filters/canny.h>
#include <proc_image_processing/cpu/filters/detectors/center_coffin_detector.h>
#include <proc_image_processing/cpu/filters/contrast_brightness.h>
#include <proc_image_processing/cpu/filters/convex_hull.h>
#include <proc_image_processing/cpu/filters/deep_2019.h>
#include <proc_image_processing/cpu/filters/dilate.h>
#include <proc_image_processing/cpu/filters/equalize.h>
#include <proc_image_processing/cpu/filters/erode.h>
#include <proc_image_processing/cpu/filters/detectors/fence_detector.h>
#include <proc_image_processing/cpu/filters/detectors/gate_detector.h>
#include <proc_image_processing/cpu/filters/detectors/handle_detector.h>
#include <proc_image_processing/cpu/filters/hough_line.h>
#include <proc_image_processing/cpu/filters/hsv_threshold.h>
#include <proc_image_processing/cpu/filters/image_accumulator.h>
#include <proc_image_processing/cpu/filters/image_cropper.h>
#include <proc_image_processing/cpu/filters/in_range.h>
#include <proc_image_processing/cpu/filters/laplacian.h>
#include <proc_image_processing/cpu/filters/mission_test_fake_string.h>
#include <proc_image_processing/cpu/filters/morphology.h>
#include <proc_image_processing/cpu/filters/original_image.h>
#include <proc_image_processing/cpu/filters/detectors/pipe_angle_detector.h>
#include <proc_image_processing/cpu/filters/remove_mask.h>
#include <proc_image_processing/cpu/filters/rotate.h>
#include <proc_image_processing/cpu/filters/schar_adding.h>
#include <proc_image_processing/cpu/filters/scharr.h>
#include <proc_image_processing/cpu/filters/sobel.h>
#include <proc_image_processing/cpu/filters/square_detection.h>
#include <proc_image_processing/cpu/filters/stats_threshold.h>
#include <proc_image_processing/cpu/filters/submarine_frame_masker.h>
#include <proc_image_processing/cpu/filters/subtract_all_planes.h>
#include <proc_image_processing/cpu/filters/test_filter.h>
#include <proc_image_processing/cpu/filters/threshold.h>
#include <proc_image_processing/cpu/filters/threshold_between.h>
#include <proc_image_processing/cpu/filters/detectors/vampire_body_detector.h>
#include <proc_image_processing/cpu/filters/detectors/vampire_torpedoes_close_detector.h>
#include <proc_image_processing/cpu/filters/detectors/vampire_torpedoes_detector.h>
#include <proc_image_processing/cpu/filters/white_filter.h>
#include <proc_image_processing/cpu/filters/white_noise_takedown.h>
// <FACTORY_GENERATOR_HEADER_INCLUDES/>
#include <memory>
#include <string>

namespace proc_image_processing {

// Class that provides an interface
// for the proc_image_processing project.
// It enables instantiation via a string
// and holds the list of all the filters.
class FilterFactory {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<FilterFactory>;

  // KEEPING A REFERENCE TO GlobalParamHandler. VERY IMPORTANT
  static std::unique_ptr<Filter> createInstance(const std::string &name, const GlobalParamHandler &globalParams);

  static std::string GetFilterList();
};

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTER_FACTORY_H_
