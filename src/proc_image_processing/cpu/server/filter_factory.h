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

#include <filters/adaptive_threshold.h>
#include <filters/background_substract.h>
#include <filters/bilateral_filter.h>
#include <filters/blurr.h>
#include <filters/buoy_single.h>
#include <filters/canny.h>
#include <filters/convex_hull.h>
#include <filters/delorean_detector.h>
#include <filters/dilate.h>
#include <filters/erode.h>
#include <filters/fence_detector.h>
#include <filters/filter.h>
#include <filters/handle_detector.h>
#include <filters/hough_line.h>
#include <filters/image_accumulator.h>
#include <filters/in_range.h>
#include <filters/laplacian.h>
#include <filters/mission_test_fake_string.h>
#include <filters/morphology.h>
#include <filters/object_feature_calculator.h>
#include <filters/object_finder.h>
#include <filters/original_image.h>
#include <filters/pipe_detector.h>
#include <filters/rotate.h>
#include <filters/remove_mask.h>
#include <filters/hsv_threshold.h>
#include <filters/contrast_brightness.h>
#include <filters/equalize.h>
#include <filters/schar_adding.h>
#include <filters/scharr.h>
#include <filters/sobel.h>
#include <filters/stats_threshold.h>
#include <filters/submarine_frame_masker.h>
#include <filters/subtract_all_planes.h>
#include <filters/test_filter.h>
#include <filters/threshold.h>
#include <filters/torpedoes_detector.h>
#include <filters/track_detector.h>
#include <filters/train_detector.h>
#include <filters/white_noise_takedown.h>
#include <filters/image_cropper.h>
#include <filters/gate_finder.h>
#include <filters/square_detection.h>
#include <filters/white_filter.h>
#include <filters/deep_dice.h>
#include <filters/pipe_detector_angle.h>
#include <filters/deep_2019.h>
#include <filters/vampire_torpidoes_detector.h>
#include <filters/vampire_body_detector.h>
#include <filters/vampire_torpidoes_detector_close.h>
#include <filters/threshold_between.h>
#include <filters/center_coffin_detector.h>
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
  static Filter *createInstance(const std::string &name,
                                const GlobalParamHandler &globalParams);

  static std::string GetFilterList();
};

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTER_FACTORY_H_
