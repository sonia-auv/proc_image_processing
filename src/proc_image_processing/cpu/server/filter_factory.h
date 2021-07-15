/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>


#ifndef PROVIDER_VISION_FILTER_FACTORY_H_
#define PROVIDER_VISION_FILTER_FACTORY_H_

// <FACTORY_GENERATOR_HEADER_INCLUDES>
#include <proc_image_processing/cpu/filters/adaptive_threshold_filter.h>
#include <proc_image_processing/cpu/filters/background_subtract_filter.h>
#include <proc_image_processing/cpu/filters/bilateral_filter.h>
#include <proc_image_processing/cpu/filters/blurr_filter.h>
#include <proc_image_processing/cpu/filters/canny_filter.h>
#include <proc_image_processing/cpu/filters/detectors/center_coffin_detector.h>
#include <proc_image_processing/cpu/filters/contrast_and_brightness_filter.h>
#include <proc_image_processing/cpu/filters/convex_hull_filter.h>
#include <proc_image_processing/cpu/filters/deep_2019_filter.h>
#include <proc_image_processing/cpu/filters/dilate_filter.h>
#include <proc_image_processing/cpu/filters/equalize_filter.h>
#include <proc_image_processing/cpu/filters/erode_filter.h>
#include <proc_image_processing/cpu/filters/detectors/fence_detector.h>
#include <proc_image_processing/cpu/filters/detectors/gate_detector.h>
#include <proc_image_processing/cpu/filters/detectors/handle_detector.h>
#include <proc_image_processing/cpu/filters/hough_line_filter.h>
#include <proc_image_processing/cpu/filters/hsv_threshold_filter.h>
#include <proc_image_processing/cpu/filters/accumulator_filter.h>
#include <proc_image_processing/cpu/filters/crop_filter.h>
#include <proc_image_processing/cpu/filters/in_range_filter.h>
#include <proc_image_processing/cpu/filters/laplacian_filter.h>
#include <proc_image_processing/cpu/filters/mission_test_fake_string_filter.h>
#include <proc_image_processing/cpu/filters/morphology_filter.h>
#include <proc_image_processing/cpu/filters/original_image_filter.h>
#include <proc_image_processing/cpu/filters/detectors/pipe_angle_detector.h>
#include <proc_image_processing/cpu/filters/remove_mask_filter.h>
#include <proc_image_processing/cpu/filters/rotate_filter.h>
#include <proc_image_processing/cpu/filters/scharr_adding_filter.h>
#include <proc_image_processing/cpu/filters/scharr.h>
#include <proc_image_processing/cpu/filters/sobel_filter.h>
#include <proc_image_processing/cpu/filters/detectors/square_detector.h>
#include <proc_image_processing/cpu/filters/statistical_threshold_filter.h>
#include <proc_image_processing/cpu/filters/hide_submarine_frame_filter.h>
#include <proc_image_processing/cpu/filters/subtract_all_planes_filter.h>
#include <proc_image_processing/cpu/filters/test_filter.h>
#include <proc_image_processing/cpu/filters/threshold_filter.h>
#include <proc_image_processing/cpu/filters/interval_threshold_filter.h>
#include <proc_image_processing/cpu/filters/detectors/vampire_body_detector.h>
#include <proc_image_processing/cpu/filters/detectors/vampire_torpedoes_close_detector.h>
#include <proc_image_processing/cpu/filters/detectors/vampire_torpedoes_detector.h>
#include <proc_image_processing/cpu/filters/white_filter.h>
#include <proc_image_processing/cpu/filters/white_noise_take_down_filter.h>
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

  using Ptr = std::shared_ptr<FilterFactory>;

  // KEEPING A REFERENCE TO GlobalParamHandler. VERY IMPORTANT
  static std::unique_ptr<Filter> createInstance(const std::string &name, const GlobalParamHandler &globalParams);

  static std::string getFilters();
};

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTER_FACTORY_H_
