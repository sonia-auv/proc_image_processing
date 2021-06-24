/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>


#ifndef PROVIDER_VISION_FILTER_FACTORY_H_
#define PROVIDER_VISION_FILTER_FACTORY_H_
// <FACTORY_GENERATOR_HEADER_INCLUDES>
#include <proc_image_processing/filters/adaptive_threshold.h>
#include <proc_image_processing/filters/background_substract.h>
#include <proc_image_processing/filters/bilateral_filter.h>
#include <proc_image_processing/filters/blurr.h>
#include <proc_image_processing/filters/canny.h>
#include <proc_image_processing/filters/contrast_brightness.h>
#include <proc_image_processing/filters/convex_hull.h>
#include <proc_image_processing/filters/deep_2019.h>
#include <proc_image_processing/filters/dilate.h>
#include <proc_image_processing/filters/equalize.h>
#include <proc_image_processing/filters/erode.h>
#include <proc_image_processing/filters/hough_line.h>
#include <proc_image_processing/filters/hsv_threshold.h>
#include <proc_image_processing/filters/image_accumulator.h>
#include <proc_image_processing/filters/image_cropper.h>
#include <proc_image_processing/filters/in_range.h>
#include <proc_image_processing/filters/laplacian.h>
#include <proc_image_processing/filters/mission_test_fake_string.h>
#include <proc_image_processing/filters/morphology.h>
#include <proc_image_processing/filters/original_image.h>
#include <proc_image_processing/filters/remove_mask.h>
#include <proc_image_processing/filters/rotate.h>
#include <proc_image_processing/filters/scharr.h>
#include <proc_image_processing/filters/schar_adding.h>
#include <proc_image_processing/filters/sobel.h>
#include <proc_image_processing/filters/square_detection.h>
#include <proc_image_processing/filters/stats_threshold.h>
#include <proc_image_processing/filters/submarine_frame_masker.h>
#include <proc_image_processing/filters/subtract_all_planes.h>
#include <proc_image_processing/filters/test_filter.h>
#include <proc_image_processing/filters/threshold.h>
#include <proc_image_processing/filters/threshold_between.h>
#include <proc_image_processing/filters/white_filter.h>
#include <proc_image_processing/filters/white_noise_takedown.h>
#include <proc_image_processing/filters/detectors/center_coffin_detector.h>
#include <proc_image_processing/filters/detectors/fence_detector.h>
#include <proc_image_processing/filters/detectors/gate_detector.h>
#include <proc_image_processing/filters/detectors/handle_detector.h>
#include <proc_image_processing/filters/detectors/pipe_angle_detector.h>
#include <proc_image_processing/filters/detectors/vampire_body_detector.h>
#include <proc_image_processing/filters/detectors/vampire_torpedoes_close_detector.h>
#include <proc_image_processing/filters/detectors/vampire_torpedoes_detector.h>
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
    static std::unique_ptr<Filter> createInstance(const std::string& name, const GlobalParamHandler& globalParams);

    static std::string GetFilterList();
  };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTER_FACTORY_H_
