#ifndef PROC_IMAGE_PROCESSING_FILTER_FACTORY_H_
#define PROC_IMAGE_PROCESSING_FILTER_FACTORY_H_

// <FACTORY_GENERATOR_HEADER_INCLUDES>
#include <proc_image_processing/cpu/filters/custom/accumulator_filter.h>
#include <proc_image_processing/cpu/filters/low_pass/thresholds/adaptive_threshold_filter.h>
#include <proc_image_processing/cpu/filters/low_pass/background_subtract_filter.h>
#include <proc_image_processing/cpu/filters/low_pass/bgr_filter.h>
#include <proc_image_processing/cpu/filters/low_pass/bilateral_filter.h>
#include <proc_image_processing/cpu/filters/low_pass/colors/single_color_filter.h>
#include <proc_image_processing/cpu/filters/low_pass/blur_filter.h>
#include <proc_image_processing/cpu/filters/custom/bounding_box_filter.h>
#include <proc_image_processing/cpu/filters/high_pass/edge_detection/canny_filter.h>
#include <proc_image_processing/cpu/filters/detectors/center_detector.h>
#include <proc_image_processing/cpu/filters/low_pass/clahe_filter.h>
#include <proc_image_processing/cpu/filters/detectors/contour_detector.h>
#include <proc_image_processing/cpu/filters/low_pass/contrast_and_brightness_filter.h>
#include <proc_image_processing/cpu/filters/high_pass/convex_hull_filter.h>
#include <proc_image_processing/cpu/filters/transformations/crop_filter.h>
#include <proc_image_processing/cpu/filters/custom/deep_filter.h>
#include <proc_image_processing/cpu/filters/transformations/dilate_filter.h>
#include <proc_image_processing/cpu/filters/detectors/ellipse_detector.h>
#include <proc_image_processing/cpu/filters/low_pass/equalize_histogram_filter.h>
#include <proc_image_processing/cpu/filters/transformations/erode_filter.h>
#include <proc_image_processing/cpu/filters/detectors/fence_detector.h>
#include <proc_image_processing/cpu/filters/detectors/gate_detector.h>
#include <proc_image_processing/cpu/filters/detectors/handle_detector.h>
#include <proc_image_processing/cpu/filters/custom/hide_submarine_frame_filter.h>
#include <proc_image_processing/cpu/filters/low_pass/hsv_filter.h>
#include <proc_image_processing/cpu/filters/high_pass/edge_detection/hough_line_filter.h>
#include <proc_image_processing/cpu/filters/low_pass/thresholds/hsv_threshold_filter.h>
#include <proc_image_processing/cpu/filters/custom/in_range_filter.h>
#include <proc_image_processing/cpu/filters/low_pass/thresholds/interval_threshold_filter.h>
#include <proc_image_processing/cpu/filters/low_pass/lab_filter.h>
#include <proc_image_processing/cpu/filters/high_pass/edge_detection/laplacian_filter.h>
#include <proc_image_processing/cpu/filters/custom/mission_test_fake_string_filter.h>
#include <proc_image_processing/cpu/filters/transformations/morphology_filter.h>
#include <proc_image_processing/cpu/filters/detectors/obstacle_detector.h>
#include <proc_image_processing/cpu/filters/detectors/orb_sift_match.h>
#include <proc_image_processing/cpu/filters/custom/original_image_filter.h>
#include <proc_image_processing/cpu/filters/detectors/pipe_angle_detector.h>
#include <proc_image_processing/cpu/filters/detectors/pipe_straight_detector.h>
#include <proc_image_processing/cpu/filters/low_pass/remove_mask_filter.h>
#include <proc_image_processing/cpu/filters/transformations/rotate_filter.h>
#include <proc_image_processing/cpu/filters/high_pass/edge_detection/scharr_adding_filter.h>
#include <proc_image_processing/cpu/filters/high_pass/edge_detection/scharr_filter.h>
#include <proc_image_processing/cpu/filters/detectors/shape_detector.h>
#include <proc_image_processing/cpu/filters/detectors/sift_match.h>
#include <proc_image_processing/cpu/filters/detectors/sift_calculator.h>
#include <proc_image_processing/cpu/filters/high_pass/edge_detection/sobel_filter.h>
#include <proc_image_processing/cpu/filters/detectors/square_detector.h>
#include <proc_image_processing/cpu/filters/low_pass/thresholds/statistical_threshold_filter.h>
#include <proc_image_processing/cpu/filters/low_pass/subtract_all_planes_filter.h>
#include <proc_image_processing/cpu/filters/low_pass/subtract_plane_adder_filter.h>
#include <proc_image_processing/cpu/filters/custom/test_filter.h>
#include <proc_image_processing/cpu/filters/low_pass/thresholds/threshold_filter.h>
#include <proc_image_processing/cpu/filters/detectors/vampire_body_detector.h>
#include <proc_image_processing/cpu/filters/detectors/vampire_torpedoes_close_detector.h>
#include <proc_image_processing/cpu/filters/detectors/vampire_torpedoes_detector.h>
#include <proc_image_processing/cpu/filters/low_pass/white_filter.h>
#include <proc_image_processing/cpu/filters/low_pass/white_noise_removal_filter.h>
#include <proc_image_processing/cpu/filters/low_pass/ycrcb_filter.h>
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
        static std::unique_ptr<Filter> createInstance(
                const std::string_view &name,
                const GlobalParameterHandler &gph
        );

        static std::string getFilters();
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTER_FACTORY_H_
