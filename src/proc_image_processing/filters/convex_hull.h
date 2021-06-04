/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROVIDER_VISION_FILTERS_CONVEX_HULL_H_
#define PROVIDER_VISION_FILTERS_CONVEX_HULL_H_

#include <proc_image_processing/filters/filter.h>
#include <memory>

namespace proc_image_processing {

  class ConvexHull : public Filter {
  public:
    //==========================================================================
    // T Y P E D E F   A N D   E N U M

    using Ptr = std::shared_ptr<ConvexHull>;

    //============================================================================
    // P U B L I C   C / D T O R S

    explicit ConvexHull(const GlobalParamHandler& globalParams)
      : Filter(globalParams),
      enable_("Enable", false, &parameters_),
      mode_("Mode", 0, 0, 3, &parameters_,
        "0=CV_RETR_EXTERNAL,1=CV_RETR_LIST, 2=CV_RETR_CCOMP, "
        "3=CV_RETR_TREE"),
      method_("Method", 0, 0, 3, &parameters_,
        "0=CV_CHAIN_APPROX_NONE, 1=CV_CHAIN_APPROX_SIMPLE, "
        "2=CV_CHAIN_APPROX_TC89_L1, "
        "3=CV_CHAIN_APPROX_TC89_KCOS") {
      SetName("ConvexHull");
    }

    virtual ~ConvexHull() {}

    //============================================================================
    // P U B L I C   M E T H O D S

    virtual void Execute(cv::Mat& image) {
      if (enable_()) {
        int mode, method;
        switch (mode_()) {
        case 0:
          mode = char(CV_RETR_EXTERNAL);
          break;
        case 1:
          mode = char(CV_RETR_LIST);
          break;
        case 2:
          mode = char(CV_RETR_CCOMP);
          break;
        case 3:
          mode = char(CV_RETR_TREE);
          break;
        }
        switch (method_()) {
        case 0:
          method = char(CV_CHAIN_APPROX_NONE);
          break;
        case 1:
          method = char(CV_CHAIN_APPROX_SIMPLE);
          break;
        case 2:
          method = char(CV_CHAIN_APPROX_TC89_L1);
          break;
        case 3:
          method = char(CV_CHAIN_APPROX_TC89_KCOS);
          break;
        }
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;

        // Find contours
        cv::findContours(image, contours, hierarchy, mode, method,
          cv::Point(0, 0));

        // Find the convex hull object for each contour
        std::vector<std::vector<cv::Point>> hull(contours.size());
        for (size_t i = 0; i < contours.size(); i++) {
          cv::convexHull(cv::Mat(contours[i]), hull[i], false);
        }

        // Draw Hull contour
        image = cv::Mat::zeros(image.size(), CV_8UC1);
        for (size_t i = 0; i < contours.size(); i++) {
          cv::drawContours(image, hull, i, cv::Scalar(255, 255, 255), CV_FILLED);
        }
      }
    }


  private:
    //============================================================================
    // P R I V A T E   M E M B E R S

    Parameter<bool> enable_;
    RangedParameter<int> mode_, method_;
  };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_CONVEX_HULL_H_
