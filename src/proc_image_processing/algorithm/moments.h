/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROVIDER_VISION_ALGORITHM_MOMENTS_H_
#define PROVIDER_VISION_ALGORITHM_MOMENTS_H_

#include <math.h>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

namespace proc_image_processing {

  class Moments {
  public:
    //==========================================================================
    // T Y P E D E F   A N D   E N U M

    using Ptr = std::shared_ptr<Moments>;

    //============================================================================
    // P U B L I C   C / D T O R S

    Moments(cv::Mat image, bool binary);

    Moments();

    ~Moments();

    //============================================================================
    // P U B L I C   O P E R A T O R S

    void operator=(Moments moments);

    //============================================================================
    // P U B L I C   M E M B E R S

    // Points are in local coordinate, meaning that if the image was taken
    // from a rotated rectangle, the x and y a rotated too!
    cv::Point real_center_;

    cv::Point mass_center_;

    float y_distance_from_center_;

    float x_distance_from_center_;

    cv::Moments cv_moments_;
  };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_ALGORITHM_MOMENTS_H_
