/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROVIDER_VISION_ALGORITHM_ROT_RECT_H_
#define PROVIDER_VISION_ALGORITHM_ROT_RECT_H_

#include <math.h>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

namespace proc_image_processing {

#define PROVIDER_VISION_TOP_LEFT 0
#define PROVIDER_VISION_TOP_RIGHT 4
#define PROVIDER_VISION_BOTTOM_LEFT 1
#define PROVIDER_VISION_BOTTOM_RIGHT 2

  // Rotated rect ensure that the height member is the longest one
  // and the angle is in the direction of the height
  // it also contains usefull method to play with rotated rectangle
  class RotRect : public cv::RotatedRect {
  public:
    //==========================================================================
    // T Y P E D E F   A N D   E N U M

    using Ptr = std::shared_ptr<RotRect>;

    //============================================================================
    // P U B L I C   C / D T O R S

    // Constructor/destructor
    RotRect(const std::vector<cv::Point>& edges);

    RotRect(const cv::RotatedRect& rotRect);

    RotRect(const RotRect& a);

    RotRect();

    ~RotRect();

    //============================================================================
    // P U B L I C   O P E R A T O R S

    RotRect& operator=(RotRect rotRect);

    RotRect& operator=(cv::RotatedRect rotRect);

    bool operator==(const RotRect& rotRect);

    //============================================================================
    // P U B L I C   M E T H O D S

    void drawRect(cv::Mat& out, cv::Scalar color, int thickness = 1);

    // Create the class with another rotated rect
    void swap(RotRect& a);

    cv::Point2f* getCorners();

  private:
    //============================================================================
    // P R I V A T E   M E T H O D S

    // Set height to the longest side of the rectangle and
    void setValues();

    //============================================================================
    // P R I V A T E   M E M B E R S

    cv::Point2f pts_[4];
  };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_ALGORITHM_ROT_RECT_H_
