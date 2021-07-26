/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROVIDER_VISION_ALGORITHM_TYPE_AND_CONST_H_
#define PROVIDER_VISION_ALGORITHM_TYPE_AND_CONST_H_

#include <memory>
#include <opencv2/opencv.hpp>

namespace proc_image_processing {

  typedef std::vector<cv::Point> contour_t;
  typedef std::vector<contour_t> contourList_t;
  typedef std::vector<cv::Vec4i> hierachy_t;

#define NEXT_CTR 0
#define PREV_CTR 1
#define FIRST_CHILD_CTR 2
#define PARENT_CTR 3

  typedef std::vector<cv::Vec4i> defectuosity_t;

  // Enum for the rotation function
  enum rotationType { R_NONE = 0, R_90, R_180, R_270 };

  enum symmetryType { S_NONE = 0, S_X_AXIS, S_Y_AXIS, S_BOTH };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_ALGORITHM_TYPE_AND_CONST_H_
