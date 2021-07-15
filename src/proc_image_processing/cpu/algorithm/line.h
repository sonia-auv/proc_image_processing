/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROVIDER_VISION_ALGORITHM_LINE_H_
#define PROVIDER_VISION_ALGORITHM_LINE_H_

#include <math.h>
#include <stdlib.h>
#include <memory>
#include <opencv2/imgproc.hpp>
#include <eigen3/Eigen/Eigen>

namespace proc_image_processing {

  /**
   * Basic definitnion of a line
   */
  class Line {
  public:


    using Ptr = std::shared_ptr<Line>;



    Line(const cv::Point& start, const cv::Point& end);



    // Debug
    void draw(cv::Mat &img, cv::Scalar color);

      cv::Point getPerpendicularLine();

      cv::Point getCenter();

      cv::Point getStart();

      cv::Point getEnd();

      std::vector<cv::Point> getPoints(cv::Mat &img);

      float getAngle();

      float getLength();

  private:
      // start point is leftmost point, end id rightmost point
    cv::Point center_;

    cv::Point start_;

    cv::Point end_;

    // Degree
    float angle_;

    float length_;
    bool  isSwap_;
  };

  bool lengthSort(Line a, Line b);

  bool centerXSort(Line a, Line b);

  bool centerYSort(Line a, Line b);

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_ALGORITHM_LINE_H_
