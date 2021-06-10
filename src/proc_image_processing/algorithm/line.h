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
    void Draw(cv::Mat& img, cv::Scalar color);

    cv::Point PerpendicularLine();

    // Getters
    cv::Point GetCenter();

    cv::Point GetStart();

    cv::Point GetEnd();

    std::vector<cv::Point> GenerateLine(cv::Mat& img);

    float GetAngle();

    float GetLength();

  private:
    // start point is leftmosst point, end id farrigth point
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
