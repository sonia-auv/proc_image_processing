/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROVIDER_VISION_ALGORITHM_CONTOUR_LIST_H_
#define PROVIDER_VISION_ALGORITHM_CONTOUR_LIST_H_

#include <memory>
#include <opencv2/opencv.hpp>
#include "proc_image_processing/algorithm/contour.h"

namespace proc_image_processing {

  class ContourList {
  public:
    using Ptr = std::shared_ptr<ContourList>;

    typedef std::vector<Contour::ContourVec> ContourListVector;

    // Contour navigation constant
    static const unsigned int NEXT = 0;
    static const unsigned int PREVIOUS = 1;
    static const unsigned int FIRST_CHILD = 2;
    static const unsigned int PARENT = 3;

    enum METHOD {
      ALL,             // All the contour in the image
      INNER,           // Every contour with a parent
      INNER_MOST,      // Every contour with a parent AND no child
      OUTER,           // Every contour without a parent
      OUTER_NO_CHILD,  // Ever contour without a parent AND no child
      HIERARCHY        // All the contours, fills the hierarchy member
    };

    ContourList(const cv::Mat& image, const METHOD method);

    Contour operator[](size_t index);

    void DrawContours(cv::Mat& img, const cv::Scalar& color, int thickness = 2);

    // Vector overload
    size_t GetSize();

    std::vector<std::vector<cv::Point>> GetAsPoint();

    std::vector<Contour> GetAsContour();

    std::vector<cv::Vec4i> GetHierachy();

    ContourListVector contour_list_point_;

    std::vector<Contour> contour_vec_;

    // Contains the hierachy when METHOD used is HIERACHY
    std::vector<cv::Vec4i> hierarchy_;


  private:
    bool HasChild(const cv::Vec4i& hierarchy_def);

    bool HasParent(const cv::Vec4i& hierarchy_def);

    // Retrieval method
    void RetrieveAllContours(const cv::Mat& image);

    // Retrieve contour with hierachy. Sets the vector _hierachy of this object.
    void RetrieveHiearchyContours(const cv::Mat& image);

    // All innermost contour i.e. no child
    void RetrieveInnerContours(const cv::Mat& image);

    // All inner contour i.e. has a parent
    void RetrieveInnerMostContours(const cv::Mat& image);

    // All outer contour i.e. has no parent
    void RetrieveOuterContours(const cv::Mat& image);

    // All contour that has no child AND no parent
    void RetrieveOutNoChildContours(const cv::Mat& image);
  };

  inline size_t ContourList::GetSize() { return contour_vec_.size(); }

  inline std::vector<std::vector<cv::Point>> ContourList::GetAsPoint() {
    return contour_list_point_;
  }

  inline std::vector<Contour> ContourList::GetAsContour() { return contour_vec_; }

  inline std::vector<cv::Vec4i> ContourList::GetHierachy() { return hierarchy_; }

  inline Contour ContourList::operator[](size_t index) {
    return contour_vec_[index];
  }

  inline bool ContourList::HasChild(const cv::Vec4i& hierarchy_def) {
    return hierarchy_def[FIRST_CHILD] != -1;
  }

  inline bool ContourList::HasParent(const cv::Vec4i& hierarchy_def) {
    return hierarchy_def[PARENT] != -1;
  }

  inline void ContourList::DrawContours(cv::Mat& img, const cv::Scalar& color,
    int thickness) {
    cv::drawContours(img, contour_list_point_, -1, color, thickness);
  }

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_ALGORITHM_CONTOUR_LIST_H_
