/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROVIDER_VISION_ALGORITHM_CONTOUR_LIST_H_
#define PROVIDER_VISION_ALGORITHM_CONTOUR_LIST_H_

#include <memory>
#include <opencv2/opencv.hpp>
#include "contour.h"

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

        ContourList(const cv::Mat &image, METHOD method);

        Contour operator[](size_t index);

        void drawContours(cv::Mat &img, const cv::Scalar &color, int thickness = 2);

        // Vector overload
        size_t getSize();

        std::vector<std::vector<cv::Point>> getAsPoints();

        std::vector<Contour> getAsContour();

        std::vector<cv::Vec4i> getHierarchy();

        ContourListVector contour_list_point_;

        std::vector<Contour> contour_vec_;

        // Contains the hierarchy when METHOD used is HIERARCHY
        std::vector<cv::Vec4i> hierarchy_;


    private:
        bool hasChild(const cv::Vec4i &hierarchy_def);

        bool hasParent(const cv::Vec4i &hierarchy_def);

        // Retrieval method
        void retrieveAllContours(const cv::Mat &image);

        // Retrieve contour with hierarchy. Sets the vector _hierarchy of this object.
        void retrieveHierarchyContours(const cv::Mat &image);

        // All innermost contour i.e. no child
        void retrieveInnerContours(const cv::Mat &image);

        // All inner contour i.e. has a parent
        void retrieveInnerMostContours(const cv::Mat &image);

        // All outer contour i.e. has no parent
        void retrieveOuterContours(const cv::Mat &image);

        // All contour that has no child AND no parent
        void retrieveNoChildAndParentContours(const cv::Mat &image);
    };

    inline size_t ContourList::getSize() { return contour_vec_.size(); }

    inline std::vector<std::vector<cv::Point>> ContourList::getAsPoints() {
        return contour_list_point_;
    }

    inline std::vector<Contour> ContourList::getAsContour() { return contour_vec_; }

    inline std::vector<cv::Vec4i> ContourList::getHierarchy() { return hierarchy_; }

    inline Contour ContourList::operator[](size_t index) {
        return contour_vec_[index];
    }

    inline bool ContourList::hasChild(const cv::Vec4i &hierarchy_def) {
        return hierarchy_def[FIRST_CHILD] != -1;
    }

    inline bool ContourList::hasParent(const cv::Vec4i &hierarchy_def) {
        return hierarchy_def[PARENT] != -1;
    }

    inline void ContourList::drawContours(cv::Mat &img, const cv::Scalar &color,
                                          int thickness) {
        cv::drawContours(img, contour_list_point_, -1, color, thickness);
    }

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_ALGORITHM_CONTOUR_LIST_H_
