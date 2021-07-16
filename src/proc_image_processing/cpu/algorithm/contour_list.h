/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROC_IMAGE_PROCESSING_ALGORITHM_CONTOUR_LIST_H_
#define PROC_IMAGE_PROCESSING_ALGORITHM_CONTOUR_LIST_H_

#include <memory>
#include <opencv2/opencv.hpp>
#include "contour.h"

namespace proc_image_processing {

    class [[maybe_unused]] ContourList {
    public:
        using Ptr = std::shared_ptr<ContourList>;

        typedef std::vector<Contour::ContourVec> ContourListVector;

        // Contour navigation constant
        [[maybe_unused]] static const unsigned int NEXT = 0;
        [[maybe_unused]] static const unsigned int PREVIOUS = 1;
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

        void drawContours(cv::Mat &img, const cv::Scalar &color, int thickness = 2) const;

        // Vector overload
        [[maybe_unused]] size_t getSize() const;

        [[maybe_unused]] std::vector<std::vector<cv::Point>> getAsPoints() const;

        [[maybe_unused]] std::vector<Contour> getAsContour() const;

        [[maybe_unused]] std::vector<cv::Vec4i> getHierarchy() const;

        ContourListVector contour_list_point_;

        std::vector<Contour> contour_vec_;

        // Contains the hierarchy when METHOD used is HIERARCHY
        std::vector<cv::Vec4i> hierarchy_;


    private:
        static bool hasChild(const cv::Vec4i &hierarchy_def);

        static bool hasParent(const cv::Vec4i &hierarchy_def);

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

    [[maybe_unused]] inline size_t ContourList::getSize() const { return contour_vec_.size(); }

    [[maybe_unused]] inline std::vector<std::vector<cv::Point>> ContourList::getAsPoints() const {
        return contour_list_point_;
    }

    [[maybe_unused]] inline std::vector<Contour> ContourList::getAsContour() const { return contour_vec_; }

    [[maybe_unused]] inline std::vector<cv::Vec4i> ContourList::getHierarchy() const { return hierarchy_; }

    inline Contour ContourList::operator[](size_t index) {
        return contour_vec_[index];
    }

    inline bool ContourList::hasChild(const cv::Vec4i &hierarchy_def) {
        return hierarchy_def[FIRST_CHILD] != -1;
    }

    inline bool ContourList::hasParent(const cv::Vec4i &hierarchy_def) {
        return hierarchy_def[PARENT] != -1;
    }

    inline void ContourList::drawContours(cv::Mat &img, const cv::Scalar &color, int thickness) const {
        cv::drawContours(img, contour_list_point_, -1, color, thickness);
    }

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_ALGORITHM_CONTOUR_LIST_H_
