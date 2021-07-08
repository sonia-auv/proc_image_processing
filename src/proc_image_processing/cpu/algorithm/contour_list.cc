/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#include "contour_list.h"

namespace proc_image_processing {

    ContourList::ContourList(const cv::Mat &image, const METHOD method) {
        switch (method) {
            case ALL:
                RetrieveAllContours(image);
                break;
            case INNER:
                RetrieveInnerContours(image);
                break;
            case INNER_MOST:
                RetrieveInnerMostContours(image);
                break;
            case OUTER:
                RetrieveOuterContours(image);
                break;
            case OUTER_NO_CHILD:
                RetrieveOutNoChildContours(image);
                break;
            case HIERARCHY:
                RetrieveHierarchyContours(image);
                break;
            default:
                break;
        }
        for (const auto &ctr : contour_list_point_) {
            contour_vec_.emplace_back(ctr);
        }
    }

    void ContourList::RetrieveAllContours(const cv::Mat &image) {
        // Clone because find contour modifies the image.
        cv::findContours(image.clone(), contour_list_point_, CV_RETR_LIST,
                         CV_CHAIN_APPROX_SIMPLE);
    }

    void ContourList::RetrieveHierarchyContours(const cv::Mat &image) {
        cv::findContours(image.clone(), contour_list_point_, hierarchy_, CV_RETR_TREE,
                         CV_CHAIN_APPROX_SIMPLE);
    }

    void ContourList::RetrieveInnerContours(const cv::Mat &image) {
        cv::findContours(image.clone(), contour_list_point_, hierarchy_, CV_RETR_TREE,
                         CV_CHAIN_APPROX_SIMPLE);

        std::vector<std::vector<cv::Point>> new_contour_list;

        for (size_t i = 0, size = hierarchy_.size(); i < size; i++) {
            if (HasParent(hierarchy_[i])) {
                new_contour_list.push_back((contour_list_point_)[i]);
            }
        }

        std::swap(contour_list_point_, new_contour_list);
    }

    void ContourList::RetrieveInnerMostContours(const cv::Mat &image) {
        cv::findContours(image.clone(), contour_list_point_, hierarchy_, CV_RETR_TREE,
                         CV_CHAIN_APPROX_SIMPLE);

        std::vector<std::vector<cv::Point>> new_contour_list;

        for (size_t i = 0, size = hierarchy_.size(); i < size; i++) {
            if (HasParent(hierarchy_[i]) && !HasChild(hierarchy_[i])) {
                new_contour_list.push_back((contour_list_point_)[i]);
            }
        }

        std::swap(contour_list_point_, new_contour_list);
    }

    void ContourList::RetrieveOuterContours(const cv::Mat &image) {
        cv::findContours(image.clone(), contour_list_point_, hierarchy_, CV_RETR_TREE,
                         CV_CHAIN_APPROX_SIMPLE);

        std::vector<std::vector<cv::Point>> new_contour_list;

        for (size_t i = 0, size = hierarchy_.size(); i < size; i++) {
            if (!HasParent(hierarchy_[i])) {
                new_contour_list.push_back((contour_list_point_)[i]);
            }
        }

        std::swap(contour_list_point_, new_contour_list);
    }

    void ContourList::RetrieveOutNoChildContours(const cv::Mat &image) {
        cv::findContours(image.clone(), contour_list_point_, hierarchy_, CV_RETR_TREE,
                         CV_CHAIN_APPROX_SIMPLE);

        std::vector<std::vector<cv::Point>> new_contour_list;

        for (size_t i = 0, size = hierarchy_.size(); i < size; i++) {
            if (!HasParent(hierarchy_[i]) && !HasChild(hierarchy_[i])) {
                new_contour_list.push_back((contour_list_point_)[i]);
            }
        }

        std::swap(contour_list_point_, new_contour_list);
    }

}  // namespace proc_image_processing
