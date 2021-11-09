#include "contour_list.h"

namespace proc_image_processing {

    ContourList::ContourList(const cv::Mat &image, const METHOD method) {
        switch (method) {
            case ALL:
                retrieveAllContours(image);
                break;
            case INNER:
                retrieveInnerContours(image);
                break;
            case INNER_MOST:
                retrieveInnerMostContours(image);
                break;
            case OUTER:
                retrieveOuterContours(image);
                break;
            case OUTER_NO_CHILD:
                retrieveNoChildAndParentContours(image);
                break;
            case HIERARCHY:
                retrieveHierarchyContours(image);
                break;
            default:
                break;
        }
        for (const auto &ctr : contour_list_point_) {
            contour_vec_.emplace_back(ctr);
        }
    }

    void ContourList::retrieveAllContours(const cv::Mat &image) {
        // Clone because find contour modifies the image.
        cv::findContours(image.clone(), contour_list_point_, CV_RETR_LIST,
                         CV_CHAIN_APPROX_SIMPLE);
    }

    void ContourList::retrieveHierarchyContours(const cv::Mat &image) {
        cv::findContours(image.clone(), contour_list_point_, hierarchy_, CV_RETR_TREE,
                         CV_CHAIN_APPROX_SIMPLE);
    }

    void ContourList::retrieveInnerContours(const cv::Mat &image) {
        cv::findContours(image.clone(), contour_list_point_, hierarchy_, CV_RETR_TREE,
                         CV_CHAIN_APPROX_SIMPLE);

        std::vector<std::vector<cv::Point>> new_contour_list;

        for (size_t i = 0, size = hierarchy_.size(); i < size; i++) {
            if (hasParent(hierarchy_[i])) {
                new_contour_list.push_back(contour_list_point_[i]);
            }
        }

        std::swap(contour_list_point_, new_contour_list);
    }

    void ContourList::retrieveInnerMostContours(const cv::Mat &image) {
        cv::findContours(image.clone(), contour_list_point_, hierarchy_, CV_RETR_TREE,
                         CV_CHAIN_APPROX_SIMPLE);

        std::vector<std::vector<cv::Point>> new_contour_list;

        for (size_t i = 0, size = hierarchy_.size(); i < size; i++) {
            if (hasParent(hierarchy_[i]) && !hasChild(hierarchy_[i])) {
                new_contour_list.push_back(contour_list_point_[i]);
            }
        }

        std::swap(contour_list_point_, new_contour_list);
    }

    void ContourList::retrieveOuterContours(const cv::Mat &image) {
        cv::findContours(image.clone(), contour_list_point_, hierarchy_, CV_RETR_TREE,
                         CV_CHAIN_APPROX_SIMPLE);

        std::vector<std::vector<cv::Point>> new_contour_list;

        for (size_t i = 0, size = hierarchy_.size(); i < size; i++) {
            if (!hasParent(hierarchy_[i])) {
                new_contour_list.push_back(contour_list_point_[i]);
            }
        }

        std::swap(contour_list_point_, new_contour_list);
    }

    void ContourList::retrieveNoChildAndParentContours(const cv::Mat &image) {
        cv::findContours(image.clone(), contour_list_point_, hierarchy_, CV_RETR_TREE,
                         CV_CHAIN_APPROX_SIMPLE);

        std::vector<std::vector<cv::Point>> new_contour_list;

        for (size_t i = 0, size = hierarchy_.size(); i < size; i++) {
            if (!hasParent(hierarchy_[i]) && !hasChild(hierarchy_[i])) {
                new_contour_list.push_back(contour_list_point_[i]);
            }
        }

        std::swap(contour_list_point_, new_contour_list);
    }

}  // namespace proc_image_processing
