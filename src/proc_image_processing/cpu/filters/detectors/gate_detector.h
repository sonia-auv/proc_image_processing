// FACTORY_GENERATOR_CLASS_NAME=GateDetector

#ifndef PROC_IMAGE_PROCESSING_GATE_DETECTOR_H
#define PROC_IMAGE_PROCESSING_GATE_DETECTOR_H

#include <proc_image_processing/cpu/algorithm/general_function.h>
#include <proc_image_processing/cpu/algorithm/object_feature_factory.h>
#include <proc_image_processing/cpu/algorithm/object_full_data.h>
#include <proc_image_processing/cpu/algorithm/performance_evaluator.h>
#include <proc_image_processing/cpu/filters/filter.h>
#include <proc_image_processing/cpu/server/target.h>
#include <memory>
#include <ros/ros.h>

namespace proc_image_processing {

    class GateDetector : public Filter {
    public:
        using Ptr = std::shared_ptr<GateDetector>;

        explicit GateDetector(const GlobalParamHandler &globalParams)
                : Filter(globalParams),
                  enable_("Enable", false, &parameters_),
                  debug_contour_("Debug_contour", false, &parameters_),
                  use_convex_hull_("Use_convex_hull", false, &parameters_),
                  offset_y_for_fence_("Offset Y for fence", false, &parameters_),
                  offset_y_for_fence_fraction("Offset Y for fence fraction", 0.3f, 0.0f,
                                              1.0f, &parameters_),
                  check_max_y_("0. Check max y", false, &parameters_),
                  max_y_("0. Maximum y coordinate", 0.0f, 0.0f, 2000.0f, &parameters_),
                  min_area_("1. Min_area : red", 200, 0, 10000, &parameters_),
                  disable_ratio_("2. disable_ratio_check : blue", false, &parameters_),
                  targeted_ratio_("2. Ratio_target", 0.5f, 0.0f, 1.0f, &parameters_),
                  difference_from_target_ratio_("2. Diff_from_ratio_target", 0.10f, 0.0f,
                                                1.0f, &parameters_),
                  min_percent_filled_("3. Min_percent_filled : yellow", 50, 0, 100,
                                      &parameters_),
                  look_for_rectangle_("4.1 Look_for_Rectangle : green", false,
                                      &parameters_),
                  disable_angle_("4.2 disable_angle_check : green", false, &parameters_),
                  targeted_angle_("4.2 angle_target", 0.0f, 0.0f, 90.0f, &parameters_),
                  difference_from_target_angle_("4.2 Diff_from_angle_target", 30.0f, 0.0f,
                                                90.0f, &parameters_),
                  eliminate_same_x_targets_("5. Eliminate_same_x", false, &parameters_),
                  max_x_difference_for_elimination_("5. Min_x_difference", 50.0f, 0.0f,
                                                    1000.0f, &parameters_),
                  vote_most_centered_("Vote_most_centered", false, &parameters_),
                  vote_most_upright_("Vote_most_upright", false, &parameters_),
                  vote_less_difference_from_targeted_ratio_(
                          "Vote_less_diff_from_target_ratio", false, &parameters_),
                  vote_length_("Vote_length", false, &parameters_),
                  vote_higher_("Vote_higher", false, &parameters_),
                  vote_most_horizontal_("Vote most horizontal", false, &parameters_),
                  id_("ID", "buoy", &parameters_),
                  spec_1_("spec1", "red", &parameters_),
                  spec_2_("spec2", "blue", &parameters_),
                  contour_retrieval_("Contour_retrieval", 0, 0, 4, &parameters_,
                                     "0=All, 1=Out, 2=Inner, 3=InnerMost, 4=OutNoChild"),
                  feature_factory_(5) {
            setName("GateDetector");
            // Little goodies for cvs
            // area_rank,length_rank,circularity,convexity,ratio,presence,percent_filled,hueMean,
        }

        ~GateDetector() override = default;

        void apply(cv::Mat &image) override {
            if (enable_()) {
                if (debug_contour_()) {
                    image.copyTo(output_image_);
                    if (output_image_.channels() == 1) {
                        cv::cvtColor(output_image_, output_image_, CV_GRAY2BGR);
                    }
                }

                if (image.channels() != 1) cv::cvtColor(image, image, CV_BGR2GRAY);
                cv::Mat originalImage = global_param_handler_.getOriginalImage();

                contourList_t contours;
                switch (contour_retrieval_()) {
                    case 1:
                        retrieveOuterContours(image, contours);
                        break;
                    case 2:
                        retrieveAllInnerContours(image, contours);
                        break;
                    case 3:
                        retrieveInnerContours(image, contours);
                        break;
                    case 4:
                        retrieveNoChildAndParentContours(image, contours);
                        break;
                    default:
                        retrieveAllContours(image, contours);
                        break;
                }

                ObjectFullData::FullObjectPtrVec objVec;
                for (int i = 0; i < contours.size(); i++) {
                    if (use_convex_hull_()) {
                        cv::convexHull(contours[i], contours[i]);
                    }

                    ObjectFullData::Ptr object =
                            std::make_shared<ObjectFullData>(originalImage, image, contours[i]);

                    if (object.get() == nullptr) {
                        continue;
                    }

                    // AREA
                    if (object->getCenterPoint().y > max_y_() && check_max_y_()) {
                        continue;
                    }

                    if (object->getArea() < min_area_()) {
                        continue;
                    }
                    if (debug_contour_()) {
                        cv::drawContours(output_image_, contours, i, CV_RGB(255, 0, 0), 2);
                    }

                    // RATIO
                    //feature_factory_.computeAllFeature(object);
                    feature_factory_.ratioFeature(object);
                    if (!disable_ratio_() && (fabs(object->getRatio() - targeted_ratio_()) >
                                              fabs(difference_from_target_ratio_()))) {
                        continue;
                    }
                    if (debug_contour_()) {
                        cv::drawContours(output_image_, contours, i, CV_RGB(0, 0, 255), 2);
                    }

                    // PERCENT FILLED
                    ObjectFeatureFactory::percentFilledFeature(object);
                    float percent_filled = getPercentFilled(image, object->getUprightRect());
                    if ((percent_filled) < min_percent_filled_()) {
                        continue;
                    }
                    if (debug_contour_()) {
                        cv::drawContours(output_image_, contours, i, CV_RGB(255, 255, 0), 2);
                    }

                    // ANGLE
                    if (!disable_angle_() &&
                        (fabs(fabs(object->getRotRect().angle) - targeted_angle_()) >
                         fabs(difference_from_target_angle_()))) {
                        continue;
                    }

                    // RECTANGLE
                    if (look_for_rectangle_() && !isRectangle(contours[i], 10)) {
                        // if (look_for_rectangle_() && !isSquare(contours[i], min_area_(),
                        // 80.0f, 0.0f, 100.0f)) {
                        continue;
                    }

                    if (debug_contour_()) {
                        cv::drawContours(output_image_, contours, i, CV_RGB(0, 255, 0), 2);
                    }

                    objVec.push_back(object);
                }

                int num_of_objects = objVec.size();

                if (num_of_objects > 1) {
                    if (vote_most_centered_()) {
                        std::sort(
                                objVec.begin(), objVec.end(),
                                [](const ObjectFullData::Ptr &a, const ObjectFullData::Ptr &b) -> bool {
                                    return getDistanceFromCenter(a) < getDistanceFromCenter(b);
                                });
                        objVec[0]->vote();
                        if (num_of_objects > 2) {
                            objVec[1]->vote();
                        }
                        cv::circle(output_image_, cv::Point(objVec[0]->getCenterPoint().x,
                                                            objVec[0]->getCenterPoint().y) -
                                                  cv::Point(12, 12),
                                   6, CV_RGB(255, 0, 0), -1);
                    }

                    if (vote_length_()) {
                        std::sort(objVec.begin(), objVec.end(),
                                  [](const ObjectFullData::Ptr &a, const ObjectFullData::Ptr &b) -> bool {
                                      return fabs(a->getHeight()) > fabs(b->getHeight());
                                  });
                        objVec[0]->vote();
                        if (num_of_objects > 2) {
                            objVec[1]->vote();
                        }
                        cv::circle(output_image_, cv::Point(objVec[0]->getCenterPoint().x,
                                                            objVec[0]->getCenterPoint().y) -
                                                  cv::Point(12, 6),
                                   6, CV_RGB(0, 0, 255), -1);
                    }

                    if (vote_most_upright_()) {
                        std::sort(objVec.begin(), objVec.end(),
                                  [](const ObjectFullData::Ptr &a, const ObjectFullData::Ptr &b) -> bool {
                                      return fabs(a->getRotRect().angle) <
                                             fabs(b->getRotRect().angle);
                                  });
                        objVec[0]->vote();
                        if (num_of_objects > 2) {
                            objVec[1]->vote();
                        }
                        cv::circle(output_image_, cv::Point(objVec[0]->getCenterPoint().x,
                                                            objVec[0]->getCenterPoint().y) -
                                                  cv::Point(12, 0),
                                   6, CV_RGB(255, 0, 255), -1);
                    }

                    if (vote_most_horizontal_()) {
                        std::sort(objVec.begin(), objVec.end(),
                                  [](const ObjectFullData::Ptr &a, const ObjectFullData::Ptr &b) -> bool {
                                      return fabs(a->getRotRect().angle) >
                                             fabs(b->getRotRect().angle);
                                  });
                        objVec[0]->vote();
                        if (num_of_objects > 2) {
                            objVec[1]->vote();
                        }
                        cv::circle(output_image_, cv::Point(objVec[0]->getCenterPoint().x,
                                                            objVec[0]->getCenterPoint().y) -
                                                  cv::Point(12, 0),
                                   6, CV_RGB(255, 0, 255), -1);
                    }

                    if (vote_less_difference_from_targeted_ratio_()) {
                        std::sort(
                                objVec.begin(), objVec.end(),
                                [this](const ObjectFullData::Ptr &a, const ObjectFullData::Ptr &b) -> bool {
                                    return fabs(a->getRatio() - targeted_ratio_()) <
                                           fabs(b->getRatio() - targeted_ratio_());
                                });
                        objVec[0]->vote();
                        if (num_of_objects > 2) {
                            objVec[1]->vote();
                        }
                        cv::circle(output_image_, cv::Point(objVec[0]->getCenterPoint().x,
                                                            objVec[0]->getCenterPoint().y) -
                                                  cv::Point(12, -6),
                                   6, CV_RGB(0, 255, 255), -1);
                    }

                    if (vote_higher_()) {
                        std::sort(
                                objVec.begin(), objVec.end(),
                                [](const ObjectFullData::Ptr &a, const ObjectFullData::Ptr &b)
                                        -> bool { return a->getCenterPoint().y < b->getCenterPoint().y; });
                        objVec[0]->vote();
                        if (num_of_objects > 2) {
                            objVec[1]->vote();
                        }

                        cv::circle(output_image_, cv::Point(objVec[0]->getCenterPoint().x,
                                                            objVec[0]->getCenterPoint().y) -
                                                  cv::Point(12, -12),
                                   6, CV_RGB(255, 255, 0), -1);
                    }

                    std::sort(objVec.begin(), objVec.end(),
                              [](const ObjectFullData::Ptr &a, const ObjectFullData::Ptr &b)
                                      -> bool { return a->getArea() > b->getArea(); });

                    objVec[0]->vote();
                    if (num_of_objects > 2) {
                        objVec[1]->vote();
                    }
                    cv::circle(output_image_, cv::Point(objVec[0]->getCenterPoint().x,
                                                        objVec[0]->getCenterPoint().y) -
                                              cv::Point(12, -18),
                               6, CV_RGB(255, 150, 150), -1);
                }

                std::sort(objVec.begin(), objVec.end(),
                          [](const ObjectFullData::Ptr &a, const ObjectFullData::Ptr &b)
                                  -> bool { return a->getVoteCount() > b->getVoteCount(); });


                if (eliminate_same_x_targets_() && objVec.size() > 1) {
                    removeSameXTarget(objVec);
                }

                ObjectFullData::FullObjectPtrVec finalists;
                if (!objVec.empty()) {
                    finalists.push_back(objVec[0]);
                    if (objVec.size() > 1) {
                        finalists.push_back(objVec[1]);
                    }
                }

                std::sort(finalists.begin(), finalists.end(),
                          [](const ObjectFullData::Ptr &a, const ObjectFullData::Ptr &b)
                                  -> bool { return a->getCenterPoint().y < b->getCenterPoint().y; });

                if (!finalists.empty()) {
                    Target target;
                    //        ObjectFullData::Ptr object = objVec[0];
                    float x;
                    for (auto &finalist : finalists)
                        x = x + finalist->getCenterPoint().x;
                    x = x / finalists.size();
                    float y;
                    int y_count = 0;
                    for (size_t j = 0; j < 2 && j < finalists.size(); j++) {
                        y = y + finalists[j]->getCenterPoint().y;
                        y_count++;
                    }

                    y = y / y_count;

                    cv::Point center((int) round(x), (int) round(y));
                    target.setTarget(
                            id_(), center.x, center.y, 0, 0, 0, image.rows, image.cols);
                    target.setSpecialField1(spec_1_());
                    target.setSpecialField2(spec_2_());
                    notify(target);
                    if (debug_contour_()) {
                        cv::circle(output_image_,
                                   cv::Point((int) round(x), (int) round(y)),
                                   3, CV_RGB(0, 255, 0), 3);
                    }
                }
                if (debug_contour_()) {
                    output_image_.copyTo(image);
                }
            }
        }

        static float getDistanceFromCenter(const ObjectFullData::Ptr &object) {
            cv::Point center(object->getBinaryImage().cols / 2,
                             object->getBinaryImage().rows / 2);
            float x_diff = object->getCenterPoint().x - center.x;
            float y_diff = object->getCenterPoint().y - center.y;
            return x_diff * x_diff + y_diff * y_diff;
        };

    private:
        cv::Mat output_image_;

        Parameter<bool> enable_, debug_contour_, use_convex_hull_;
        Parameter<bool> offset_y_for_fence_;

        RangedParameter<double> offset_y_for_fence_fraction;

        Parameter<bool> check_max_y_;

        RangedParameter<double> max_y_, min_area_;

        Parameter<bool> disable_ratio_;

        RangedParameter<double> targeted_ratio_, difference_from_target_ratio_, min_percent_filled_;

        Parameter<bool> look_for_rectangle_, disable_angle_;

        RangedParameter<double> targeted_angle_, difference_from_target_angle_;

        Parameter<bool> eliminate_same_x_targets_;

        RangedParameter<double> max_x_difference_for_elimination_;

        Parameter<bool> vote_most_centered_,
                vote_most_upright_,
                vote_less_difference_from_targeted_ratio_,
                vote_length_,
                vote_higher_,
                vote_most_horizontal_;

        Parameter <std::string> id_, spec_1_, spec_2_;

        RangedParameter<int> contour_retrieval_;

        ObjectFeatureFactory feature_factory_;

        bool isSameX(const ObjectFullData::Ptr &a, const ObjectFullData::Ptr &b);

        // check if ref is higher than compared
        static bool isHigher(const ObjectFullData::Ptr &ref, const ObjectFullData::Ptr &compared);

        void removeSameXTarget(ObjectFullData::FullObjectPtrVec &vec);
    };

    inline void GateDetector::removeSameXTarget(
            ObjectFullData::FullObjectPtrVec &vec) {
        std::vector<unsigned int> index_to_eliminate;
        // We should not have much target, so double loop is ok...
        for (unsigned int i = 0, size = vec.size(); i < size; i++) {
            for (unsigned int j = 0; j < size; j++) {
                if (i == j) {
                    continue;
                }
                if (isSameX(vec[i], vec[j])) {
                    // If I is higher, eliminate it
                    if (isHigher(vec[i], vec[j])) {
                        index_to_eliminate.push_back(i);
                    }
                }
            }
        }
        // Erase from vector
        if (!index_to_eliminate.empty()) {
            // Erase same indexes
            std::sort(index_to_eliminate.begin(), index_to_eliminate.end());
            index_to_eliminate.erase(
                    std::unique(index_to_eliminate.begin(), index_to_eliminate.end()),
                    index_to_eliminate.end());
            // Erase the values from the vector.
            for (int i = index_to_eliminate.size() - 1; i >= 0; i--) {
                vec.erase(vec.begin() + i);
            }
        }
    }

    inline bool GateDetector::isSameX(const ObjectFullData::Ptr &a, const ObjectFullData::Ptr &b) {
        double x_difference = static_cast<double>(a->getCenterPoint().x) - static_cast<double>(b->getCenterPoint().x);
        double abs_x_difference = fabs(x_difference);
        return abs_x_difference < max_x_difference_for_elimination_();
    }

    // check if ref is higher than compared
    inline bool GateDetector::isHigher(const ObjectFullData::Ptr &ref, const ObjectFullData::Ptr &compared) {
        return ref->getCenterPoint().y < compared->getCenterPoint().y;
    }

}  // namespace proc_image_processing

#endif //PROC_IMAGE_PROCESSING_GATE_DETECTOR_H
