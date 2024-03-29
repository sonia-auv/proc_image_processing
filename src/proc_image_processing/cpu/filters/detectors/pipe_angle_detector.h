// FACTORY_GENERATOR_CLASS_NAME=PipeAngleDetector

#ifndef PROC_IMAGE_PROCESSING_FILTERS_PIPE_ANGLE_DETECTOR_H_
#define PROC_IMAGE_PROCESSING_FILTERS_PIPE_ANGLE_DETECTOR_H_

#include <proc_image_processing/cpu/algorithm/general_function.h>
#include <proc_image_processing/cpu/algorithm/object_full_data.h>
#include <proc_image_processing/cpu/algorithm/performance_evaluator.h>
#include <proc_image_processing/cpu/algorithm/line.h>
#include <proc_image_processing/cpu/filters/filter.h>
#include <proc_image_processing/cpu/server/target.h>
#include <memory>

namespace proc_image_processing {

    class PipeAngleDetector : public Filter {
    public:
        using Ptr = std::shared_ptr<PipeAngleDetector>;

        explicit PipeAngleDetector(const GlobalParameterHandler &globalParams)
                : Filter(globalParams),
                  angle_(0.0f),
                  debug_contour_("Debug contour", false, &parameters_),
                  min_area_("Minimum area", 200, 0, 10000, &parameters_),
                  min_pixel_("Minimum pixel", 0, 20, 100, &parameters_) {
            setName("PipeAngleDetector");
        }

        ~PipeAngleDetector() override = default;

        void apply(cv::Mat &image) override {
            intersectionPoint_.clear();
            if (debug_contour_()) {
                image.copyTo(output_image_);
                if (output_image_.channels() == 1) {
                    cv::cvtColor(output_image_, output_image_, CV_GRAY2BGR);
                }
            }

            if (image.channels() != 1) {
                cv::cvtColor(image, image, CV_BGR2GRAY);
            }

            cv::Mat originalImage = global_param_handler_.getOriginalImage();

            PerformanceEvaluator timer;
            timer.resetStartTime();

            contourList_t contours;
            retrieveOuterContours(image, contours);
            ObjectFullData::FullObjectPtrVec objVec;
            ObjectFullData::Ptr firstObject = nullptr;
            ObjectFullData::Ptr lastObject = nullptr;
            for (int i = 0; i < contours.size(); i++) {
                ObjectFullData::Ptr object = std::make_shared<ObjectFullData>(
                        output_image_,
                        image,
                        reinterpret_cast<Contour &&>(contours[i])
                );;

                std::vector<cv::Point> realContour = contours[i];

                if (object.get() == nullptr) {
                    continue;
                }

                // AREA
                if (object->getArea() < min_area_()) {
                    continue;
                }

                Line lineFit = getLineOnPolygon(contours[i], output_image_.cols);

                Line linePer = getPerpendicularLine(lineFit, object->getCenterPoint());

                std::vector<cv::Point> perpendicularLine = linePer.getPoints(output_image_);

                std::vector<std::tuple<cv::Point, int>> intersectionPoint;

                for (cv::Point &linePoint : perpendicularLine) {
                    for (size_t id = 0; id < realContour.size(); id++) {
                        if (std::abs(cv::norm(linePoint - realContour[id])) < min_pixel_()) {
                            std::tuple<cv::Point, int> data = std::make_tuple(realContour[id], id);
                            intersectionPoint.push_back(data);
                        }
                    }
                }

                bool oneTime = false;
                for (std::tuple<cv::Point, int> &pointAndId1 : intersectionPoint) {
                    for (std::tuple<cv::Point, int> &pointAndId2 : intersectionPoint) {
                        int id1 = std::get<1>(pointAndId1);
                        int id2 = std::get<1>(pointAndId2);
                        if (std::abs(id2 - id1) >= (float) realContour.size() / 2 && !oneTime) {
                            intersectionPoint_.push_back(pointAndId2);
                            intersectionPoint_.push_back(pointAndId1);
                            oneTime = true;
                        }
                    }
                }

                std::vector<int> firstContourId;
                std::vector<int> lastContourId;

                if (intersectionPoint_.size() == 2) {
                    int idMax = std::max(std::get<1>(intersectionPoint_[0]), std::get<1>(intersectionPoint_[1]));
                    int idMin = std::min(std::get<1>(intersectionPoint_[0]), std::get<1>(intersectionPoint_[1]));

                    for (int idLastContour = idMin; idLastContour <= idMax; idLastContour++) {
                        lastContourId.push_back(idLastContour);
                    }

                    int contourSize = (int) contours[i].size();
                    while (idMax != idMin) {
                        if (idMax > (contourSize - 1)) {
                            idMax = idMax - contourSize;
                        } else {
                            firstContourId.push_back(idMax);
                            idMax++;
                        }
                    }

                    std::vector<cv::Point> firstContour;
                    std::vector<cv::Point> lastContour;

                    for (int &id : firstContourId) {
                        firstContour.push_back(realContour[id]);
                    }

                    for (int &id : lastContourId) {
                        lastContour.push_back(realContour[id]);
                    }

                    firstObject = std::make_shared<ObjectFullData>(originalImage, image,
                                                                   reinterpret_cast<Contour &&>(firstContour));
                    lastObject = std::make_shared<ObjectFullData>(originalImage, image,
                                                                  reinterpret_cast<Contour &&>(lastContour));
                }

                if (debug_contour_()) {
                    cv::drawContours(output_image_, contours, i, CV_RGB(0, 255, 0), 2);
                }
                objVec.push_back(object);
            }

            std::sort(
                    objVec.begin(),
                    objVec.end(),
                    [](const ObjectFullData::Ptr &a, const ObjectFullData::Ptr &b) -> bool {
                        return a->getArea() > b->getArea();
                    }
            );

            // Since we search only one buoy, get the biggest from sort function
            if (!objVec.empty()) {
                if (firstObject != nullptr && lastObject != nullptr) {
                    angle_ = firstObject->getCenterPoint().y > lastObject->getCenterPoint().y
                             ? lastObject->getRotRect().angle : firstObject->getRotRect().angle;

                    if (debug_contour_()) {
                        if (firstObject->getCenterPoint().y > lastObject->getCenterPoint().y) {
                            cv::circle(output_image_, lastObject->getCenterPoint(), 3, CV_RGB(0, 0, 255), 3);
                        } else {
                            cv::circle(output_image_, firstObject->getCenterPoint(), 3, CV_RGB(0, 0, 255), 3);
                        }
                    }
                }
                Target target;
                ObjectFullData::Ptr object = objVec[0];
                cv::Point center = object->getCenterPoint();
                target.setTarget("pipe",
                                 center.x,
                                 center.y,
                                 object->getWidth(),
                                 object->getHeight(),
                                 angle_,
                                 image.rows,
                                 image.cols
                );
                notify(target);
                if (debug_contour_()) {
                    cv::circle(
                            output_image_,
                            objVec[0]->getCenterPoint(),
                            3,
                            CV_RGB(0, 255, 0),
                            3
                    );
                }
            }
            if (debug_contour_()) {
                output_image_.copyTo(image);
            }
        }

    private:
        Parameter<bool> debug_contour_;
        RangedParameter<double> min_area_;
        RangedParameter<double> min_pixel_;

        cv::Mat output_image_;
        std::vector<std::tuple<cv::Point, int>> intersectionPoint_;
        float angle_;
    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_FILTERS_PIPE_ANGLE_DETECTOR_H_