/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
/// \section LICENSE
/// This file is part of S.O.N.I.A. software.
///
/// S.O.N.I.A. software is free software: you can redistribute it and/or modify
/// it under the terms of the GNU General Public License as published by
/// the Free Software Foundation, either version 3 of the License, or
/// (at your option) any later version.
///
/// S.O.N.I.A. software is distributed in the hope that it will be useful,
/// but WITHOUT ANY WARRANTY; without even the implied warranty of
/// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
/// GNU General Public License for more details.
///
/// You should have received a copy of the GNU General Public License
/// along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.

#ifndef PROVIDER_VISION_FILTERS_PIPE_ANGLE_DETECTOR_H_
#define PROVIDER_VISION_FILTERS_PIPE_ANGLE_DETECTOR_H_

#include <proc_image_processing/algorithm/general_function.h>
#include <proc_image_processing/algorithm/object_full_data.h>
#include <proc_image_processing/algorithm/performance_evaluator.h>
#include <proc_image_processing/algorithm/line.h>
#include <proc_image_processing/filters/filter.h>
#include <proc_image_processing/server/target.h>
#include <memory>

namespace proc_image_processing {

    class PipeAngleDetector : public Filter {
    public:
        using Ptr = std::shared_ptr<PipeAngleDetector>;

        explicit PipeAngleDetector(const GlobalParamHandler& globalParams)
            : Filter(globalParams),
            angle_(0.0f),
            debug_contour_("Debug_contour", false, &parameters_),
            min_area_("Min_area", 200, 0, 10000, &parameters_),
            min_pixel_("Min_pixel", 0, 20, 100, &parameters_) {
            SetName("PipeAngleDetector");
        }

        virtual ~PipeAngleDetector() {}

        virtual void ApplyFilter(cv::Mat& image) {
                intersectionPoint_.clear();
                if (debug_contour_()) {
                    image.copyTo(output_image_);
                    if (output_image_.channels() == 1) {
                        cv::cvtColor(output_image_, output_image_, CV_GRAY2BGR);
                    }
                }

                if (image.channels() != 1) cv::cvtColor(image, image, CV_BGR2GRAY);
                cv::Mat originalImage = global_params_.getOriginalImage();

                PerformanceEvaluator timer;
                timer.UpdateStartTime();

                contourList_t contours;
                RetrieveOuterContours(image, contours);
                ObjectFullData::FullObjectPtrVec objVec;
                ObjectFullData::Ptr firstObject = nullptr;
                ObjectFullData::Ptr lastObject = nullptr;
                for (int i = 0, size = contours.size(); i < size; i++) {
                    ObjectFullData::Ptr object =
                        std::make_shared<ObjectFullData>(originalImage, image, contours[i]);

                    std::vector<cv::Point> realContour = contours[i];

                    if (object.get() == nullptr) {
                        continue;
                    }

                    // AREA
                    if (object->GetArea() < min_area_()) {
                        continue;
                    }

                    Line lineFit = FitLineOnPolygone(contours[i], output_image_.cols);

                    Line linePer = GetPerpendicularLine(lineFit, object->GetCenter());

                    std::vector<cv::Point> perpendicularLine = linePer.GenerateLine(output_image_);

                    std::vector<std::tuple<cv::Point, int>> intersectionPoint;

                    for (cv::Point& linePoint : perpendicularLine) {
                        for (size_t id = 0; id < realContour.size(); id++) {
                            if (std::abs(cv::norm(linePoint - realContour[id])) < min_pixel_()) {
                                std::tuple<cv::Point, int> data = std::make_tuple(realContour[id], id);
                                intersectionPoint.push_back(data);
                            }
                        }
                    }

                    bool oneTime = false;
                    for (std::tuple<cv::Point, int>& pointAndId1 : intersectionPoint) {
                        for (std::tuple<cv::Point, int>& pointAndId2 : intersectionPoint) {
                            int id1 = std::get<1>(pointAndId1);
                            int id2 = std::get<1>(pointAndId2);
                            if (std::abs(id2 - id1) >= (float)realContour.size() / 2 && !oneTime) {
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

                        int contourSize = (int)contours[i].size();
                        while (idMax != idMin) {
                            if (idMax > (contourSize - 1)) {
                                idMax = idMax - contourSize;
                            }
                            else {
                                firstContourId.push_back(idMax);
                                idMax++;
                            }
                        }

                        std::vector<cv::Point> firstContour;
                        std::vector<cv::Point> lastContour;

                        for (int& id : firstContourId) {
                            firstContour.push_back(realContour[id]);
                        }

                        for (int& id : lastContourId) {
                            lastContour.push_back(realContour[id]);
                        }

                        firstObject = std::make_shared<ObjectFullData>(originalImage, image, firstContour);
                        lastObject = std::make_shared<ObjectFullData>(originalImage, image, lastContour);
                    }

                    if (debug_contour_()) {
                        cv::drawContours(output_image_, contours, i, CV_RGB(0, 255, 0), 2);
                    }
                    objVec.push_back(object);
                }

                std::sort(objVec.begin(), objVec.end(),
                    [](ObjectFullData::Ptr a, ObjectFullData::Ptr b) -> bool {
                        return a->GetArea() > b->GetArea();
                    });

                // Since we search only one buoy, get the biggest from sort function
                if (objVec.size() > 0) {
                    if (firstObject != nullptr && lastObject != nullptr) {
                        angle_ = firstObject->GetCenter().y > lastObject->GetCenter().y ? lastObject->GetRotatedRect().angle : firstObject->GetRotatedRect().angle;

                        if (debug_contour_()) {
                            if (firstObject->GetCenter().y > lastObject->GetCenter().y) {
                                cv::circle(output_image_, lastObject->GetCenter(), 3, CV_RGB(0, 0, 255), 3);

                            }
                            else {
                                cv::circle(output_image_, firstObject->GetCenter(), 3, CV_RGB(0, 0, 255), 3);
                            }
                        }
                    }
                    Target target;
                    ObjectFullData::Ptr object = objVec[0];
                    cv::Point center = object->GetCenter();
                    target.SetTarget("pipe", center.x, center.y, object->GetWidth(),
                        object->GetHeight(), angle_,
                        image.rows, image.cols);
                    NotifyTarget(target);
                    if (debug_contour_()) {
                        cv::circle(output_image_, objVec[0]->GetCenter(), 3,
                            CV_RGB(0, 255, 0), 3);

                    }
                }
                if (debug_contour_()) {
                    output_image_.copyTo(image);
                }
        }

    private:
        cv::Mat output_image_;

        float angle_;

        Parameter<bool> debug_contour_;

        std::vector<std::tuple<cv::Point, int>> intersectionPoint_;

        RangedParameter<double> min_area_, min_pixel_;
    };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_PIPE_ANGLE_DETECTOR_H_