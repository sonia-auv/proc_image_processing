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

#ifndef PROVIDER_VISION_FILTERS_PIPE_DETECTOR_H_
#define PROVIDER_VISION_FILTERS_PIPE_DETECTOR_H_

#include <proc_image_processing/algorithm/general_function.h>
#include <proc_image_processing/algorithm/object_full_data.h>
#include <proc_image_processing/algorithm/performance_evaluator.h>
#include <proc_image_processing/algorithm/line.h>
#include <proc_image_processing/filters/filter.h>
#include <proc_image_processing/server/target.h>
#include <memory>

namespace proc_image_processing {

class PipeDetector : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<PipeDetector>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit PipeDetector(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        enable_("Enable", false, &parameters_),
        debug_contour_("Debug_contour", false, &parameters_),
        look_for_rectangle_("Look_for_Rectangle", false, &parameters_),
        min_area_("Min_area", 200, 0, 10000, &parameters_) {
    SetName("PipeDetector");
  }

  virtual ~PipeDetector() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void Execute(cv::Mat &image) {
    if (enable_()) {
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
      for (int i = 0, size = contours.size(); i < size; i++) {
        ObjectFullData::Ptr object =
            std::make_shared<ObjectFullData>(originalImage, image, contours[i]);
        if (object.get() == nullptr) {
          continue;
        }
        //
        // AREA
        //
        if (object->GetArea() < min_area_()) {
          continue;
        }
        if (debug_contour_()) {
          cv::drawContours(output_image_, contours, i, CV_RGB(0, 255, 0), 2);
          cv::Vec4d line;
          int cols =  output_image_.cols;
          //int rows =  output_image_.rows;
          cv::fitLine(contours[i], line, cv::DIST_L2, 0, 0.01 , 0.01);

          int lefty = int((-line[2] * line[1] / line[0]) + line[3]);
          int righty = int(((cols - line[2]) * line[1] / line[0]) + line[3]);

          cv::Point p1;
          cv::Point p2;

          p1.x = cols-1;
          p1.y = righty;
          p2.x = 0;
          p2.y = lefty;

          Line line1(p1, p2);

          cv::Point2f vector = line1.PerpendicularLine();

          p1 = object->GetCenter();
          p2 = vector + object->GetCenter();

          cv::Point2f doubleVector = -(p2 - p1);

          cv::Point start = doubleVector + object->GetCenter();
          cv::Point end   = vector + object->GetCenter();

          Line line2(start, end);

          std::vector<cv::Point> perpendicularLine = line2.GenerateLine(output_image_);

          std::vector<std::tuple<cv::Point, int>> intersectionPoint;

          for (cv::Point &linePoint : perpendicularLine)
          {
            for (int id = 0; id < contours[i].size(); id++)
            {
              if (std::abs(cv::norm(linePoint - contours[i][id])) < 20)
              {
                std::tuple<cv::Point, int> data = std::make_tuple(contours[i][id], id);
                intersectionPoint.push_back(data);
              }
            }
          }
              bool oneTime = false;
              for (std::tuple<cv::Point, int> &pointAndId1 : intersectionPoint)
              {
                  for (std::tuple<cv::Point, int> &pointAndId2 : intersectionPoint) {
                      int id1 = std::get<1>(pointAndId1);
                      int id2 = std::get<1>(pointAndId2);
                      if (std::abs(id2 - id1) >= 10 && !oneTime)
                      {
                          intersectionPoint_.push_back(pointAndId2);
                          intersectionPoint_.push_back(pointAndId1);
                          oneTime = true;
                      }

                  }
              }

          std::vector<int> firstContourId;
          std::vector<int> lastContourId;

          int idMax = std::max(std::get<1>(intersectionPoint_[0]), std::get<1>(intersectionPoint_[1]));
          int idMin = std::min(std::get<1>(intersectionPoint_[0]), std::get<1>(intersectionPoint_[1]));


           for (int idLastContour = idMin; idLastContour <= idMax; idLastContour++)
           {
               lastContourId.push_back(idLastContour);
           }

          int contourSize = (int)contours[i].size();
          while (idMax != idMin)
          {
              if (idMax > (contourSize - 1))
              {
                  idMax = idMax - contourSize;
              } else
              {
                  firstContourId.push_back(idMax);
                  idMax++;
              }
          }


          std::vector<cv::Point> firstContour;
          std::vector<cv::Point> lastContour;

          for (int &id : firstContourId)
          {
              firstContour.push_back(contours[i][id]);
          }

          for (int &id : lastContourId)
          {
              lastContour.push_back(contours[i][id]);
          }

            ObjectFullData::Ptr firstObject =
                    std::make_shared<ObjectFullData>(originalImage, image, firstContour);

            ObjectFullData::Ptr lastObject =
                    std::make_shared<ObjectFullData>(originalImage, image, lastContour);

            cv::circle(output_image_, lastObject->GetCenter(), 3,
                       CV_RGB(255, 0, 0), 3);

            cv::circle(output_image_, firstObject->GetCenter(), 3,
                       CV_RGB(0, 0, 255), 3);



          std::vector<std::vector<cv::Point>> coooooo;
            coooooo.push_back(firstContour);
            cv::drawContours(output_image_, coooooo, 0, CV_RGB(0, 0, 255), 2);

//            for (std::tuple<cv::Point, int> &point : intersectionPoint_)
//            {
//                cv::circle(output_image_, std::get<0>(point), 3,
//                           CV_RGB(255, 0, 0), 3);
//            }


//          std::vector<std::vector<cv::Point>> realLine;
//          realLine.push_back(perpendicularLine);
//
//          line2.Draw(output_image_, CV_RGB(0, 255, 0));
//          cv::drawContours(output_image_, realLine, 0, CV_RGB(255, 0, 0), 2);
//          line1.Draw(output_image_, cv::Scalar(255, 0, 0));

        }

        //
        // RECTANGLE
        //
        if (look_for_rectangle_() && !IsRectangle(contours[i], 10)) {
          continue;
        }

        objVec.push_back(object);
      }

      std::sort(objVec.begin(), objVec.end(),
                [](ObjectFullData::Ptr a, ObjectFullData::Ptr b) -> bool {
                  return a->GetArea() > b->GetArea();
                });

      // Since we search only one buoy, get the biggest from sort function
      if (objVec.size() > 0) {
        Target target;
        ObjectFullData::Ptr object = objVec[0];
        cv::Point center = object->GetCenter();
        target.SetTarget("pipe", center.x, center.y, object->GetLength(),
                         object->GetLength(), object->GetRotatedRect().angle,
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
  }

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  cv::Mat output_image_;

  Parameter<bool> enable_, debug_contour_, look_for_rectangle_;

  std::vector<std::tuple<cv::Point, int>> intersectionPoint_;

  RangedParameter<double> min_area_;
};

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_PIPE_DETECTOR_H_
