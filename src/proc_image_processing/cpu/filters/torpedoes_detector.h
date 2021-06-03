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

#ifndef PROVIDER_VISION_FILTERS_TORPEDOES_DETECTOR_H_
#define PROVIDER_VISION_FILTERS_TORPEDOES_DETECTOR_H_

#include <proc_image_processing/cpu/algorithm/general_function.h>
#include <proc_image_processing/cpu/algorithm/object_full_data.h>
#include <proc_image_processing/cpu/algorithm/object_ranker.h>
#include "filter.h"
#include <proc_image_processing/cpu/server/target.h>
#include <memory>
#include <string>

namespace proc_image_processing {

class TorpedoesDetector : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<TorpedoesDetector>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit TorpedoesDetector(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        enable_("Enable", false, &parameters_),
        debug_contour_("Debug_contour", false, &parameters_),
        sensibility_("Sensibility", 0.05, 0.0, 1.0, &parameters_),
        min_area_("Minimum Area", 150, 0, 10000, &parameters_),
        angle_("Angle", 0.75, 0.0, 3.14, &parameters_),
        median_("Median multp.", 2.0, 1.0, 100, &parameters_),
        ratio_max_("PCA Ratio max", 2.5, 1.0, 100, &parameters_),
        ratio_min_("PCA Ratio min", 1, 1.0, 100, &parameters_) {
    SetName("TorpedoesDetector");
  }

  virtual ~TorpedoesDetector() {}

  //============================================================================
  // P U B L I C   M E T H O D S
  //----------------------------------------------------------------------------
  //
  virtual void Execute(cv::Mat &image) {
    if (enable_()) {
      if (debug_contour_()) {
        image.copyTo(output_image_);
        if (output_image_.channels() == 1) {
          cv::cvtColor(output_image_, output_image_, CV_GRAY2BGR);
        }
      }
      if (image.channels() != 1) cv::cvtColor(image, image, CV_BGR2GRAY);

      ObjectFullData::FullObjectPtrVec interior_squares;
      ObjectFullData::Ptr panel;
      std::vector<Target> target_vec;
      GetInnerSquare(image, interior_squares);
      GetBigSquare(image, panel, interior_squares);
      // Founded nothing. We already have a filterchain for the big
      // board. Exit filter.
      if (interior_squares.size() == 0) return;

      // Register size categories separator
      size_t nb_of_element = interior_squares.size();
      float bigger_object_size = interior_squares[0]->GetArea();
      float smaller_object_size =
          interior_squares[nb_of_element - 1]->GetArea();
      cv::Point2f panel_center;

      // If we have the complete board, we use the board to set
      // the identity of the elements
      if (panel.get() != nullptr) {
        panel_center = panel->GetCenter();

        // Approach real near, only one square.
        if (nb_of_element == 1) {
          ObjectFullData::Ptr obj = interior_squares[0];
          Target current_square_target;
          current_square_target.SetTarget(obj, "torpedoes");
          current_square_target.SetSpecField_1("a");
          current_square_target.SetSpecField_2("small");
          target_vec.push_back(current_square_target);
          if (debug_contour_()) {
            cv::circle(output_image_, obj->GetCenter(), 4, CV_RGB(0, 255, 0),
                       4);
            cv::putText(output_image_, "a-small", obj->GetCenter(),
                        cv::FONT_HERSHEY_SIMPLEX, 1 /*fontscale*/,
                        cv::Scalar(255, 0, 0), 3 /*thickness*/, 6 /*lineType*/,
                        false);
          }
          // Approach, select the smallest one.
        } else if (nb_of_element == 2) {
          // Get the smallest one.
          ObjectFullData::Ptr small, big;
          if (interior_squares[0]->GetArea() < interior_squares[1]->GetArea()) {
            small = interior_squares[0];
            big = interior_squares[1];
          } else {
            small = interior_squares[1];
            big = interior_squares[0];
          }
          Target small_target, big_target;
          small_target.SetTarget(small, "torpedoes");
          small_target.SetSpecField_1("a");
          small_target.SetSpecField_2("small");
          target_vec.push_back(small_target);

          big_target.SetTarget(big, "torpedoes");
          big_target.SetSpecField_1("b");
          big_target.SetSpecField_2("big");
          target_vec.push_back(big_target);

          if (debug_contour_()) {
            cv::circle(output_image_, small->GetCenter(), 4, CV_RGB(0, 255, 0),
                       4);
            cv::circle(output_image_, big->GetCenter(), 4, CV_RGB(0, 255, 0),
                       4);

            cv::putText(output_image_, "a-small", small->GetCenter(),
                        cv::FONT_HERSHEY_SIMPLEX, 1 /*fontscale*/,
                        cv::Scalar(255, 0, 0), 3 /*thickness*/, 6 /*lineType*/,
                        false);
            cv::putText(output_image_, "b-big", big->GetCenter(),
                        cv::FONT_HERSHEY_SIMPLEX, 1 /*fontscale*/,
                        cv::Scalar(255, 0, 0), 3 /*thickness*/, 6 /*lineType*/,
                        false);
          }
        } else {  // See everything, the world is beautiful
          for (auto &inner_square : interior_squares) {
            cv::Point2f square_center = inner_square->GetCenter();
            // A | C
            //-------
            // B | D
            Target current_square_target;
            current_square_target.SetTarget(inner_square, "torpedoes");

            // Big square
            float area_dist_from_bigger =
                fabs(inner_square->GetArea() - bigger_object_size);
            float area_dist_from_smaller =
                fabs(inner_square->GetArea() - smaller_object_size);

            // the < 1 comparaison is to check if we are looking at the bigger
            // or the smaller object right now. If so, set their values
            // if not check it is near wich one.
            if (area_dist_from_bigger < 1) {
              current_square_target.SetSpecField_2("big");
            } else if (area_dist_from_smaller < 1) {
              current_square_target.SetSpecField_2("small");
            } else if (area_dist_from_bigger < area_dist_from_smaller) {
              current_square_target.SetSpecField_2("big");
            } else  // Small square
            {
              current_square_target.SetSpecField_2("small");
            }
            // LEFT
            if (square_center.x < panel_center.x) {
              // Y is higher, upside of the origin
              // 0 in y is at the top, so if y is smaller, for us, the object is
              // higher in the image.

              // UP
              if (square_center.y < panel_center.y) {
                current_square_target.SetSpecField_1("d");
              }  // DOWN
              else {
                current_square_target.SetSpecField_1("c");
              }
            } else  // RIGHT
            {
              // Y is higher, upside of the origin
              // 0 in y is at the top, so if y is smaller, for us, the object is
              // higher in the image.

              // UP
              if (square_center.y < panel_center.y) {
                current_square_target.SetSpecField_1("b");
              }  // DOWN
              else {
                current_square_target.SetSpecField_1("a");
              }
            }
            target_vec.push_back(current_square_target);
            if (debug_contour_()) {
              cv::circle(output_image_, square_center, 4, CV_RGB(0, 255, 0), 4);
              std::stringstream ss;
              ss << current_square_target.GetSpecField_1() << "-"
                 << current_square_target.GetSpecField_2();
              cv::putText(output_image_, ss.str(), square_center,
                          cv::FONT_HERSHEY_SIMPLEX, 1 /*fontscale*/,
                          cv::Scalar(255, 0, 0), 3 /*thickness*/,
                          6 /*lineType*/, false);
            }
          }
        }
      }

      for (auto &target : target_vec) {
        NotifyTarget(target);
      }

      if (debug_contour_()) {
        cv::cvtColor(image, image, CV_GRAY2BGR);
        output_image_.copyTo(image);
      }
    }
  }
  //---------------------------------------------------------------------------
  //
  void GetInnerSquare(const cv::Mat &in,
                      ObjectFullData::FullObjectPtrVec &object_data) {
    ContourList inner_contours(in, ContourList::INNER_MOST);
    cv::Mat original = global_params_.getOriginalImage();

    for (auto &contour : inner_contours.contour_vec_) {
      contour.ApproximateBySize();
      Contour::ContourVec ctr = contour.GetContour();
      if (IsSquare(ctr, min_area_(), angle_(), ratio_min_(), ratio_max_())) {
        object_data.push_back(
            std::make_shared<ObjectFullData>(original, in, ctr));
      }
    }

    std::sort(object_data.begin(), object_data.end(),
              [](ObjectFullData::Ptr a, ObjectFullData::Ptr b) -> bool {
                return a->GetArea() > b->GetArea();
              });

    if (object_data.size() > 4) {
      object_data.erase(object_data.begin() + 4, object_data.end());
    }

    if (debug_contour_()) {
      for (auto &object : object_data) {
        cv::polylines(output_image_, object->GetContourCopy().GetContour(),
                      true, CV_RGB(255, 0, 0), 3);
      }

    }  // End if was a square
  }    // End iteration through contours.

  //----------------------------------------------------------------------------
  //
  void GetBigSquare(const cv::Mat &in, ObjectFullData::Ptr &big_square,
                    const ObjectFullData::FullObjectPtrVec &inside_squares) {
    ContourList contours(in, ContourList::OUTER);

    std::vector<std::pair<ObjectFullData::Ptr, int>> contour_vote;

    cv::Mat original(global_params_.getOriginalImage());
    for (const auto &contour : contours.GetAsPoint()) {
      contour_vote.push_back(std::pair<ObjectFullData::Ptr, int>(
          std::make_shared<ObjectFullData>(original, in, contour), 0));
    }

    // Votes for the contour with the most
    // for each contours found, count how many inner square there is inside.
    for (auto &outer_square : contour_vote) {
      for (auto inner_square : inside_squares) {
        if (cv::pointPolygonTest(
                outer_square.first->GetContourCopy().GetContour(),
                inner_square->GetCenter(), false) > 0.0f) {
          outer_square.second += 1;
        }
      }
    }

    std::sort(contour_vote.begin(), contour_vote.end(),
              [](const std::pair<ObjectFullData::Ptr, int> &a,
                 const std::pair<ObjectFullData::Ptr, int> &b) -> bool {
                return a.second > b.second;
              });

    if (contour_vote.size() != 0) {
      big_square = contour_vote[0].first;
      if (debug_contour_()) {
        cv::polylines(output_image_, big_square->GetContourCopy().GetContour(),
                      true, CV_RGB(255, 0, 255), 3);
      }
    }
  }

  //----------------------------------------------------------------------------
  //
  void EliminateDoubleObjects(ObjectFullData::FullObjectPtrVec &object_data) {
    // Check if centers are near each other and delete if so
    // TODO: Might be dangerous to use erase...
    // for (size_t r = 1; r < vcenter.size(); r++) {
    //  if (abs(vcenter[r].x - vcenter[r - 1].x) < 5
    //      || abs(vcenter[r].y - vcenter[r - 1].y) < 5) {
    //    vcenter.erase(vcenter.begin() + r);
    //    squares.erase(squares.begin() + r);
    //  }
    //}
  }

  //----------------------------------------------------------------------------
  //
  void EliminateNonFittingObjectsWithSize(
      ObjectFullData::FullObjectPtrVec &object_data) {
    //      // Check if square area is bigger than _median times median of areas
    // of all squares.
    //      std::vector<int> areas(squares.size());
    //      for (size_t i = 0; i < squares.size(); i++) {
    //        areas[i] = (int) std::fabs(cv::contourArea(cv::Mat(squares[i])));
    //      }
    //
    //      if (areas.size() > 0) {
    //        size_t n = areas.size() / 2;
    //        std::nth_element(areas.begin(), areas.begin() + n, areas.end());
    //
    //
    //        double median = areas[n];
    //        // TODO: Might be dangerous to use erase...
    //        for (size_t i = 0; i < squares.size(); i++) {
    //          if (areas[i] > _median * median) {
    //            squares.erase(squares.begin() + i);
    //            vcenter.erase(vcenter.begin() + i);
    //            areas.erase(areas.begin() + i);
    //            i--;
    //          }
    //        }
    //      }
  }

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  Parameter<bool> enable_, debug_contour_;
  RangedParameter<double> sensibility_;
  RangedParameter<double> min_area_;
  RangedParameter<double> angle_;
  RangedParameter<double> median_;
  RangedParameter<double> ratio_max_;
  RangedParameter<double> ratio_min_;
  cv::Mat output_image_;
};

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_FILTERS_TORPEDOES_DETECTOR_H_
