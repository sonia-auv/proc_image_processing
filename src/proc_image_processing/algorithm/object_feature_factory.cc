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

#include "proc_image_processing/algorithm/object_feature_factory.h"

namespace proc_image_processing {

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
ObjectFeatureFactory::ObjectFeatureFactory(unsigned int memorySize)
    : frame_memory_(memorySize) {
  using namespace std::placeholders;
  feature_fct_map_.emplace(
      ObjectFeatureData::Feature::RATIO,
      std::bind(&ObjectFeatureFactory::RatioFeature, this, _1));
  feature_fct_map_.emplace(
      ObjectFeatureData::Feature::CONVEXITY,
      std::bind(&ObjectFeatureFactory::ConvexityFeature, this, _1));
  feature_fct_map_.emplace(
      ObjectFeatureData::Feature::PERCENT_FILLED,
      std::bind(&ObjectFeatureFactory::PercentFilledFeature, this, _1));
  feature_fct_map_.emplace(
      ObjectFeatureData::Feature::CIRCULARITY,
      std::bind(&ObjectFeatureFactory::CircularityFeature, this, _1));
  feature_fct_map_.emplace(
      ObjectFeatureData::Feature::PRESENCE_CONSISTENCY,
      std::bind(&ObjectFeatureFactory::PresenceConsistencyFeature, this, _1));
  feature_fct_map_.emplace(
      ObjectFeatureData::Feature::HUE_MEAN,
      std::bind(&ObjectFeatureFactory::HueMeanFeature, this, _1));
  feature_fct_map_.emplace(
      ObjectFeatureData::Feature::SAT_MEAN,
      std::bind(&ObjectFeatureFactory::SatMeanFeature, this, _1));
  feature_fct_map_.emplace(
      ObjectFeatureData::Feature::INTENSITY_MEAN,
      std::bind(&ObjectFeatureFactory::IntensityMeanFeature, this, _1));
  feature_fct_map_.emplace(
      ObjectFeatureData::Feature::RED_MEAN,
      std::bind(&ObjectFeatureFactory::RedMeanFeature, this, _1));
  feature_fct_map_.emplace(
      ObjectFeatureData::Feature::GREEN_MEAN,
      std::bind(&ObjectFeatureFactory::GreenMeanFeature, this, _1));
  feature_fct_map_.emplace(
      ObjectFeatureData::Feature::BLUE_MEAN,
      std::bind(&ObjectFeatureFactory::BlueMeanFeature, this, _1));
  feature_fct_map_.emplace(
      ObjectFeatureData::Feature::GRAY_MEAN,
      std::bind(&ObjectFeatureFactory::GrayMeanFeature, this, _1));
}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
void ObjectFeatureFactory::PercentFilledFeature(ObjectFullData::Ptr object) {
  if ((object.get() != nullptr) && (object->GetPercentFilled() == -1.0f)) {
    float percentFilled = 0.0f;

    cv::Size imageSize = object->GetImageSize();
    cv::Mat drawImage(imageSize, CV_8UC3, cv::Scalar::all(0));
    contourList_t contours;
    cv::Point2f pts[4];
    RotRect rrect = object->GetRotatedRect();
    rrect.points(pts);
    contour_t contour(4);
    for (int i = 0; i < 4; i++) {
      contour[i].x = int(pts[i].x);
      contour[i].y = int(pts[i].y);
    }
    contours.push_back(contour);
    contours.push_back(object->GetContourCopy().GetContour());

    // Draw the biggest one (contour by corners)
    // Then draw the contour in black over the biggest one.
    cv::drawContours(drawImage, contours, 0, CV_RGB(255, 255, 255), -1);
    cv::drawContours(drawImage, contours, 1, CV_RGB(0, 0, 0), -1);

    cv::cvtColor(drawImage, drawImage, CV_BGR2GRAY);
    float notCovered = cv::countNonZero(drawImage);
    // safety, should not happen
    float rrectArea = rrect.size.area();
    if (rrectArea != 0) percentFilled = 1.0f - (notCovered / rrectArea);
    object->SetPercentFilled(percentFilled);
  }
}

//------------------------------------------------------------------------------
//
float ObjectFeatureFactory::CalculatePlaneMean(ObjectFullData::Ptr object,
                                               int plane) {
  float mean = 0.0f;
  if (object.get() != nullptr) {
    cv::Mat binaryImage(object->GetImageSize(), CV_8UC3, cv::Scalar::all(0));
    contourList_t contours;
    contours.push_back(object->GetContourCopy().GetContour());
    cv::drawContours(binaryImage, contours, -1, CV_RGB(255, 255, 255), -1);
    cv::cvtColor(binaryImage, binaryImage, CV_BGR2GRAY);
    cv::Mat colorbinaryImage;

    cv::bitwise_and(object->GetPlanes(plane), cv::Scalar::all(255),
                    colorbinaryImage, binaryImage);
    long unsigned int accumulator = 0, nbPixel = 0;

    int rows = colorbinaryImage.rows, cols = colorbinaryImage.cols;
    for (int y = 0; y < rows; y++) {
      uchar *ptr = colorbinaryImage.ptr<uchar>(y);
      for (int x = 0; x < cols; x++) {
        if (ptr[x] != 0) {
          accumulator += ptr[x];
          nbPixel++;
        }
      }
    }
    if (nbPixel != 0) {
      mean = (float(accumulator) / float(nbPixel * 255));
    }
  }
  return mean;
}

}  // namespace proc_image_processing
