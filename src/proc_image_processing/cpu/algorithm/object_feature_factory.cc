#include "object_feature_factory.h"

namespace proc_image_processing {

    ObjectFeatureFactory::ObjectFeatureFactory(unsigned int memorySize)
            : frame_memory_(memorySize) {
        using namespace std::placeholders;
        feature_fct_map_.try_emplace(
                ObjectFeatureData::Feature::RATIO,
                std::bind(&ObjectFeatureFactory::ratioFeature, _1));
        feature_fct_map_.try_emplace(
                ObjectFeatureData::Feature::CONVEXITY,
                std::bind(&ObjectFeatureFactory::convexityFeature, _1));
        feature_fct_map_.try_emplace(
                ObjectFeatureData::Feature::PERCENT_FILLED,
                std::bind(&ObjectFeatureFactory::percentFilledFeature, _1));
        feature_fct_map_.try_emplace(
                ObjectFeatureData::Feature::CIRCULARITY,
                std::bind(&ObjectFeatureFactory::circularityFeature, _1));
        feature_fct_map_.try_emplace(
                ObjectFeatureData::Feature::PRESENCE_CONSISTENCY,
                std::bind(&ObjectFeatureFactory::presenceConsistencyFeature, this, _1));
        feature_fct_map_.try_emplace(
                ObjectFeatureData::Feature::HUE_MEAN,
                std::bind(&ObjectFeatureFactory::hueMeanFeature, _1));
        feature_fct_map_.try_emplace(
                ObjectFeatureData::Feature::SAT_MEAN,
                std::bind(&ObjectFeatureFactory::saturationMeanFeature, _1));
        feature_fct_map_.try_emplace(
                ObjectFeatureData::Feature::INTENSITY_MEAN,
                std::bind(&ObjectFeatureFactory::intensityMeanFeature, _1));
        feature_fct_map_.try_emplace(
                ObjectFeatureData::Feature::RED_MEAN,
                std::bind(&ObjectFeatureFactory::redMeanFeature, _1));
        feature_fct_map_.try_emplace(
                ObjectFeatureData::Feature::GREEN_MEAN,
                std::bind(&ObjectFeatureFactory::greenMeanFeature, _1));
        feature_fct_map_.try_emplace(
                ObjectFeatureData::Feature::BLUE_MEAN,
                std::bind(&ObjectFeatureFactory::blueMeanFeature, _1));
        feature_fct_map_.try_emplace(
                ObjectFeatureData::Feature::GRAY_MEAN,
                std::bind(&ObjectFeatureFactory::grayMeanFeature, _1));
    }

    void ObjectFeatureFactory::percentFilledFeature(const ObjectFullData::Ptr &object) {
        if ((object.get() != nullptr) && (object->getPercentFilled() == -1.0f)) {
            float percentFilled = 0.0f;

            cv::Size imageSize = object->getImageSize();
            cv::Mat drawImage(imageSize, CV_8UC3, cv::Scalar::all(0));
            contourList_t contours;
            cv::Point2f pts[4];
            RotRect rrect = object->getRotRect();
            rrect.points(pts);
            contour_t contour(4);
            for (int i = 0; i < 4; i++) {
                contour[i].x = int(pts[i].x);
                contour[i].y = int(pts[i].y);
            }
            contours.push_back(contour);
            contours.push_back(object->getContourCopy().getContour());

            // draw the biggest one (contour by corners)
            // Then draw the contour in black over the biggest one.
            cv::drawContours(drawImage, contours, 0, CV_RGB(255, 255, 255), -1);
            cv::drawContours(drawImage, contours, 1, CV_RGB(0, 0, 0), -1);

            cv::cvtColor(drawImage, drawImage, CV_BGR2GRAY);
            float notCovered = cv::countNonZero(drawImage);
            // safety, should not happen
            float rrectArea = rrect.size.area();
            if (rrectArea != 0) percentFilled = 1.0f - (notCovered / rrectArea);
            object->setPercentFilled(percentFilled);
        }
    }

    float ObjectFeatureFactory::calculatePlaneMean(const ObjectFullData::Ptr &object, int plane) {
        float mean = 0.0f;
        if (object.get() != nullptr) {
            cv::Mat binaryImage(object->getImageSize(), CV_8UC3, cv::Scalar::all(0));
            contourList_t contours;
            contours.push_back(object->getContourCopy().getContour());
            cv::drawContours(binaryImage, contours, -1, CV_RGB(255, 255, 255), -1);
            cv::cvtColor(binaryImage, binaryImage, CV_BGR2GRAY);
            cv::Mat colorbinaryImage;

            cv::bitwise_and(object->getPlanes(plane), cv::Scalar::all(255),
                            colorbinaryImage, binaryImage);
            long unsigned int accumulator = 0, nbPixel = 0;

            int rows = colorbinaryImage.rows, cols = colorbinaryImage.cols;
            for (int y = 0; y < rows; y++) {
                auto *ptr = colorbinaryImage.ptr<uchar>(y);
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
