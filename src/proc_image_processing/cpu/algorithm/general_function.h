/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROVIDER_VISION_ALGORITHM_GENERAL_FUNCTION_H_
#define PROVIDER_VISION_ALGORITHM_GENERAL_FUNCTION_H_

#include "rot_rect.h"
#include "type_and_const.h"
#include "line.h"
#include <memory>
#include <opencv2/opencv.hpp>

namespace proc_image_processing {

#define ACCURACY_TABLE_SIZE 21
    // Degree accuracy are the values corresponding to norm(A X B)/(norm(A) *
    // norm(B))
    // for angles of 90 +- index_in_array.
    // Exemple:
    // DEGREE_ACCURACY_FOR_90[5] is the result of norm(A X B)/(norm(A) * norm(B))
    // when the angle between A and B is 90+t or 90-5
    const float ACCURACY_TABLE[ACCURACY_TABLE_SIZE] = {
            1.0f, 0.999848, 0.999391, 0.99863, 0.997564, 0.996195, 0.994522,
            0.992546, 0.990268, 0.987688, 0.984808, 0.981627, 0.978148, 0.97437,
            0.970296, 0.965926, 0.961262, 0.956305, 0.951057, 0.945519, 0.939693};

    // General function not associate to any object, that can be use by them
    // or in the filters.

    // Image splitting
    // Return vector with blue,green,red,hue,saturation,intensity,gray (in order)
    std::vector<cv::Mat> getColorPlanes(cv::Mat image);

    // Camera offset
    void setCameraOffset(cv::Point &pt, int rows, int cols);

    // Contours getter
    void retrieveContours(cv::Mat image, contourList_t &contours);

    void retrieveInnerContours(cv::Mat image, contourList_t &contours);

    void retrieveAllInnerContours(cv::Mat image, contourList_t &contours);

    void retrieveOuterContours(cv::Mat image, contourList_t &contours);

    void retrieveAllContours(cv::Mat image, contourList_t &contours);

    void retrieveHierarchyContours(cv::Mat image, contourList_t &contours, hierachy_t &hierarchy);

    void retrieveNoChildAndParentContours(cv::Mat image, contourList_t &contours);

    void retrieveContourRotRect(cv::RotatedRect rect, contour_t &contour);

    Line getLineOnPolygon(contour_t contour, int cols);

    Line getPerpendicularLine(Line line, cv::Point2f center);

    void retrieveContourRotRect(RotRect rect, contour_t &contour);

    // Features calculation
    float getResolutionRatio(float width, float height);

    float getConvexityRatio(contour_t contour);

    float getConvexHullArea(contour_t contour);

    float getCircleIndex(float area, float perimeter);

    float getCircleIndex(contour_t contour);

    // Receive a binary input and calculates the number of white pixel over the
    // total number of pixel in the upright rectangle
    float getPercentFilled(const cv::Mat &image, const cv::Rect &rectangle);

    float getPercentFilled(const cv::Mat &image, const cv::RotatedRect &rectangle);

    cv::Scalar getMeans(contour_t contour, cv::Mat image, bool middle = true);

    bool isRectangle(contour_t &contour, unsigned int accuracy = 5);

    bool isSquare(std::vector<cv::Point> &approx, double min_area, double angle, double ratio_min, double ratio_max);

    cv::Mat getImageFromRotatedRect(cv::RotatedRect rect, cv::Mat image);

    cv::Mat getImageFromContour(contour_t rect, cv::Mat image);

    // Uses the enum given in type_and_const.h to control the rotation
    cv::Mat rotateImage(cv::Mat in, rotationType rotation, symmetryType symmetry);

    // Inverse a single channel image.
    void inverseImage(const cv::Mat &in, cv::Mat &out);

    // Process PCA
    cv::Point getEigenPosition(std::vector<cv::Point> &pts);

    std::vector<double> getEigenValues(std::vector<cv::Point> &pts);

    std::vector<cv::Point2d> getEigenVectors(std::vector<cv::Point> &pts);

    double getAngleBetweenPoints(cv::Point pt1, cv::Point pt2, cv::Point pt0);

    void drawSquares(cv::Mat &image, const std::vector<std::vector<cv::Point>> &squares);

    bool compareCoordinates(const cv::Point &p1, const cv::Point &p2);

    float getMedian(std::vector<float> values);

    inline bool compareVerticesLength(const std::pair<unsigned int, cv::Vec3f> &a,
                                      const std::pair<unsigned int, cv::Vec3f> &b) {
        return norm(a.second) > norm(b.second);
    }

    inline bool compareVerticesIndex(const std::pair<unsigned int, cv::Vec3f> &a,
                                     const std::pair<unsigned int, cv::Vec3f> &b) {
        return a.first < b.first;
    }

    inline float getEuclideanDistance(const cv::Point2f &pt1, const cv::Point2f &pt2) {
        return sqrt(pow((pt1.x - pt2.x), 2) + pow((pt1.y - pt2.y), 2));
    }

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_ALGORITHM_GENERAL_FUNCTION_H_
