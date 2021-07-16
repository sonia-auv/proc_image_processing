/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROC_IMAGE_PROCESSING_ALGORITHM_GENERAL_FUNCTION_H_
#define PROC_IMAGE_PROCESSING_ALGORITHM_GENERAL_FUNCTION_H_

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
    // when the getAngle between A and B is 90+t or 90-5
    const float ACCURACY_TABLE[ACCURACY_TABLE_SIZE] = {
            1.0f, 0.999848, 0.999391, 0.99863, 0.997564, 0.996195, 0.994522,
            0.992546, 0.990268, 0.987688, 0.984808, 0.981627, 0.978148, 0.97437,
            0.970296, 0.965926, 0.961262, 0.956305, 0.951057, 0.945519, 0.939693};

    // General function not associate to any object, that can be use by them
    // or in the filters.

    // Image splitting
    // Return vector with blue,green,red,hue,saturation,intensity,gray (in order)
    std::vector<cv::Mat> getColorPlanes(const cv::Mat &image);

    // Camera offset
    void setCameraOffset(cv::Point &pt, int rows, int cols);

    // Contours getter
    void retrieveContours(const cv::Mat &image, contourList_t &contours);

    void retrieveInnerContours(const cv::Mat &image, contourList_t &contours);

    void retrieveAllInnerContours(const cv::Mat &image, contourList_t &contours);

    void retrieveOuterContours(const cv::Mat &image, contourList_t &contours);

    void retrieveAllContours(const cv::Mat &image, contourList_t &contours);

    void retrieveHierarchyContours(const cv::Mat &image, contourList_t &contours, hierarchy_t &hierarchy);

    void retrieveNoChildAndParentContours(const cv::Mat &image, contourList_t &contours);

    void retrieveContourRotRect(const cv::RotatedRect &rect, contour_t &contour);

    Line getLineOnPolygon(const contour_t &contour, int cols);

    Line getPerpendicularLine(Line line, const cv::Point2f &center);

    void retrieveContourRotRect(RotRect rect, contour_t &contour);

    // Features calculation
    float getResolutionRatio(float width, float height);

    float getConvexityRatio(const contour_t &contour);

    float getConvexHullArea(const contour_t &contour);

    float getCircleIndex(float area, float perimeter);

    float getCircleIndex(const contour_t &contour);

    // Receive a binary input and calculates the number of white pixel over the
    // total number of pixel in the upright rectangle
    float getPercentFilled(const cv::Mat &image, const cv::Rect &rectangle);

    float getPercentFilled(const cv::Mat &image, const cv::RotatedRect &rectangle);

    cv::Scalar getMeans(const contour_t &contour, const cv::Mat &image, bool middle = true);

    bool isRectangle(contour_t &contour, unsigned int accuracy = 5);

    bool
    isSquare(std::vector<cv::Point> &approx, double min_area, double angle, double ratio_min, double ratio_max);

    cv::Mat getImageFromRotatedRect(const cv::RotatedRect &rect, const cv::Mat &image);

    cv::Mat getImageFromContour(const contour_t &rect, const cv::Mat &image);

    // Uses the enum given in type_and_const.h to control the rotation
    cv::Mat rotateImage(const cv::Mat &in, rotationType rotation, symmetryType symmetry);

    // Inverse a single channel image.
    void inverseImage(const cv::Mat &in, cv::Mat &out);

    void pcaAnalysis(std::vector<cv::Point> &pts, cv::PCA &pca);

    // Process PCA
    cv::Point getEigenPosition(std::vector<cv::Point> &pts);

    std::vector<double> getEigenValues(std::vector<cv::Point> &pts);

    std::vector<cv::Point2d> getEigenVectors(std::vector<cv::Point> &pts);

    double getAngleBetweenPoints(const cv::Point &pt1, const cv::Point &pt2, const cv::Point &pt0);

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

    inline double getEuclideanDistance(const cv::Point2f &pt1, const cv::Point2f &pt2) {
        return sqrt(pow((pt1.x - pt2.x), 2) + pow((pt1.y - pt2.y), 2));
    }

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_ALGORITHM_GENERAL_FUNCTION_H_
