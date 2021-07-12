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
    std::vector<cv::Mat> GetColorPlanes(cv::Mat image);

    // Camera offset
    void SetCameraOffset(cv::Point &pt, int rows, int cols);

    // Contours getter
    void RetrieveContours(cv::Mat image, contourList_t &contours);

    void RetrieveInnerContours(cv::Mat image, contourList_t &contours);

    void RetrieveAllInnerContours(cv::Mat image, contourList_t &contours);

    void RetrieveOuterContours(cv::Mat image, contourList_t &contours);

    void RetrieveAllContours(cv::Mat image, contourList_t &contours);

    void RetrieveHiearchyContours(cv::Mat image, contourList_t &contours,
                                  hierachy_t &hierarchy);

    void RetrieveOutNoChildContours(cv::Mat image, contourList_t &contours);

    void RetrieveContourRotRect(cv::RotatedRect rect, contour_t &contour);

    Line FitLineOnPolygone(contour_t contour, int cols);

    Line GetPerpendicularLine(Line line, cv::Point2f center);

    void RetrieveContourRotRect(RotRect rect, contour_t &contour);

    // Features calculation
    float CalculateRatio(float width, float height);

    float CalculateConvexityRatio(contour_t contour);

    float CalculateConvexHullArea(contour_t contour);

    float CalculateCircleIndex(float area, float perimeter);

    float CalculateCircleIndex(contour_t contour);

    // Receive a binary input and calculates the number of white pixel over the
    // total number of pixel in the upright rectangle
    float CalculatePourcentFilled(const cv::Mat &image, const cv::Rect &rectangle);

    float CalculatePourcentFilled(const cv::Mat &image,
                                  const cv::RotatedRect &rectangle);

    cv::Scalar CalculateMeans(contour_t contour, cv::Mat image, bool middle = true);

    bool IsRectangle(contour_t &contour, unsigned int degreeAccuracy = 5);

    bool IsSquare(std::vector<cv::Point> &approx, double min_area, double angle,
                  double ratio_min, double ratio_max);

    cv::Mat ExtractImageFromRect(cv::RotatedRect rect, cv::Mat image);

    cv::Mat ExtractImageFromRect(contour_t rect, cv::Mat image);

    // Uses the enum given in type_and_const.h to control the rotation
    cv::Mat RotateImage(cv::Mat in, rotationType rotation, symmetryType symmetry);

    // Inverse a single channel image.
    void InverseImage(const cv::Mat &in, cv::Mat &out);

    // Process PCA
    cv::Point GetEigenPos(std::vector<cv::Point> &pts);

    std::vector<double> GetEigenValues(std::vector<cv::Point> &pts);

    std::vector<cv::Point2d> GetEigenVectors(std::vector<cv::Point> &pts);

    double AngleBetweenThreePoints(cv::Point pt1, cv::Point pt2, cv::Point pt0);

    void DrawSquares(cv::Mat &image,
                     const std::vector<std::vector<cv::Point>> &squares);

    bool CompareYX(const cv::Point &p1, const cv::Point &p2);

    float Median(std::vector<float> values);

    inline bool SortVerticesLength(const std::pair<unsigned int, cv::Vec3f> &a,
                                   const std::pair<unsigned int, cv::Vec3f> &b) {
        return norm(a.second) > norm(b.second);
    }

    inline bool SortVerticesIndex(const std::pair<unsigned int, cv::Vec3f> &a,
                                  const std::pair<unsigned int, cv::Vec3f> &b) {
        return a.first < b.first;
    }

    inline float EucledianPointDistance(const cv::Point2f &pt1,
                                        const cv::Point2f &pt2) {
        return sqrt(pow((pt1.x - pt2.x), 2) + pow((pt1.y - pt2.y), 2));
    }

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_ALGORITHM_GENERAL_FUNCTION_H_
