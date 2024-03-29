#include "general_function.h"

namespace proc_image_processing {

    std::vector<cv::Mat> getColorPlanes(const cv::Mat &image) {
        cv::Mat gray, hsi;
        std::vector<cv::Mat> planes(7);

        cv::cvtColor(image, gray, CV_BGR2GRAY);
        cv::cvtColor(image, hsi, CV_BGR2HSV);

        // Set to zeros
        for (int i = 0; i < 7; i++)
            planes[i] = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);

        cv::split(image, &planes[0]);
        cv::split(hsi, &planes[3]);
        gray.copyTo(planes[6]);
        return planes;
    }

    void setCameraOffset(cv::Point &pt, int rows, int cols) {
        pt.x = pt.x - (cols / 2);
        pt.y = -(pt.y - (rows / 2));
    }

    void retrieveContours(const cv::Mat &image, contourList_t &contours) {
        retrieveOuterContours(image, contours);
    }

    void retrieveInnerContours(const cv::Mat &image, contourList_t &contours) {
        if (image.channels() != 1) return;

        contourList_t contourVect;
        hierarchy_t hierarchy;
        // CLone beacause find contour modifies the image.
        cv::Mat temp = image.clone();
        cv::findContours(temp, contourVect, hierarchy, CV_RETR_TREE,
                         CV_CHAIN_APPROX_SIMPLE);

        for (int i = 0; i < (int) hierarchy.size(); i++) {
            // No child means no contour inside that contours.
            // 1 parent mean that it is inside a contour.
            if (hierarchy[i][FIRST_CHILD_CTR] == -1 && hierarchy[i][PARENT_CTR] != -1) {
                std::vector<cv::Point> tempContour;
                cv::approxPolyDP(contourVect[i], tempContour, 3, true);
                contours.push_back(tempContour);
            }
        }
    }

    void retrieveAllInnerContours(const cv::Mat &image, contourList_t &contours) {
        if (image.channels() != 1) return;

        contourList_t contourVect;
        hierarchy_t hierarchy;
        // CLone beacause find contour modifies the image.
        cv::Mat temp = image.clone();
        cv::findContours(temp, contourVect, hierarchy, CV_RETR_TREE,
                         CV_CHAIN_APPROX_SIMPLE);

        for (int i = 0; i < (int) hierarchy.size(); i++) {
            // No child means no contour inside that contours.
            // 1 parent mean that it is inside a contour.
            if (hierarchy[i][PARENT_CTR] != -1) {
                std::vector<cv::Point> tempContour;
                cv::approxPolyDP(contourVect[i], tempContour, 3, true);
                contours.push_back(tempContour);
            }
        }
    }

    void retrieveOuterContours(const cv::Mat &image, contourList_t &contours) {
        if (image.channels() != 1) return;

        contourList_t contourVect;
        // CLone beacause find contour modifies the image.
        cv::Mat temp = image.clone();
        cv::findContours(temp, contourVect, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        for (auto &i : contourVect) {
            // TO CHECK : New tempContour each time?
            std::vector<cv::Point> tempContour;
            // Three is normal value... Purely hardcoded from xp.
            cv::approxPolyDP(i, tempContour, 3, true);
            contours.push_back(tempContour);
        }
    }

    void retrieveNoChildAndParentContours(const cv::Mat &image, contourList_t &contours) {
        if (image.channels() != 1) return;

        contourList_t contourVect;
        hierarchy_t hierachy;
        // CLone beacause find contour modifies the image.
        cv::Mat temp = image.clone();
        retrieveHierarchyContours(image, contourVect, hierachy);
        for (int i = 0; i < (int) contourVect.size(); i++) {
            // TO CHECK : New tempContour each time?
            // If no child and no parent, good
            if (hierachy[i][FIRST_CHILD_CTR] == -1 && hierachy[i][PARENT_CTR] == -1) {
                // Three is normal value... Purely hardcoded from xp.
                contour_t tempContour;
                cv::approxPolyDP(contourVect[i], tempContour, 3, true);
                contours.push_back(tempContour);
            }
        }
    }

    void retrieveAllContours(const cv::Mat &image, contourList_t &contours) {
        if (image.channels() != 1) return;
        contourList_t contourVect;

        // CLone beacause find contour modifies the image.
        cv::Mat temp = image.clone();
        cv::findContours(temp, contourVect, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
        for (auto &i : contourVect) {
            // TO CHECK : New tempContour each time?
            std::vector<cv::Point> tempContour;
            // Three is normal value... Purely hardcoded from xp.
            cv::approxPolyDP(i, tempContour, 3, true);
            contours.push_back(tempContour);
        }
    }

    void retrieveHierarchyContours(const cv::Mat &image, contourList_t &contours, hierarchy_t &hierarchy) {
        if (image.channels() != 1) return;

        // CLone beacause find contour modifies the image.
        cv::Mat temp = image.clone();
        cv::findContours(temp, contours, hierarchy, CV_RETR_TREE,
                         CV_CHAIN_APPROX_SIMPLE);

        for (int i = 0; i < (int) hierarchy.size(); i++) {
            cv::approxPolyDP(contours[i], contours[i], 3, true);
        }
    }

    void retrieveContourRotRect(const cv::RotatedRect &rect, contour_t &contour) {
        cv::Point2f pts[4];
        rect.points(pts);

        for (auto &pt : pts) {
            contour.push_back(pt);
        }
    }

    Line getLineOnPolygon(const contour_t &contour, int cols) {
        cv::Vec4d line;
        cv::fitLine(contour, line, cv::DIST_L2, 0, 0.01, 0.01);

        int lefty = int((-line[2] * line[1] / line[0]) + line[3]);
        int righty = int(((cols - line[2]) * line[1] / line[0]) + line[3]);

        cv::Point p1;
        cv::Point p2;

        p1.x = cols - 1;
        p1.y = righty;
        p2.x = 0;
        p2.y = lefty;

        Line lineFit(p1, p2);

        return lineFit;
    }

    Line getPerpendicularLine(Line line, const cv::Point2f &center) {
        cv::Point p1;
        cv::Point p2;

        cv::Point2f vector = line.getPerpendicularLine();

        p1 = center;
        p2 = vector + center;

        cv::Point2f doubleVector = -(p2 - p1);

        cv::Point start = doubleVector + center;
        cv::Point end = vector + center;

        Line perpendicularLine(start, end);
        return perpendicularLine;

    }

    float getResolutionRatio(float width, float height) {
        if (width == 0 && height == 0) return 0;
        return std::min((height / width), (width / height)) * 100;
    }

    double getConvexityRatio(const contour_t &contour) {
        if (contour.size() <= 2) return -1;
        auto convexHullArea = getConvexHullArea(contour);
        if (convexHullArea == 0) return 0;
        return (cv::contourArea(contour) / convexHullArea) * 100;
    }

    double getConvexHullArea(const contour_t &contour) {
        if (contour.size() <= 2) return -1;
        contour_t convexHull;
        cv::convexHull(contour, convexHull, false, true);
        return cv::contourArea(convexHull, false);
    }

    double getCircleIndex(double area, double perimeter) {
        auto circumference = perimeter / (2 * M_PI);
        auto radiusArea = sqrt(area / (M_PI));
        if (circumference == 0 && radiusArea == 0) return 0;
        return circumference > radiusArea ? radiusArea / circumference : circumference / radiusArea;
    }

    double getCircleIndex(const contour_t &contour) {
        return getCircleIndex(cv::contourArea(contour, false),
                              cv::arcLength(contour, true)
        );
    }

    cv::Scalar getMeans(const contour_t &contour, const cv::Mat &image, bool middle) {
        cv::Mat opImage;
        if (image.channels() > 1)
            cv::cvtColor(image, opImage, CV_BGR2GRAY);
        else
            image.copyTo(opImage);
        cv::Mat matRoi;
        cv::Rect boundRect = cv::boundingRect(cv::Mat(contour));
        if (middle) {
            cv::Rect roi(
                    boundRect.x + 0.25 * boundRect.width,
                    boundRect.y + 0.25 * boundRect.height,
                    0.5 * boundRect.width,
                    0.5 * boundRect.height
            );

            matRoi = cv::Mat(opImage, roi);
        } else {
            matRoi = cv::Mat(opImage, boundRect);
        }
        return cv::mean(matRoi);
    }

    cv::Mat getImageFromContour(const contour_t &rect, const cv::Mat &image) {
        return getImageFromRotatedRect(RotRect(rect), image);
    }

    cv::Mat getImageFromRotatedRect(const cv::RotatedRect &rect, const cv::Mat &image) {
        // thanks to http://felix.abecassis.me/2011/10/opencv-rotation-deskewing/

        cv::Mat rotated, rotationMat, originalOut;
        cv::Mat returnImage = cv::Mat::zeros(rect.size, image.type());

        // Gets the rotation matrix
        rotationMat = cv::getRotationMatrix2D(rect.center, rect.angle, 1.0);

        // perform the affine transformation
        cv::warpAffine(
                image,
                rotated,
                rotationMat,
                image.size(),
                cv::INTER_LINEAR,
                cv::BORDER_CONSTANT,
                cv::Scalar(0, 0, 0)
        );

        // crop the resulting image
        cv::getRectSubPix(rotated, rect.size, rect.center, returnImage);

        return returnImage;
    }

    double getPercentFilled(const cv::Mat &image, const cv::Rect &rectangle) {
        cv::Mat opImage;
        if (image.channels() > 1)
            cv::cvtColor(image, opImage, CV_BGR2GRAY);
        else
            image.copyTo(opImage);

        auto totalCount = rectangle.height * rectangle.width;
        auto whiteCount = cv::countNonZero(opImage(rectangle));
        return totalCount != 0 ? (whiteCount / totalCount) * 100 : 0;
    }

    double getPercentFilled(const cv::Mat &image, const cv::RotatedRect &rectangle) {
        cv::Mat opImage;
        image.channels() > 1 ? cv::cvtColor(image, opImage, CV_BGR2GRAY) : image.copyTo(opImage);
        // image to contain the rectangle drawn
        cv::Mat rotRectDraw = cv::Mat::zeros(opImage.size(), CV_8UC1);
        // Got the rotated rect and its points
        contour_t rectContour;
        retrieveContourRotRect(rectangle, rectContour);
        // The rotated rect is filled and draw
        cv::fillConvexPoly(rotRectDraw, rectContour, cv::Scalar(255));

        // Find the enclosing upright rectangle for the ROI
        cv::Rect upRightRect = cv::boundingRect(rectContour);

        // Clips to make sure doesn't go over the borders
        upRightRect.x = MIN(MAX(upRightRect.x, 0), image.size().width);
        upRightRect.y = MIN(MAX(upRightRect.y, 0), image.size().height);

        upRightRect.width = MIN(MAX(upRightRect.x + upRightRect.width, 0), image.size().width) - upRightRect.x;
        upRightRect.height = MIN(MAX(upRightRect.y + upRightRect.height, 0), image.size().height) - upRightRect.y;

        // Gets the ROI
        cv::Mat roiGray = opImage(upRightRect);
        cv::Mat roiRectGray = rotRectDraw(upRightRect);

        auto rotRectPix = cv::countNonZero(roiRectGray);

        cv::Mat notFilledPix;
        cv::subtract(roiRectGray, roiGray, notFilledPix);
        auto countNonZeroResult = cv::countNonZero(notFilledPix);
        return (1 - (countNonZeroResult / rotRectPix)) * 100;
    }

    cv::Mat rotateImage(const cv::Mat &in, rotationType rotation, symmetryType symmetry) {
        // Could use extractRotation function with rotated rect?
        cv::Size size(in.cols, in.rows);
        cv::Point2f origin(0, 0);
        cv::RotatedRect rect(origin, size, 0);
        cv::Mat out = in.clone();

        cv::Mat rotationMat;
        // Rotation first for no real reason...
        switch (rotation) {
            case R_90:
                rotationMat = cv::getRotationMatrix2D(cv::Point((in.cols) / 2, (in.rows) / 2), -90, 1.0);
                cv::warpAffine(in, out, rotationMat, in.size(), cv::INTER_AREA);
                break;
            case R_180:
                cv::flip(in, out, -1);
                break;
            case R_270:
                rotationMat = cv::getRotationMatrix2D(cv::Point((in.cols) / 2, (in.rows) / 2), 90, 1.0);
                cv::warpAffine(in, out, rotationMat, in.size(), cv::INTER_AREA);
                break;
            case R_NONE:
                out = in;
                break;
        }

        switch (symmetry) {
            case S_X_AXIS:
                cv::flip(out, out, 0);
                break;
            case S_Y_AXIS:
                cv::flip(out, out, 1);
                break;
            case S_BOTH:
                cv::flip(out, out, -1);
                break;
            case S_NONE:
                break;
        }
        return out;
    }

    void drawRectangle(cv::Point2f *pts, cv::Mat &image, const cv::Scalar &color) {
        if (image.data == nullptr) return;

        cv::line(image, pts[0], pts[1], color, 3);
        cv::line(image, pts[1], pts[2], color, 3);
        cv::line(image, pts[2], pts[3], color, 3);
        cv::line(image, pts[3], pts[0], color, 3);
    }

    void inverseImage(const cv::Mat &in, cv::Mat &out) {
        cv::Mat temp(in.rows, in.cols, CV_16SC1);

        // Enables negative numbers
        in.convertTo(temp, CV_16SC1);
        // pixel of value 1 become -254
        // pixel of value 250 become -5
        cv::subtract(
                temp,
                cv::Mat(in.rows, in.cols, CV_16SC1, cv::Scalar(255, 255, 255)),
                temp
        );

        // Put back positive values
        cv::multiply(temp, cv::Mat::ones(temp.size(), CV_16SC1), temp, -1);
        // Convert in uchar type
        temp.convertTo(out, CV_8UC1);
    }

    bool isRectangle(contour_t &contour, unsigned int accuracy) {
        // Clip to make sure we have a good index.
        // unsigned, so no check for negative.
        if (accuracy >= ACCURACY_TABLE_SIZE)
            accuracy = ACCURACY_TABLE_SIZE - 1;

        // If less than 4 points, cannot be a rectangle
        if (contour.size() < 4) return false;
        int trueSquareAngleCount = 0;

        // Stores the lines between the points.
        std::vector<cv::Vec3f> vertices, longestVertices;
        std::vector<std::pair<unsigned int, cv::Vec3f>> sortedVertices;
        size_t j, size;
        for (j = 0, size = contour.size(); j < size; j++) {
            // Go to next point. If over the contour size,
            // get the first one to close the contour
            size_t k = j + 1;
            if (k >= size) k = 0;
            cv::Point val = contour[k] - contour[j];
            sortedVertices.emplace_back(j, cv::Vec3f(val.x, val.y, 0.0f));
            //		vertices.push_back(cv::Vec3f(val.x , val.y, 0.0f));
        }
        // X point means X vertices with this implementation.
        // Since we know it cannot be smaller than 4, no need to double
        // check the size.
        // Set the longest vertices first.
        std::sort(sortedVertices.begin(), sortedVertices.end(), compareVerticesLength);
        // Then take the four first and resort them in order of index to keep cross
        // product
        // valid.
        std::sort(sortedVertices.begin(), (sortedVertices.begin() + 4), compareVerticesIndex);
        // Get the four first vertices, the longest because of the sort
        for (j = 0; j < 4; j++) {
            longestVertices.push_back(sortedVertices[j].second);
        }

        // to have a single loop and less if statement,
        // we add the first contours to the end, so we can get the last
        // getAngle value.
        longestVertices.push_back(longestVertices[0]);
        for (j = 0, size = longestVertices.size() - 1; j < size; j++) {
            cv::Vec3f A = longestVertices[j];
            cv::Vec3f B = -longestVertices[j + 1];
            cv::Vec3f C = A.cross(B);
            auto ninetyNorm = norm(C) / (norm(A) * norm(B));
            // Do not check for negativity because inner contour runs
            // clockwise, so they always give negative.
            if (ninetyNorm >= ACCURACY_TABLE[accuracy]) trueSquareAngleCount++;
        }

        return trueSquareAngleCount == 4;
    }

    bool
    isSquare(std::vector<cv::Point> &approx, double min_area, double angle, double ratio_min, double ratio_max) {
        if (approx.size() == 4 &&
            std::fabs(cv::contourArea(cv::Mat(approx))) > min_area &&
            cv::isContourConvex(cv::Mat(approx))) {
            double maxCosine = 0;
            std::vector<double> eigenValues;
            eigenValues = getEigenValues(approx);
            double ratio = fabs(eigenValues[0] / eigenValues[1]);

            for (int j = 2; j < 5; j++) {
                double cosine = std::fabs(
                        getAngleBetweenPoints(approx[j % 4], approx[j - 2], approx[j - 1])
                );
                maxCosine = MAX(maxCosine, cosine);
            }

            // Check if angleBetweenThreePoints inside are un angle_ and check if
            // between ratios of PCA Eigen values
            if (maxCosine < angle && ratio < ratio_max && ratio > ratio_min) {
                return true;
            } else {
                return false;
            }

        } else {
            return false;
        }
    }

    void pcaAnalysis(std::vector<cv::Point> &pts, cv::PCA &pca) {
        // Construct a buffer used by the pca analysis
        cv::Mat data_pts = cv::Mat((int) pts.size(), 2, CV_64FC1);

        for (int i = 0; i < data_pts.rows; ++i) {
            data_pts.at<double>(i, 0) = pts[i].x;
            data_pts.at<double>(i, 1) = pts[i].y;
        }

        // Perform PCA
        pca = cv::PCA(data_pts, cv::Mat(), CV_PCA_DATA_AS_ROW);
    }

    cv::Point getEigenPosition(std::vector<cv::Point> &pts) {
        cv::PCA pca;
        pcaAnalysis(pts, pca);

        // Store the position of the object
        cv::Point pos;
        pos.x = (int) pca.mean.at<double>(0, 0);
        pos.y = (int) pca.mean.at<double>(0, 1);

        return pos;
    }

    std::vector<double> getEigenValues(std::vector<cv::Point> &pts) {
        cv::PCA pca;
        pcaAnalysis(pts, pca);

        // Store the eigenvalues
        std::vector<double> eigen_val(2);
        for (int i = 0; i < 2; ++i) {
            eigen_val[i] = pca.eigenvalues.at<double>(0, i);
        }

        return eigen_val;
    }

    std::vector<cv::Point2d> getEigenVectors(std::vector<cv::Point> &pts) {
        cv::PCA pca;
        pcaAnalysis(pts, pca);

        std::vector<cv::Point2d> eigen_vector(2);
        for (int i = 0; i < 2; ++i) {
            eigen_vector[i] = cv::Point2d(pca.eigenvectors.at<double>(i, 0), pca.eigenvectors.at<double>(i, 1));
        }

        return eigen_vector;
    }

    double getAngleBetweenPoints(const cv::Point &pt1, const cv::Point &pt2, const cv::Point &pt0) {
        double dx1 = pt1.x - pt0.x;
        double dy1 = pt1.y - pt0.y;
        double dx2 = pt2.x - pt0.x;
        double dy2 = pt2.y - pt0.y;
        return (dx1 * dx2 + dy1 * dy2) /
               sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
    }

    void drawSquares(cv::Mat &image, const std::vector<std::vector<cv::Point>> &squares) {
        for (const auto &square : squares) {
            const cv::Point *p = &square[0];
            int n = (int) square.size();
            polylines(image, &p, &n, 1, true, cv::Scalar(0, 255, 0), 10, CV_AA);
        }
    }

    bool compareCoordinates(const cv::Point &p1, const cv::Point &p2) {
        return std::tie(p1.x, p1.y) < std::tie(p2.x, p2.y);
    }

    float getMedian(std::vector<float> values) {
        size_t size = values.size();
        std::sort(values.begin(), values.end());
        auto median = size % 2 == 0 ? (values[size / 2 - 1] + values[size / 2]) / 2 : values[size / 2];
        return median;
    };

}  // namespace proc_image_processing
