/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#include <proc_image_processing/algorithm/general_function.h>
#include <tuple>

namespace proc_image_processing {

  //==============================================================================
  // M E T H O D S   S E C T I O N

  //------------------------------------------------------------------------------
  //
  std::vector<cv::Mat> GetColorPlanes(cv::Mat image) {
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

  //------------------------------------------------------------------------------
  //
  void SetCameraOffset(cv::Point& pt, int rows, int cols) {
    pt.x = pt.x - (cols / 2);
    pt.y = -(pt.y - (rows / 2));
  }

  //------------------------------------------------------------------------------
  //
  void RetrieveContours(cv::Mat image, contourList_t& contours) {
    RetrieveOuterContours(image, contours);
  }

  //------------------------------------------------------------------------------
  //
  void RetrieveInnerContours(cv::Mat image, contourList_t& contours) {
    if (image.channels() != 1) return;

    contourList_t contourVect;
    hierachy_t hierarchy;
    // CLone beacause find contour modifies the image.
    cv::Mat temp = image.clone();
    cv::findContours(temp, contourVect, hierarchy, CV_RETR_TREE,
      CV_CHAIN_APPROX_SIMPLE);

    for (int i = 0; i < (int)hierarchy.size(); i++) {
      // No child means no contour inside that contours.
      // 1 parent mean that it is inside a contour.
      if (hierarchy[i][FIRST_CHILD_CTR] == -1 && hierarchy[i][PARENT_CTR] != -1) {
        std::vector<cv::Point> tempContour;
        cv::approxPolyDP(contourVect[i], tempContour, 3, true);
        contours.push_back(tempContour);
      }
    }
  }

  //------------------------------------------------------------------------------
  //
  void RetrieveAllInnerContours(cv::Mat image, contourList_t& contours) {
    if (image.channels() != 1) return;

    contourList_t contourVect;
    hierachy_t hierarchy;
    // CLone beacause find contour modifies the image.
    cv::Mat temp = image.clone();
    cv::findContours(temp, contourVect, hierarchy, CV_RETR_TREE,
      CV_CHAIN_APPROX_SIMPLE);

    for (int i = 0; i < (int)hierarchy.size(); i++) {
      // No child means no contour inside that contours.
      // 1 parent mean that it is inside a contour.
      if (hierarchy[i][PARENT_CTR] != -1) {
        std::vector<cv::Point> tempContour;
        cv::approxPolyDP(contourVect[i], tempContour, 3, true);
        contours.push_back(tempContour);
      }
    }
  }

  //------------------------------------------------------------------------------
  //
  void RetrieveOuterContours(cv::Mat image, contourList_t& contours) {
    if (image.channels() != 1) return;

    contourList_t contourVect;
    // CLone beacause find contour modifies the image.
    cv::Mat temp = image.clone();
    cv::findContours(temp, contourVect, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    for (int i = 0; i < (int)contourVect.size(); i++) {
      // TO CHECK : New tempContour each time?
      std::vector<cv::Point> tempContour;
      // Three is normal value... Purely hardcoded from xp.
      cv::approxPolyDP(contourVect[i], tempContour, 3, true);
      contours.push_back(tempContour);
    }
  }

  //------------------------------------------------------------------------------
  //
  void RetrieveOutNoChildContours(cv::Mat image, contourList_t& contours) {
    if (image.channels() != 1) return;

    contourList_t contourVect;
    hierachy_t hierachy;
    // CLone beacause find contour modifies the image.
    cv::Mat temp = image.clone();
    RetrieveHiearchyContours(image, contourVect, hierachy);
    for (int i = 0; i < (int)contourVect.size(); i++) {
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

  //------------------------------------------------------------------------------
  //
  void RetrieveAllContours(cv::Mat image, contourList_t& contours) {
    if (image.channels() != 1) return;
    contourList_t contourVect;

    // CLone beacause find contour modifies the image.
    cv::Mat temp = image.clone();
    cv::findContours(temp, contourVect, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
    for (int i = 0; i < (int)contourVect.size(); i++) {
      // TO CHECK : New tempContour each time?
      std::vector<cv::Point> tempContour;
      // Three is normal value... Purely hardcoded from xp.
      cv::approxPolyDP(contourVect[i], tempContour, 3, true);
      contours.push_back(tempContour);
    }
  }

  //------------------------------------------------------------------------------
  //
  void RetrieveHiearchyContours(cv::Mat image, contourList_t& contours,
    hierachy_t& hierarchy) {
    if (image.channels() != 1) return;

    // CLone beacause find contour modifies the image.
    cv::Mat temp = image.clone();
    cv::findContours(temp, contours, hierarchy, CV_RETR_TREE,
      CV_CHAIN_APPROX_SIMPLE);

    for (int i = 0; i < (int)hierarchy.size(); i++) {
      cv::approxPolyDP(contours[i], contours[i], 3, true);
    }
  }

  //------------------------------------------------------------------------------
  //
  void RetrieveContourRotRect(cv::RotatedRect rect, contour_t& contour) {
    cv::Point2f pts[4];
    rect.points(pts);

    for (int j = 0; j < 4; j++) {
      contour.push_back(pts[j]);
    }
  }

  ////------------------------------------------------------------------------------
  ////
  //float StandardizeAngle()

  //------------------------------------------------------------------------------
  //
  Line FitLineOnPolygone(contour_t contour, int cols) {
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

  //------------------------------------------------------------------------------
  //
  Line GetPerpendicularLine(Line line, cv::Point2f center) {
    cv::Point p1;
    cv::Point p2;

    cv::Point2f vector = line.PerpendicularLine();

    p1 = center;
    p2 = vector + center;

    cv::Point2f doubleVector = -(p2 - p1);

    cv::Point start = doubleVector + center;
    cv::Point end = vector + center;

    Line perpendicularLine(start, end);
    return perpendicularLine;

  }


  //------------------------------------------------------------------------------
  //
  float CalculateRatio(float width, float height) {
    if (width == 0 && height == 0) return 0;
    return std::min((height / width), (width / height)) * 100;
  }

  //------------------------------------------------------------------------------
  //
  float CalculateConvexityRatio(contour_t contour) {
    if (contour.size() <= 2) return -1;
    float convexHullArea = CalculateConvexHullArea(contour);
    if (convexHullArea == 0) return 0;
    return (cv::contourArea(contour) / convexHullArea) * 100;
  }

  //------------------------------------------------------------------------------
  //
  float CalculateConvexHullArea(contour_t contour) {
    if (contour.size() <= 2) return -1;
    contour_t convexHull;
    cv::convexHull(contour, convexHull, false, true);
    return cv::contourArea(convexHull, false);
  }

  //------------------------------------------------------------------------------
  //
  float CalculateCircleIndex(float area, float perimeter) {
    float radiusCircum = perimeter / (2 * M_PI);
    float radiusArea = sqrt(area / (M_PI));
    if (radiusCircum == 0 && radiusArea) return 0;
    return radiusCircum > radiusArea ? radiusArea / radiusCircum
      : radiusCircum / radiusArea;
  }

  //------------------------------------------------------------------------------
  //
  float CalculateCircleIndex(contour_t contour) {
    return CalculateCircleIndex(cv::contourArea(contour, false),
      cv::arcLength(contour, true));
  }

  //------------------------------------------------------------------------------
  //
  cv::Scalar CalculateMeans(contour_t contour, cv::Mat image, bool middle) {
    cv::Mat opImage;
    if (image.channels() > 1)
      cv::cvtColor(image, opImage, CV_BGR2GRAY);
    else
      image.copyTo(opImage);
    cv::Mat matRoi;
    cv::Rect boundRect = cv::boundingRect(cv::Mat(contour));
    if (middle) {
      cv::Rect roi((boundRect.x + 0.25 * boundRect.width),
        (boundRect.y + 0.25 * boundRect.height), 0.5 * boundRect.width,
        0.5 * boundRect.height);

      matRoi = cv::Mat(opImage, roi);
    }
    else {
      matRoi = cv::Mat(opImage, boundRect);
    }
    return cv::mean(matRoi);
  }

  //------------------------------------------------------------------------------
  //
  cv::Mat ExtractImageFromRect(contour_t rect, cv::Mat image) {
    return ExtractImageFromRect(RotRect(rect), image);
  }

  //------------------------------------------------------------------------------
  //
  cv::Mat ExtractImageFromRect(cv::RotatedRect rect, cv::Mat image) {
    /*
     *
     *  thanks to http://felix.abecassis.me/2011/10/opencv-rotation-deskewing/
     *
     */

    cv::Mat rotated, rotationMat, originalOut;
    cv::Mat returnImage =
      cv::Mat::zeros(rect.size.height, rect.size.width, image.type());

    // Gets the rotation matrix
    rotationMat = cv::getRotationMatrix2D(rect.center, rect.angle, 1.0);

    // perform the affine transformation
    cv::warpAffine(image, rotated, rotationMat, image.size(), cv::INTER_LINEAR,
      cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

    // crop the resulting image

    cv::getRectSubPix(rotated, rect.size, rect.center, returnImage);

    return returnImage;
  }

  //------------------------------------------------------------------------------
  //
  float CalculatePourcentFilled(const cv::Mat& image, const cv::Rect& rectangle) {
    cv::Mat opImage;
    if (image.channels() > 1)
      cv::cvtColor(image, opImage, CV_BGR2GRAY);
    else
      image.copyTo(opImage);

    float totalCount = rectangle.height * rectangle.width;
    float whiteCount = cv::countNonZero(opImage(rectangle));
    if (totalCount != 0) return ((whiteCount / totalCount) * 100);
    return 0.0f;
  }

  //------------------------------------------------------------------------------
  //
  float CalculatePourcentFilled(const cv::Mat& image,
    const cv::RotatedRect& rectangle) {
    cv::Mat opImage;
    if (image.channels() > 1)
      cv::cvtColor(image, opImage, CV_BGR2GRAY);
    else
      image.copyTo(opImage);
    // image to contain the rectangle drawn
    cv::Mat rotRectDraw = cv::Mat::zeros(opImage.size(), CV_8UC1);
    // Got the rotated rect and its points
    contour_t rectContour;
    RetrieveContourRotRect(rectangle, rectContour);
    // The rotated rect is filled and draw
    cv::fillConvexPoly(rotRectDraw, rectContour, cv::Scalar(255));

    // Find the enclosing upright rectangle for the ROI
    cv::Rect upRightRect = cv::boundingRect(rectContour);

    // Clips to make sure doesn't go over the borders
    upRightRect.x = MIN(MAX(upRightRect.x, 0), image.size().width);
    upRightRect.y = MIN(MAX(upRightRect.y, 0), image.size().height);

    upRightRect.width =
      MIN(MAX(upRightRect.x + upRightRect.width, 0), image.size().width) -
      upRightRect.x;
    upRightRect.height =
      MIN(MAX(upRightRect.y + upRightRect.height, 0), image.size().height) -
      upRightRect.y;

    // Gets the ROI
    cv::Mat roiGray = opImage(upRightRect);
    cv::Mat roiRectGray = rotRectDraw(upRightRect);

    float rotRectPix = cv::countNonZero(roiRectGray);

    cv::Mat notFilledPix;
    cv::subtract(roiRectGray, roiGray, notFilledPix);
    float countNonZeroResult = cv::countNonZero(notFilledPix);
    return (1 - (countNonZeroResult / rotRectPix)) * 100;
  }

  //------------------------------------------------------------------------------
  //
  cv::Mat RotateImage(cv::Mat in, rotationType rotation, symmetryType symmetry) {
    // Could use extractRotation function with rotated rect?
    cv::Size size(in.cols, in.rows);
    cv::Point2f origin(0, 0);
    cv::RotatedRect rect(origin, size, 0);
    cv::Mat out = in.clone();

    cv::Mat rotationMat;
    // Rotation first for no real reason...
    switch (rotation) {
    case R_90:
      rotationMat = cv::getRotationMatrix2D(
        cv::Point((in.cols) / 2, (in.rows) / 2), -90, 1.0);
      cv::warpAffine(in, out, rotationMat, in.size(), cv::INTER_AREA);
      break;
    case R_180:
      cv::flip(in, out, -1);
      break;
    case R_270:
      rotationMat = cv::getRotationMatrix2D(
        cv::Point((in.cols) / 2, (in.rows) / 2), 90, 1.0);
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

  //------------------------------------------------------------------------------
  //
  void drawRectangle(cv::Point2f* pts, cv::Mat& image, cv::Scalar color) {
    if (sizeof(pts) != 8 || image.data == nullptr) return;

    cv::line(image, pts[0], pts[1], color, 3);
    cv::line(image, pts[1], pts[2], color, 3);
    cv::line(image, pts[2], pts[3], color, 3);
    cv::line(image, pts[3], pts[0], color, 3);
  }

  //------------------------------------------------------------------------------
  //
  void InverseImage(const cv::Mat& in, cv::Mat& out) {
    cv::Mat temp(in.rows, in.cols, CV_16SC1);

    // Enables negative numbers
    in.convertTo(temp, CV_16SC1);
    // pixel of value 1 become -254
    // pixel of value 250 become -5
    cv::subtract(temp,
      cv::Mat(in.rows, in.cols, CV_16SC1, cv::Scalar(255, 255, 255)),
      temp);

    // Put back positive values
    cv::multiply(temp, cv::Mat::ones(temp.size(), CV_16SC1), temp, -1);
    // Convert in uchar type
    temp.convertTo(out, CV_8UC1);
  }

  //------------------------------------------------------------------------------
  //
  bool IsRectangle(contour_t& contour, unsigned int degreeAcuracy) {
    // Clip to make sure we have a good index.
    // unsigned, so no check for negative.
    if (degreeAcuracy >= ACCURACY_TABLE_SIZE)
      degreeAcuracy = ACCURACY_TABLE_SIZE - 1;

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
      sortedVertices.push_back(
        std::pair<unsigned int, cv::Vec3f>(j, cv::Vec3f(val.x, val.y, 0.0f)));
      //		vertices.push_back(cv::Vec3f(val.x , val.y, 0.0f));
    }
    // X point means X vertices with this implementation.
    // Since we know it cannot be smaller than 4, no need to double
    // check the size.
    // Set the longest vertices first.
    std::sort(sortedVertices.begin(), sortedVertices.end(), SortVerticesLength);
    // Then take the four first and resort them in order of index to keep cross
    // product
    // valid.
    std::sort(sortedVertices.begin(), (sortedVertices.begin() + 4),
      SortVerticesIndex);
    // Get the four first vertices, the longest because of the sort
    for (j = 0; j < 4; j++) {
      longestVertices.push_back(sortedVertices[j].second);
    }

    // to have a single loop and less if statement,
    // we add the first contours to the end, so we can get the last
    // angle value.
    longestVertices.push_back(longestVertices[0]);
    for (j = 0, size = longestVertices.size() - 1; j < size; j++) {
      cv::Vec3f A = longestVertices[j];
      cv::Vec3f B = -longestVertices[j + 1];
      cv::Vec3f C = A.cross(B);
      float ninetyNorm = (norm(C)) / (norm(A) * norm(B));
      // Do not check for negativity because inner contour runs
      // clockwise, so they always give negative.
      if (ninetyNorm >= ACCURACY_TABLE[degreeAcuracy]) trueSquareAngleCount++;
    }

    return trueSquareAngleCount == 4;
  }

  //------------------------------------------------------------------------------
  //
  bool IsSquare(std::vector<cv::Point>& approx, double min_area, double angle,
    double ratio_min, double ratio_max) {
    if (approx.size() == 4 &&
      std::fabs(cv::contourArea(cv::Mat(approx))) > min_area &&
      cv::isContourConvex(cv::Mat(approx))) {
      double maxCosine = 0;
      std::vector<double> eigenValues;
      eigenValues = GetEigenValues(approx);
      double ratio = fabs(eigenValues[0] / eigenValues[1]);

      for (int j = 2; j < 5; j++) {
        double cosine = std::fabs(
          AngleBetweenThreePoints(approx[j % 4], approx[j - 2], approx[j - 1]));
        maxCosine = MAX(maxCosine, cosine);
      }

      // Check if angleBetweenThreePoints inside are un angle_ and check if
      // between ratios of PCA Eigen values
      if (maxCosine < angle && ratio < ratio_max && ratio > ratio_min) {
        return true;
      }
      else {
        return false;
      }

    }
    else {
      return false;
    }
  }

  //------------------------------------------------------------------------------
  //
  cv::Point GetEigenPos(std::vector<cv::Point>& pts) {
    // Construct a buffer used by the pca analysis
    cv::Mat data_pts = cv::Mat((int)pts.size(), 2, CV_64FC1);
    for (int i = 0; i < data_pts.rows; ++i) {
      data_pts.at<double>(i, 0) = pts[i].x;
      data_pts.at<double>(i, 1) = pts[i].y;
    }

    // Perform PCA analysis
    cv::PCA pca_analysis(data_pts, cv::Mat(), CV_PCA_DATA_AS_ROW);

    // Store the position of the object
    cv::Point pos;
    pos.x = (int)pca_analysis.mean.at<double>(0, 0);
    pos.y = (int)pca_analysis.mean.at<double>(0, 1);

    return pos;
  }

  //------------------------------------------------------------------------------
  //
  std::vector<double> GetEigenValues(std::vector<cv::Point>& pts) {
    // Construct a buffer used by the pca analysis
    cv::Mat data_pts = cv::Mat((int)pts.size(), 2, CV_64FC1);
    for (int i = 0; i < data_pts.rows; ++i) {
      data_pts.at<double>(i, 0) = pts[i].x;
      data_pts.at<double>(i, 1) = pts[i].y;
    }

    // Perform PCA analysis
    cv::PCA pca_analysis(data_pts, cv::Mat(), CV_PCA_DATA_AS_ROW);

    // Store the eigenvalues
    std::vector<double> eigen_val(2);
    for (int i = 0; i < 2; ++i) {
      eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
    }

    return eigen_val;
  }

  //------------------------------------------------------------------------------
  //
  std::vector<cv::Point2d> GetEigenVectors(std::vector<cv::Point>& pts) {
    // Construct a buffer used by the pca analysis
    cv::Mat data_pts = cv::Mat((int)pts.size(), 2, CV_64FC1);
    for (int i = 0; i < data_pts.rows; ++i) {
      data_pts.at<double>(i, 0) = pts[i].x;
      data_pts.at<double>(i, 1) = pts[i].y;
    }

    // Perform PCA analysis
    cv::PCA pca_analysis(data_pts, cv::Mat(), CV_PCA_DATA_AS_ROW);

    // Store the eigenvectors
    std::vector<cv::Point2d> eigen_vecs(2);
    for (int i = 0; i < 2; ++i) {
      eigen_vecs[i] = cv::Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
        pca_analysis.eigenvectors.at<double>(i, 1));
    }

    return eigen_vecs;
  }

  //------------------------------------------------------------------------------
  //
  double AngleBetweenThreePoints(cv::Point pt1, cv::Point pt2, cv::Point pt0) {
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1 * dx2 + dy1 * dy2) /
      sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
  }

  //------------------------------------------------------------------------------
  //
  void DrawSquares(cv::Mat& image,
    const std::vector<std::vector<cv::Point>>& squares) {
    for (size_t i = 0; i < squares.size(); i++) {
      const cv::Point* p = &squares[i][0];
      int n = (int)squares[i].size();
      polylines(image, &p, &n, 1, true, cv::Scalar(0, 255, 0), 10, CV_AA);
    }
  }

  //------------------------------------------------------------------------------
  //
  bool CompareYX(const cv::Point& p1, const cv::Point& p2) {
    return std::tie(p1.x, p1.y) < std::tie(p2.x, p2.y);
  }

  //------------------------------------------------------------------------------
  //
  float Median(std::vector<float> values) {
    float median;
    size_t size = values.size();

    std::sort(values.begin(), values.end());

    if (size % 2 == 0) {
      median = (values[size / 2 - 1] + values[size / 2]) / 2;
    }
    else {
      median = values[size / 2];
    }

    return median;
  };

}  // namespace proc_image_processing
