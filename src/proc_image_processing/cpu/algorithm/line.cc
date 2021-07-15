/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#include "line.h"

namespace proc_image_processing {

    Line::Line(const cv::Point &start, const cv::Point &end)
            : start_(start), end_(end), angle_(0), isSwap_(false) {
        if (start_.x > end_.x) {
            isSwap_ = true;
            std::swap(start_, end_);
        }

        int yDiff = abs(start_.y - end_.y);
        int xDiff = abs(start_.x - end_.x);

        if (start_.y > end_.y)
            center_.y = end_.y + yDiff / 2;
        else
            center_.y = start_.y + yDiff / 2;

        if (start_.x > end_.x)
            center_.x = end_.x + xDiff / 2;
        else
            center_.x = start_.x + xDiff / 2;

        // inversion in the start_ end_ for x y is because y is positive
        // downward.
        float at = atan2(static_cast<double>(start_.y - end_.y),
                         static_cast<double>(end_.x - start_.x));
        angle_ = at / (2 * M_PI) * 360;

        length_ = sqrt(pow((start_.y - end_.y), 2) + pow((start_.y - end_.y), 2));
    }

    void Line::draw(cv::Mat &img, cv::Scalar color) {
        cv::line(img, start_, end_, color, 4, 8);
    }

    cv::Point Line::getPerpendicularLine() {
        std::vector<cv::Point> secondLine;

        cv::Point vector1;
        Eigen::Vector3d vector2;
        vector1 = end_ - start_;

        Eigen::Vector3d vector(vector1.x, vector1.y, 1);

        double min = fabs(vector.x());
        Eigen::Vector3d cardinalAxis;
        cardinalAxis << 1, 0, 1;

        if (fabs(vector.y()) < min) {
            cardinalAxis << 0, 1, 1;
        }

        vector2 = vector.cross(cardinalAxis);

        return cv::Point(vector2.x(), vector2.y());
    }

    std::vector<cv::Point> Line::getPoints(cv::Mat &img) {
        std::vector<cv::Point> line;
        cv::LineIterator it(img, start_, end_);

        for (int i = 0; i < it.count; i++, ++it) {
            cv::Point p = it.pos();
            line.push_back(p);
        }

        return line;
    }

    cv::Point Line::getCenter() { return center_; }

    cv::Point Line::getStart() { return start_; }

    cv::Point Line::getEnd() { return end_; }

    float Line::getAngle() { return angle_; }

    float Line::getLength() { return length_; }

    bool lengthSort(Line a, Line b) { return a.getLength() > b.getLength(); }

    bool centerXSort(Line a, Line b) { return a.getCenter().x > b.getCenter().x; }

    bool centerYSort(Line a, Line b) { return a.getCenter().y > b.getCenter().y; }

}  // namespace proc_image_processing
