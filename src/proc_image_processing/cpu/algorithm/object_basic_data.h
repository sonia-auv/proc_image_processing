/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>


#ifndef PROC_IMAGE_PROCESSING_ALGORITHM_OBJECT_BASIC_DATA_H_
#define PROC_IMAGE_PROCESSING_ALGORITHM_OBJECT_BASIC_DATA_H_

#include <assert.h>
#include "rot_rect.h"
#include "type_and_const.h"
#include <memory>
#include "contour.h"

namespace proc_image_processing {

    class ObjectBasicData {
    public:
        using Ptr = std::shared_ptr<ObjectBasicData>;

        static const int BLUE_PLANE = 0;
        static const int GREEN_PLANE = 1;
        static const int RED_PLANE = 2;
        static const int HUE_PLANE = 3;
        static const int SATURATION_PLANE = 4;
        static const int INTENSITY_PLANE = 5;
        static const int GRAY_PLANE = 6;
        static const int NB_OF_PLANE = 7;

        enum OBJECT_DATA {
            AREA,
            CONVEX_HULL,
            CIRCUMFERENCE,
            ROTATED_RECT,
            UP_RIGHT_RECT,
            MOMENTS,
            PLANES
        };

        ObjectBasicData(const cv::Mat &originalImage, const cv::Mat &binaryImage, Contour contour);

        virtual ~ObjectBasicData() = default;

        static void setPlaneInRange(int &planeID);

        // Voting system
        void vote();

        int getVoteCount() const;

        void resetVotes();

        // All getters calculate their element if they are not already calculated.
        // If they are, simply return them.
        float getArea();

        float getHeight();

        float getWidth();

        float getConvexHullArea();

        float getCircumference();

        const RotRect &getRotRect();

        float getAngle();

        cv::Point2f &getCenterPoint();

        const cv::Rect &getUprightRect();

        const cv::Moments &getMoments(bool binary);

        // Images are already reference in opencv...
        const cv::Mat &getPlanes(int planesID);

        cv::Mat getBinaryImageAtUprightRect();

        Contour getContourCopy();

        cv::Size getImageSize() const;

        const cv::Mat &getBinaryImage();

        const cv::Mat &getOriginalImage();

    private:
        std::map<OBJECT_DATA, bool> is_calculated_map_;

        float area_, convex_hull_area_, circumference_;

        RotRect rect_;
        cv::Rect up_right_rect_;
        cv::Moments cv_moments_;
        std::vector<cv::Mat> planes_;
        cv::Mat original_image_, binary_image_;

        int vote_count_;
        Contour contour_;
    };

    inline void ObjectBasicData::vote() { vote_count_++; }

    inline int ObjectBasicData::getVoteCount() const { return vote_count_; }

    inline void ObjectBasicData::resetVotes() { vote_count_ = 0; }

    inline float ObjectBasicData::getArea() {
        if (!is_calculated_map_[AREA]) {
            area_ = cv::contourArea(contour_.getContour(), false);
            is_calculated_map_[AREA] = true;
        }
        return area_;
    }

    inline float ObjectBasicData::getHeight() {
        if (!is_calculated_map_[ROTATED_RECT]) {
            rect_ = RotRect(contour_.getContour());
            is_calculated_map_[ROTATED_RECT] = true;
        }
        return rect_.size.height;
    }

    inline float ObjectBasicData::getWidth() {
        if (!is_calculated_map_[ROTATED_RECT]) {
            rect_ = RotRect(contour_.getContour());
            is_calculated_map_[ROTATED_RECT] = true;
        }
        return rect_.size.width;
    }

    inline void ObjectBasicData::setPlaneInRange(int &planeID) {
        // Clamping the planeID in [0; NB_OF_PLANE - 1]
        planeID =
                planeID < 0 ? 0 : (planeID > NB_OF_PLANE - 1 ? NB_OF_PLANE - 1 : planeID);
    }

    inline float ObjectBasicData::getConvexHullArea() {
        if (!is_calculated_map_[CONVEX_HULL]) {
            contour_t convexHull;
            cv::convexHull(contour_.getContour(), convexHull, false, true);
            convex_hull_area_ = cv::contourArea(convexHull, false);
            is_calculated_map_[CONVEX_HULL] = true;
        }
        return convex_hull_area_;
    }

    inline float ObjectBasicData::getCircumference() {
        if (!is_calculated_map_[CIRCUMFERENCE]) {
            circumference_ = cv::arcLength(contour_.getContour(), true);
            is_calculated_map_[CIRCUMFERENCE] = true;
        }
        return circumference_;
    }

    inline const RotRect &ObjectBasicData::getRotRect() {
        if (!is_calculated_map_[ROTATED_RECT]) {
            rect_ = RotRect(contour_.getContour());
            is_calculated_map_[ROTATED_RECT] = true;
        }
        return rect_;
    }

    inline float ObjectBasicData::getAngle() {
        getRotRect();
        return rect_.angle;
    }

    inline cv::Point2f &ObjectBasicData::getCenterPoint() {
        getRotRect();
        return rect_.center;
    }

    inline const cv::Rect &ObjectBasicData::getUprightRect() {
        if (!is_calculated_map_[UP_RIGHT_RECT]) {
            up_right_rect_ = cv::boundingRect(contour_.getContour());
            is_calculated_map_[UP_RIGHT_RECT] = true;
        }
        return up_right_rect_;
    }

    inline cv::Mat ObjectBasicData::getBinaryImageAtUprightRect() {
        // Making sure we have calculated the rectangle.
        cv::Rect uprightRect = getUprightRect();
        // Clone is necessary since the object is created now
        // OpenCV Mat are smart pointer
        return cv::Mat(binary_image_, uprightRect);
    }

    inline Contour ObjectBasicData::getContourCopy() { return contour_; }

    inline cv::Size ObjectBasicData::getImageSize() const { return original_image_.size(); }

    inline const cv::Mat &ObjectBasicData::getBinaryImage() { return original_image_; }

    inline const cv::Mat &ObjectBasicData::getOriginalImage() { return binary_image_; }

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_ALGORITHM_OBJECT_BASIC_DATA_H_
