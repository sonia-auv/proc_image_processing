// FACTORY_GENERATOR_CLASS_NAME=OrbSiftMatch

#ifndef PROC_IMAGE_PROCESSING_ORB_SIFT_MATCH_H
#define PROC_IMAGE_PROCESSING_ORB_SIFT_MATCH_H

#include <proc_image_processing/cpu/config.h>
#include <proc_image_processing/cpu/filters/filter.h>
#include <proc_image_processing/cpu/algorithm/performance_evaluator.h>
#include "opencv2/core.hpp"
#include "opencv2/opencv_modules.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
//#include "opencv4/opencv2/xfeatures2d.hpp"

namespace proc_image_processing {

    class OrbSiftMatch : public Filter {
    public:
        using Ptr = std::shared_ptr<OrbSiftMatch>;

        explicit OrbSiftMatch(const GlobalParameterHandler &globalParams)
                : Filter(globalParams),
                  debug_contour_("Debug contour", false, &parameters_),
                  objective_("Objective", 0, 0, 11, &parameters_, "0=G-Man Gate, 1=Bootlegger Gate, 2=G-Man Badge, 3=Bootlegger Gun, 4=G-Man Phone, 5=G-Man Note, 6=Bootlegger Bottle, 7=Bootlegger Barrel, 8=G-Man Torpidoe, 9=Bootlegger Torpidoe, 10=G-Man Axe, 11=Bootlegger Dollar") {
            setName("OrbSiftMatch");
        }

        ~OrbSiftMatch() override = default;

        void apply(cv::Mat &image) override {
            //image.copyTo(output_image_);

            cv::Mat ref_image;
            std::string path;
            switch(objective_()) {
                case 0:
                    path = kRefImagesPath + "GManGate" + kImagesExt;
                    break;
                case 1:
                    path = kRefImagesPath + "BootleggerGate" + kImagesExt;
                    break;
                case 2:
                    path = kRefImagesPath + "GManBadge" + kImagesExt;
                    break;
                case 3:
                    path = kRefImagesPath + "BootleggerGun" + kImagesExt;
                    break;
                case 4:
                    path = kRefImagesPath + "GManPhone" + kImagesExt;
                    break;
                case 5:
                    path = kRefImagesPath + "GManNote" + kImagesExt;
                    break;
                case 6:
                    path = kRefImagesPath + "BootleggerBottle" + kImagesExt;
                    break;
                case 7:
                    path = kRefImagesPath + "BootleggerBarrel" + kImagesExt;
                    break;
                case 8:
                    path = kRefImagesPath + "GManTorpidoe" + kImagesExt;
                    break;
                case 9:
                    path = kRefImagesPath + "BootleggerTorpidoe" + kImagesExt;
                    break;
                case 10:
                    path = kRefImagesPath + "GManAxe" + kImagesExt;
                    break;
                case 11:
                    path = kRefImagesPath + "BootleggerDollar" + kImagesExt;
                    break;
                default:
                    ROS_WARN("Wrong value in sift match filter");
                    return;
            }

            ref_image = cv::imread(path);
            if (image.empty()) {
                ROS_WARN("Provider vision image is empty");
            }
            if (ref_image.empty()) {
                ROS_WARN("Ref image is empty : %s",path.c_str());
            }

            //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
            //int minHessian = 400;
            cv::Ptr<cv::ORB> detector = cv::ORB::create();
            std::vector<cv::KeyPoint> keypoints1, keypoints2;
            cv::Mat descriptors1, descriptors2;
            detector->detectAndCompute(ref_image,cv::noArray(),keypoints1,descriptors1);
            detector->detectAndCompute(image,cv::noArray(),keypoints2,descriptors2);

            if(descriptors1.empty())
            {
                ROS_WARN("Ref image descriptors is empty");
                return;
            }
            if(descriptors2.empty())
            {
                ROS_WARN("Provider vision image descriptors is empty");
                return;
            }
            descriptors1.convertTo(descriptors1,CV_32F);
            descriptors2.convertTo(descriptors2,CV_32F);
            
            //-- Step 2: Matching descriptor vectors with a FLANN based matcher
            cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
            std::vector< std::vector<cv::DMatch> > knn_matches;
            matcher->knnMatch( descriptors1, descriptors2, knn_matches, 2 );

            //-- Filter matches using the Lowe's ratio test
            const float ratio_thresh = 0.7f;
            std::vector<cv::DMatch> good_matches;
            for (size_t i = 0; i < knn_matches.size(); i++)
            {
                if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
                {
                    good_matches.push_back(knn_matches[i][0]);
                }
            }

            //-- Draw matches
            cv::Mat img_matches;
            cv::drawMatches( ref_image, keypoints1, image, keypoints2, good_matches, img_matches, cv::Scalar::all(-1),
                 cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

            img_matches.copyTo(output_image_);

            if (debug_contour_()) {
                output_image_.copyTo(image);
            }

        }

    private:
        cv::Mat output_image_;
        Parameter<bool> debug_contour_;
        RangedParameter<int> objective_;
    };
}  // namespace proc_image_processing


#endif //PROC_IMAGE_PROCESSING_ORB_SIFT_MATCH_H