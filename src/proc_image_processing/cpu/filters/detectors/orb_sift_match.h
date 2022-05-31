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

//still slow because calculate reference image each time. 
//We need to implement the idea of the sift match if we want to use this filter correctly


namespace proc_image_processing {

    class OrbSiftMatch : public Filter {
    public:
        using Ptr = std::shared_ptr<OrbSiftMatch>;

        explicit OrbSiftMatch(const GlobalParameterHandler &globalParams)
                : Filter(globalParams),
                  debug_contour_("Debug contour", false, &parameters_),
                  objective_("Objective", 0, 0, 10, &parameters_, 
                  "0=G-Man Gate, 1=Bootlegger Gate, 2=G-Man Buoys, 3=Bootlegger Gun, 4=G-Man Bin, 5=Bootlegger Bin, 6=G-Man Torpidoe, 7=Bootlegger Torpidoe, 8=G-Man Axe, 9=Bootlegger Dollar") {
            setName("OrbSiftMatch");
        }

        ~OrbSiftMatch() override = default;

        void apply(cv::Mat &image) override {
            //image.copyTo(output_image_);

            cv::Mat ref_image;
            std::string path;
            switch(objective_()) {
                case 0:
                    path = kRefImagesPath + "01_chooseSide_gman" + kImagesExt;
                    break;
                case 1:
                    path = kRefImagesPath + "01_chooseSide_bootlegger" + kImagesExt;
                    break;
                case 2:
                    path = kRefImagesPath + "02_makeGrade_badge" + kImagesExt;
                    break;
                case 3:
                    path = kRefImagesPath + "02_makeGrade_tommyGun" + kImagesExt;
                    break;
                case 4:
                    path = kRefImagesPath + "03_collecting_gman_white" + kImagesExt;
                    break;
                case 5:
                    path = kRefImagesPath + "03_collecting_bootlegger_white" + kImagesExt;
                    break;
                case 6:
                    path = kRefImagesPath + "04_shootout_gman_red" + kImagesExt;
                    break;
                case 7:
                    path = kRefImagesPath + "04_shootout_bootlegger_red" + kImagesExt;
                    break;
                case 8:
                    path = kRefImagesPath + "05_cashSmash_axe_orange" + kImagesExt;
                    break;
                case 9:
                    path = kRefImagesPath + "05_cashSmash_dollar_orange" + kImagesExt;
                    break;
                case 10:
                    path = kRefImagesPath + "BootleggerGun.JPG";
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
            cv::Ptr<cv::ORB> detector = cv::ORB::create(500, 1.3f, 10);
            std::vector<cv::KeyPoint> keypoints1, keypoints2;
            cv::Mat descriptors1, descriptors2;
            detector->detectAndCompute(ref_image,cv::noArray(),keypoints1,descriptors1);
            detector->detectAndCompute(image,cv::noArray(),keypoints2,descriptors2);

            if(descriptors1.empty())
            {
                ROS_WARN("Ref image descriptors is empty");
                return;
            }
            if(keypoints2.size() < 2) // descriptors and keypoints have the same size
            {
                //ROS_WARN("Provider vision image descriptors is empty (or smaller than 2)");
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


            // --- Draw only points
            //Get the points that match
            std::vector<cv::Point> matching_points; // old
            // std::vector<cv::KeyPoint> matching_keypoints; // New
            for(size_t i = 0; i< good_matches.size(); i++){
                matching_points.push_back(keypoints2[good_matches[i].trainIdx].pt);
                // matching_keypoints.push_back(keypoints2[good_matches[i].trainIdx]);
            }
            //Draw the point on the image
            cv::Mat img_keypoints;
            image.copyTo(img_keypoints);
            for(size_t i = 0; i< matching_points.size(); i++){
                cv::circle(img_keypoints, matching_points[i], 3, cv::Scalar(220,20,220), 5);
            }

            //Draw all keypoints (doesn't care what match)
            // cv::drawKeypoints(image,keypoints2,img_keypoints,cv::Scalar(0,255,0),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            
            //-- Draw matches
            cv::Mat img_matches;
            cv::drawMatches( ref_image, keypoints1, image, keypoints2, good_matches, img_matches, cv::Scalar::all(-1),
                 cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );


            if (debug_contour_()) {
                img_matches.copyTo(output_image_); //Image to compare with
            }else{
                img_keypoints.copyTo(output_image_); // Just the points
            }

            output_image_.copyTo(image);



        }

    private:
        cv::Mat output_image_;
        Parameter<bool> debug_contour_;
        RangedParameter<int> objective_;
    };
}  // namespace proc_image_processing


#endif //PROC_IMAGE_PROCESSING_ORB_SIFT_MATCH_H