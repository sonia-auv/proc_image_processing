// FACTORY_GENERATOR_CLASS_NAME=SiftMatch

#ifndef PROC_IMAGE_PROCESSING_SIFT_MATCH_H
#define PROC_IMAGE_PROCESSING_SIFT_MATCH_H

#include <proc_image_processing/cpu/config.h>
#include <proc_image_processing/cpu/filters/filter.h>
#include <proc_image_processing/cpu/algorithm/performance_evaluator.h>
#include "opencv2/core.hpp"
#include "opencv2/opencv_modules.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"


// Proposition: Au lieu d'avoir la détection d'un élément à la fois. Pourquoi ne pas faire tourner tous les éléments en même temps?

namespace proc_image_processing {

    class SiftMatch : public Filter {
    public:
        using Ptr = std::shared_ptr<SiftMatch>;
        cv::Ptr<cv::ORB> detector = cv::ORB::create();

        explicit SiftMatch(const GlobalParameterHandler &globalParams)
                : Filter(globalParams),
                  objective_("Objective", 0, 0, 10, &parameters_, "0=G-Man Gate, 1=Bootlegger Gate, 2=G-Man Buoys, 3=Bootlegger Gun, 4=G-Man Bin, 5=Bootlegger Bin, 6=G-Man Torpidoe, 7=Bootlegger Torpidoe, 8=G-Man Axe, 9=Bootlegger Dollar") {
            setName("SiftMatch");
        }

        ~SiftMatch() override = default;

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

            // Faire une liste avec les descriptors de chacune image de référence
            // (même si plus tard j'ai plusieurs images, je mets tous les descriptors ensemble)
            // Comparer l'image avec chacun des descriptors 
            //Afficher l'ensemble des points qui matchent avec la bonne couleur. 


            

            //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
            std::pair<cv::Mat,std::vector<cv::KeyPoint>> descriptors_keypoints2 = calculate_descriptors_and_kp(image);
            cv::Mat descriptors2 = descriptors_keypoints2.first;
            std::vector<cv::KeyPoint> keypoints2 = descriptors_keypoints2.second;
            //Là comme ça c'est plus long mais ça va surement s'arranger


            //maintenant on va générer toutes les descriptors et keypoints des images de références
            std::pair<std::vector<cv::Mat>,std::vector<std::vector<cv::KeyPoint>>> descr_kp_ref;
            descr_kp_ref = calculate_descr_kp_references();
            std::vector<cv::Mat> ref_descriptors;
            std::vector<std::vector<cv::KeyPoint>> ref_keypoints;
            ref_descriptors = descr_kp_ref.first;
            ref_keypoints = descr_kp_ref.second;

            cv::Mat descriptors1 = ref_descriptors[objective_()];
            std::vector<cv::KeyPoint> keypoints1 = ref_keypoints[objective_()];
            //Je récupère un élément. Le but est de modifier le code proressivement mais en s'assurant qu'il fonctionne tout le temps





            //pas encore modifié la suite


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
            std::vector<cv::Point> matching_points;
            for(size_t i = 0; i< good_matches.size(); i++){
                matching_points.push_back(keypoints2[good_matches[i].trainIdx].pt);
            }
            //Draw the point on the image
            cv::Mat img_keypoints;
            image.copyTo(img_keypoints);
            for(size_t i = 0; i< matching_points.size(); i++){
                cv::circle(img_keypoints, matching_points[i], 3, cv::Scalar(220,20,220), 5);
            }

            img_keypoints.copyTo(output_image_); // Just the points
            output_image_.copyTo(image);
        }
    
    //Fonction pour calculer les descripteurs pour une image 
    std::pair<cv::Mat,std::vector<cv::KeyPoint>> calculate_descriptors_and_kp(cv::Mat image){
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        detector->detectAndCompute(image,cv::noArray(),keypoints,descriptors);
        descriptors.convertTo(descriptors,CV_32F);
        return std::make_pair(descriptors,keypoints);
    }

    //Fonction pour calculer tous les descripteurs et keypoints des images de référence
    std::pair<std::vector<cv::Mat>,std::vector<std::vector<cv::KeyPoint>>> calculate_descr_kp_references(){
        std::vector<cv::Mat> descriptors;
        std::vector<std::vector<cv::KeyPoint>> keypoints;
        


        //List can be simplified when good data structures for images is chosen!!!!!!!!!
        std::vector<std::string> list_paths({kRefImagesPath + "01_chooseSide_gman" + kImagesExt,
                                    kRefImagesPath + "01_chooseSide_bootlegger" + kImagesExt,
                                    kRefImagesPath + "02_makeGrade_badge" + kImagesExt,
                                    kRefImagesPath + "02_makeGrade_tommyGun" + kImagesExt,
                                    kRefImagesPath + "03_collecting_gman_white" + kImagesExt,
                                    kRefImagesPath + "03_collecting_bootlegger_white" + kImagesExt,
                                    kRefImagesPath + "04_shootout_gman_red" + kImagesExt,
                                    kRefImagesPath + "04_shootout_bootlegger_red" + kImagesExt,
                                    kRefImagesPath + "05_cashSmash_axe_orange" + kImagesExt,
                                    kRefImagesPath + "05_cashSmash_dollar_orange" + kImagesExt,
                                    kRefImagesPath + "BootleggerGun.JPG"
        });
        
        for(size_t i = 0; i< list_paths.size(); i++){
            std::pair<cv::Mat,std::vector<cv::KeyPoint>> descr_kp;
            cv::Mat image_for_calculation = cv::imread(list_paths[i]);
            descr_kp = calculate_descriptors_and_kp(image_for_calculation);

            if(descr_kp.first.empty())
            {
                ROS_WARN("Ref image descriptors is empty for path : %s", list_paths[i].c_str());
            }

            descriptors.push_back(descr_kp.first);
            keypoints.push_back(descr_kp.second);
        }

        return std::make_pair(descriptors, keypoints);
    }

    private:
        cv::Mat output_image_;
        RangedParameter<int> objective_;
    };
}  // namespace proc_image_processing

#endif //PROC_IMAGE_PROCESSING_SIFT_MATCH_H