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
#include<string>


namespace proc_image_processing {

    class SiftMatch : public Filter {
    public:
        using Ptr = std::shared_ptr<SiftMatch>;
        cv::Ptr<cv::ORB> detector = cv::ORB::create();
        std::vector<cv::Mat> ref_descriptors;
        // std::vector<std::vector<cv::KeyPoint>> ref_keypoints; //pas utile les kp de référence
        

        explicit SiftMatch(const GlobalParameterHandler &globalParams)
                : Filter(globalParams),
                  objective_("Objective", 0, 0, 10, &parameters_, "0=ALL, 1=ChooseSide, 2=MakeGrade, 3=Collecting, 4=Shoutout, 5=CashSmash"){
            setName("SiftMatch");
            



            //Je constate que le code ici n'est effectué qu'une seule fois donc je met mon ouverture de fichier ici
            // Ce n'est peut-être pas le bon endroit
            std::string descr_path = kConfigPath + "/descriptors/Descriptors.yml";
            create_reference_descriptors(descr_path); // Je peux le calculer à chaque fois mais je peux aussi le commenter pour gagner Quelques ms

            //Lecture des infos depuis les descripteurs

            cv::FileStorage fsRead;
            std::vector<std::string> list_paths;
            fsRead.open(descr_path, cv::FileStorage::READ);
            fsRead["indexes"] >> list_paths;
            for(int i = 0; i< list_paths.size();i++){
                cv::Mat temp_descriptor;
                fsRead[list_paths[i].substr(3)] >> temp_descriptor;   
                ref_descriptors.push_back(temp_descriptor);

                // std::vector<cv::KeyPoint> temp_kp;
                // fsRead["keypoints_" + std::to_string(i)] >> temp_kp;  
                // ref_keypoints.push_back(temp_kp);

            }
            fsRead.release();             
        }





        ~SiftMatch() override = default;

        void apply(cv::Mat &image) override {

            //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
            std::pair<cv::Mat,std::vector<cv::KeyPoint>> descriptors_keypoints = calculate_descriptors_and_kp(image);
            cv::Mat im_descriptors = descriptors_keypoints.first;
            std::vector<cv::KeyPoint> im_keypoints = descriptors_keypoints.second;

            if(im_keypoints.size() < 2) // si je n'ai pas de descripteurs, ça ne sert à rien de faire des calculs
            {
                //ROS_WARN("Less than 2 key points on the image");
                return; 
            }


            
            //-- Step 2: Matching descriptor vectors with a FLANN based matcher
            std::vector<std::vector<cv::Point>> matching_points_list;
            std::vector<cv::Mat> temp_ref_descriptors;

            switch(objective_()) {  
                case 1: // Choose Side
                    temp_ref_descriptors.push_back(ref_descriptors[0]);
                    temp_ref_descriptors.push_back(ref_descriptors[1]);
                    matching_points_list = create_matcher_list(temp_ref_descriptors, im_descriptors, im_keypoints);
                    break;
                case 2: // Make Grade
                    temp_ref_descriptors.push_back(ref_descriptors[2]);
                    temp_ref_descriptors.push_back(ref_descriptors[3]);
                    matching_points_list = create_matcher_list(temp_ref_descriptors, im_descriptors, im_keypoints);
                    break;
                case 3: // Collecting
                    temp_ref_descriptors.push_back(ref_descriptors[4]);
                    temp_ref_descriptors.push_back(ref_descriptors[5]);
                    matching_points_list = create_matcher_list(temp_ref_descriptors, im_descriptors, im_keypoints);
                    break;
                case 4: //Shoutout
                    temp_ref_descriptors.push_back(ref_descriptors[6]);
                    temp_ref_descriptors.push_back(ref_descriptors[7]);
                    matching_points_list = create_matcher_list(temp_ref_descriptors, im_descriptors, im_keypoints);
                    break;
                case 5: // Cash Shmash
                    temp_ref_descriptors.push_back(ref_descriptors[8]);
                    temp_ref_descriptors.push_back(ref_descriptors[9]);
                    matching_points_list = create_matcher_list(temp_ref_descriptors, im_descriptors, im_keypoints);
                    break;
                default:// Cas par défaut, on affiche tous les points 
                    matching_points_list = create_matcher_list(ref_descriptors, im_descriptors, im_keypoints);
            }

            
            //Liste des couleurs
            std::vector<cv::Scalar> colors;
            colors.push_back(cv::Scalar(0,0,255)); //"01_chooseSide_gman"
            colors.push_back(cv::Scalar(255,0,00)); //"01_chooseSide_bootlegger"
            colors.push_back(cv::Scalar(0,128,255)); //"02_makeGrade_badge"
            colors.push_back(cv::Scalar(255,255,0)); //"02_makeGrade_tommyGun"
            colors.push_back(cv::Scalar(255,128,0)); //"03_collecting_gman_white"
            colors.push_back(cv::Scalar(0,255,255)); //"03_collecting_bootlegger_white"
            colors.push_back(cv::Scalar(0,250,128)); //"04_shootout_gman_red"
            colors.push_back(cv::Scalar(128,255,0)); //"04_shootout_bootlegger_red"
            colors.push_back(cv::Scalar(0,128,128)); //"05_cashSmash_axe_orange"
            colors.push_back(cv::Scalar(128,128,0)); //"05_cashSmash_dollar_orange"

            //BGR d'après la doc

            //Draw the point on the image
            cv::Mat img_keypoints;
            image.copyTo(img_keypoints);
            for(size_t j = 0; j< matching_points_list.size(); j++){
                std::vector<cv::Point> matching_points = matching_points_list[j];
                for(size_t i = 0; i< matching_points.size(); i++){
                    cv::circle(img_keypoints, matching_points[i], 3, colors[j], 5);
                }
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


    //-- Step 2: Matching descriptor vectors with a FLANN based matcher
    std::vector<std::vector<cv::Point>> create_matcher_list(std::vector<cv::Mat> reference_descriptors,cv::Mat image_descriptors,std::vector<cv::KeyPoint> image_keypoints){        
        cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);

        std::vector<std::vector<cv::Point>> matching_points_list;
        for(size_t i = 0; i< reference_descriptors.size(); i++){
            cv::Mat ref_descriptor = reference_descriptors[i];
            std::vector<std::vector<cv::DMatch>> knn_matches;
            
            matcher->knnMatch( ref_descriptor, image_descriptors, knn_matches, 2 );

            //-- Filter matches using the Lowe's ratio test
            const float ratio_thresh = 0.7f; // HARCODED
            std::vector<cv::DMatch> good_matches;
            for (size_t i = 0; i < knn_matches.size(); i++)
            {
                if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
                {
                    good_matches.push_back(knn_matches[i][0]);
                }
            }

            //Get the points that match
            std::vector<cv::Point> matching_points;
            for(size_t i = 0; i< good_matches.size(); i++){
                matching_points.push_back(image_keypoints[good_matches[i].trainIdx].pt);
            }
            matching_points_list.push_back(matching_points);
        }
        return matching_points_list;
    }

    //Fonction pour calculer tous les descripteurs et keypoints des images de référence. Enregistre dans le fichier
    void create_reference_descriptors(std::string path){
        std::vector<cv::Mat> descriptors;
        std::vector<std::vector<cv::KeyPoint>> keypoints;

        std::vector<std::string> list_paths({"01_chooseSide_gman","01_chooseSide_bootlegger", "02_makeGrade_badge",
        "02_makeGrade_tommyGun","03_collecting_gman_white","03_collecting_bootlegger_white","04_shootout_gman_red",
        "04_shootout_bootlegger_red","05_cashSmash_axe_orange","05_cashSmash_dollar_orange"}); 
        //BootleggerGun.jpg is removed because it's the same as 02_makeGrade_tommyGun

        
        for(size_t i = 0; i< list_paths.size(); i++){
            std::pair<cv::Mat,std::vector<cv::KeyPoint>> descr_kp;
            std::string complete_path = kRefImagesPath + list_paths[i] + kImagesExt;
            cv::Mat image_for_calculation = cv::imread(complete_path);
            descr_kp = calculate_descriptors_and_kp(image_for_calculation);
            if(descr_kp.first.empty())
            {
                ROS_WARN("Ref image descriptors is empty for path : %s", list_paths[i].c_str());
            }

            descriptors.push_back(descr_kp.first);
            keypoints.push_back(descr_kp.second);
        }


        cv::FileStorage fsWrite(path, cv::FileStorage::WRITE);
        cv::write(fsWrite, "indexes", list_paths);
        for(int i = 0; i< descriptors.size();i++){
            cv::write(fsWrite, list_paths[i].substr(3), descriptors[i]);
            //Broken d'avoir un substr mais c'est parce que le nom des images est pas bien. 
            //Je ne peux pas commencer par un numérique
            // cv::write(fsWrite, "keypoints_" + std::to_string(i), keypoints[i]);
        }
        fsWrite.release();

    }
    
    


    private:
        cv::Mat output_image_;
        RangedParameter<int> objective_;
    };
}  // namespace proc_image_processing

#endif //PROC_IMAGE_PROCESSING_SIFT_MATCH_H