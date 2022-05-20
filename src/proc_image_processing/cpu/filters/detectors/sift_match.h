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

        explicit SiftMatch(const GlobalParameterHandler &globalParams)
                : Filter(globalParams),
                  objective_("Objective", 0, 0, 10, &parameters_, "0=G-Man Gate, 1=Bootlegger Gate, 2=G-Man Buoys, 3=Bootlegger Gun, 4=G-Man Bin, 5=Bootlegger Bin, 6=G-Man Torpidoe, 7=Bootlegger Torpidoe, 8=G-Man Axe, 9=Bootlegger Dollar") {
            setName("SiftMatch");
        }

        ~SiftMatch() override = default;

        void apply(cv::Mat &image) override {
            //image.copyTo(output_image_);

            // cv::Mat ref_image;
            // std::string path;
            // switch(objective_()) { // Code inutile
            //     case 0:
            //         path = kRefImagesPath + "01_chooseSide_gman" + kImagesExt;
            //         break;
            //     case 1:
            //         path = kRefImagesPath + "01_chooseSide_bootlegger" + kImagesExt;
            //         break;
            //     case 2:
            //         path = kRefImagesPath + "02_makeGrade_badge" + kImagesExt;
            //         break;
            //     case 3:
            //         path = kRefImagesPath + "02_makeGrade_tommyGun" + kImagesExt;
            //         break;
            //     case 4:
            //         path = kRefImagesPath + "03_collecting_gman_white" + kImagesExt;
            //         break;
            //     case 5:
            //         path = kRefImagesPath + "03_collecting_bootlegger_white" + kImagesExt;
            //         break;
            //     case 6:
            //         path = kRefImagesPath + "04_shootout_gman_red" + kImagesExt;
            //         break;
            //     case 7:
            //         path = kRefImagesPath + "04_shootout_bootlegger_red" + kImagesExt;
            //         break;
            //     case 8:
            //         path = kRefImagesPath + "05_cashSmash_axe_orange" + kImagesExt;
            //         break;
            //     case 9:
            //         path = kRefImagesPath + "05_cashSmash_dollar_orange" + kImagesExt;
            //         break;
            //     case 10:
            //         path = kRefImagesPath + "BootleggerGun.JPG";
            //         break;
            //     default:
            //         ROS_WARN("Wrong value in sift match filter");
            //         return;
            // }

            // ref_image = cv::imread(path);
            // if (image.empty()) {
            //     ROS_WARN("Provider vision image is empty");
            // }
            // if (ref_image.empty()) {
            //     ROS_WARN("Ref image is empty : %s",path.c_str());
            // }
            

            //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
            std::pair<cv::Mat,std::vector<cv::KeyPoint>> descriptors_keypoints = calculate_descriptors_and_kp(image);
            cv::Mat im_descriptors = descriptors_keypoints.first;
            std::vector<cv::KeyPoint> im_keypoints = descriptors_keypoints.second;


            //maintenant on va récupérer toutes les descriptors et keypoints des images de références
            std::vector<cv::Mat> ref_descriptors;
            std::vector<std::vector<cv::KeyPoint>> ref_keypoints; //pas utile les kp de référence
            std::string descr_path = kConfigPath + "/descriptors/Keypoints.yml";



    //Décommenter les lignes qui suivent pour fabriquer les descripteurs de référence et les enregistrer dans le dossier
    
            //Je peux commenter les 4 lignes qui suivent si j'arrive à récupérer les descripteurs
            // std::pair<std::vector<cv::Mat>,std::vector<std::vector<cv::KeyPoint>>> descr_kp_ref;
            // descr_kp_ref = calculate_descr_kp_references();
            // ref_descriptors = descr_kp_ref.first;
            // ref_keypoints = descr_kp_ref.second;

        //Enregistrement des données dans des fichiers.yml

            // cv::FileStorage fsWrite(descr_path, cv::FileStorage::WRITE);
            // for(int i = 0; i< ref_keypoints.size();i++){
            //     cv::write(fsWrite, "descriptors_" + std::to_string(i), ref_descriptors[i]);
            //     cv::write(fsWrite, "keypoints_" + std::to_string(i), ref_keypoints[i]);
            // }
            // fsWrite.release();


    //Les lignes pour lire depuis les descripteurs.
            //Il faudrait avoir ce code là ailleurs et garder ça en variable globale. 
            //ça ne sert à rien d'aller les chercher à chaque instant 



            cv::FileStorage fsRead;
            fsRead.open(descr_path, cv::FileStorage::READ);
            for(int i = 0; i< 10;i++){ // HARDCODED, nombre d'image de référence
                cv::Mat temp_descriptor;
                fsRead["descriptors_" + std::to_string(i)] >> temp_descriptor;   
                ref_descriptors.push_back(temp_descriptor);

                // std::vector<cv::KeyPoint> temp_kp;
                // fsRead["keypoints_" + std::to_string(i)] >> temp_kp;  
                // ref_keypoints.push_back(temp_kp);

            }
            fsRead.release();


            if(im_descriptors.empty()) // si je n'ai pas de descripteurs, ça ne sert à rien de faire des calculs
            {
                ROS_WARN("No descriptors on the image");
            }else{
                cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
                
                //-- Step 2: Matching descriptor vectors with a FLANN based matcher
                std::vector<std::vector<cv::Point>> matching_points_list;
                for(size_t i = 0; i< ref_descriptors.size(); i++){
                    cv::Mat ref_descriptor = ref_descriptors[i];
                    std::vector<std::vector<cv::DMatch>> knn_matches;
                    matcher->knnMatch( ref_descriptor, im_descriptors, knn_matches, 2 );

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
                        matching_points.push_back(im_keypoints[good_matches[i].trainIdx].pt);
                    }
                    matching_points_list.push_back(matching_points);
                }




                //Liste des couleurs
                std::vector<cv::Scalar> colors;
                colors.push_back(cv::Scalar(200,200,200)); //"01_chooseSide_gman"
                colors.push_back(cv::Scalar(205,00,00)); //"01_chooseSide_bootlegger"
                colors.push_back(cv::Scalar(0,0,200)); //"02_makeGrade_badge"
                colors.push_back(cv::Scalar(204,0,0)); //"02_makeGrade_tommyGun"
                colors.push_back(cv::Scalar(200,200,200)); //"03_collecting_gman_white"
                colors.push_back(cv::Scalar(200,200,200)); //"03_collecting_bootlegger_white"
                colors.push_back(cv::Scalar(200,200,200)); //"04_shootout_gman_red"
                colors.push_back(cv::Scalar(250,50,50)); //"04_shootout_bootlegger_red"
                colors.push_back(cv::Scalar(200,200,200)); //"05_cashSmash_axe_orange"
                colors.push_back(cv::Scalar(200,200,200)); //"05_cashSmash_dollar_orange"

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

        return std::make_pair(descriptors, keypoints);
    }

    private:
        cv::Mat output_image_;
        RangedParameter<int> objective_;
    };
}  // namespace proc_image_processing

#endif //PROC_IMAGE_PROCESSING_SIFT_MATCH_H