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
            //Size of the image
            //Width = 600
            //Height = 400
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
                    cv::circle(img_keypoints, matching_points[i], 3, colors[j], 2);
                }
            //Calcul de la moyenne 
                //Methode 1, moyenne brute sur toutes les données
                // cv::Point mean = mean_points(matching_points);

                //Methode 2, MeanShift
                // cv::Point mean_shift_pt = mean_shift(matching_points);

                // if(mean_shift_pt.x > -1){
                //     cv::circle(img_keypoints, mean_shift_pt, 4, colors[j], 6);
                // }

                //Methode 3, camshift. Permet d'avoir un rectangle à la fin. 
                std::pair<cv::Point,int>  mean_lgth = camshift(matching_points);
                cv::Point mean_camshift = mean_lgth.first;
                int lgth = mean_lgth.second;
                if(mean_camshift.x > 0){

                    cv::Point left_up = mean_camshift - cv::Point(lgth/2, lgth/2);
                    cv::Point right_bottom = mean_camshift + cv::Point(lgth/2, lgth/2);
                    cv::rectangle(img_keypoints, left_up, right_bottom, colors[j], 2);
                }
            }
            img_keypoints.copyTo(output_image_); // Just the points
            output_image_.copyTo(image);


            //Pour trouver le point moyen, je peux utiliser MEANSHIFT. 
            // Pour trouver un rectangle avec une orientation comme ils aimeraient bien, je peux utiliser CAMSHIFT
            
        }



    //Fonction pour calculer le Camshift
    std::pair<cv::Point,int> camshift(std::vector<cv::Point> point_list){
        int size = point_list.size();
        //Je ne cherche pas la moyenne si j'ai moins de 4 points parce que je considère que c'est du bruit
        if(size < 4){
            return std::make_pair(cv::Point(-1,-1), 0);
        }
        // ROS_INFO_STREAM(point_list.size());

        //Rectangle de vision
        int length = 200; // Taille initiale grande pour réduire l'impact de points trop loin
        cv::Point mean = point_list[0]; // starting value J'ESPERE QUE CA COPIE LA VALEUR ET CA CASSE PAS MA LISTE
        cv::Rect window;
        cv::Point old_mean;
        
        for(size_t i = 0; i< 5; i++){//Max 5 itérations
            window = cv::Rect(mean.x - length/2, mean.y - length/2, length, length);
            old_mean  = mean;
            //Trouver les points qui sont dans mon rectangle 
            std::vector<cv::Point> points_in_frame = points_inside_frame(window, point_list);
            //Calculer la moyenne des points dedans
            mean = mean_points(points_in_frame);
            length = 40 * sqrt(points_in_frame.size()); // JE NE SUIS PAS SUR DE LA FORMULE + HARDCODED
            if(cv::norm(mean-old_mean) < 5){ // Plus petit que 5 pixels en distance euclidienne
                break;
            }
        }
        return std::make_pair(mean,length);
    }


    //Fonction pour calculer le meanshift
    cv::Point mean_shift(std::vector<cv::Point> point_list){
        int size = point_list.size();
        //Je ne cherche pas la moyenne si j'ai moins de 4 points parce que je considère que c'est du bruit
        if(size < 4){
            return cv::Point(-1,-1);
        }

        //Rectangle de vision
        int w = 300;
        int h = 200;
        cv::Point mean = point_list[0]; // starting value J'ESPERE QUE CA COPIE LA VALEUR ET CA CASSE PAS MA LISTE
        cv::Rect window;
        cv::Point old_mean;
        
        for(size_t i = 0; i< 5; i++){//Max 5 itérations
            window = cv::Rect(mean.x - w/2, mean.y - h/2, w, h);
            old_mean  = mean;
            //Trouver les points qui sont dans mon rectangle 
            std::vector<cv::Point> points_in_frame = points_inside_frame(window, point_list);
            //Calculer la moyenne des points dedans
            mean = mean_points(points_in_frame);
            if(cv::norm(mean-old_mean) < 5){ // Plus petit que 5 pixels en distance euclidienne
                break;
            }
        }
        return mean;
    }

    //Fonction pour trouver les points qui sont dans un rectangle
    std::vector<cv::Point> points_inside_frame(cv::Rect window, std::vector<cv::Point> point_list){
         std::vector<cv::Point> output_list;
        for(size_t i = 0; i<point_list.size();i++){
            cv::Point aPoint = point_list[i];
            if(aPoint.x > window.x && aPoint.x < window.x + window.width && aPoint.y > window.y && aPoint.y < window.y + window.height){
                output_list.push_back(aPoint);
            }
        }
        return output_list;
    }

    //Fonction pour calculer la moyenne d'un ensemble de points
    cv::Point mean_points(std::vector<cv::Point> point_list){
        cv::Point output;
        int size = point_list.size();
        for(int i = 0; i< size; i++){
            output += point_list[i]/size;
        }
        return output;
    }


    //Fonction pour calculer les descripteurs et les keypoints pour une image 
    std::pair<cv::Mat,std::vector<cv::KeyPoint>> calculate_descriptors_and_kp(cv::Mat image){
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        detector->detectAndCompute(image,cv::noArray(),keypoints,descriptors);
        descriptors.convertTo(descriptors,CV_32F);
        return std::make_pair(descriptors,keypoints);
    }

    //Fonction pour calculer les descripteurs pour une image (redondant avec precedente)
    cv::Mat calculate_descriptors(cv::Mat image){
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        detector->detectAndCompute(image,cv::noArray(),keypoints,descriptors);
        descriptors.convertTo(descriptors,CV_32F);
        return descriptors;
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
            const float ratio_thresh = 0.7f; // HARCODED but good value
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

        std::vector<std::string> list_paths({"01_chooseSide_gman","01_chooseSide_bootlegger", "02_makeGrade_badge",
        "02_makeGrade_tommyGun","03_collecting_gman_white","03_collecting_bootlegger_white","04_shootout_gman_red",
        "04_shootout_bootlegger_red","05_cashSmash_axe_orange","05_cashSmash_dollar_orange"}); 
        
        for(size_t i = 0; i< list_paths.size(); i++){
            std::string complete_path = kRefImagesPath + list_paths[i] + kImagesExt;
            cv::Mat image_for_calculation = cv::imread(complete_path);

            //Code temporaire pour changer la taille
            cv::Mat image_rescale;
            int width = image_for_calculation.size().width;
            int height = image_for_calculation.size().height;
            
            cv::Mat descr, descr_output;
            descr_output = calculate_descriptors(image_for_calculation);

            if(descr_output.empty()){//Petite vérification
                ROS_WARN("Ref image descriptors is empty for path : %s", list_paths[i].c_str());
            }

            // ORB should be scale invariant but it doesn't look like it so I do it by myself and it's working way better
            for(int j = 2; j< 7; j += 2){
                resize(image_for_calculation, image_rescale, cv::Size(width/j, height/j));
                descr = calculate_descriptors(image_rescale);
                cv::vconcat(descr_output, descr, descr_output);
                

                // //Draw points and save images for comparison
                // std::pair<cv::Mat,std::vector<cv::KeyPoint>> descr_kp = calculate_descriptors_and_kp(image_rescale);
                // cv::Mat image_kp;
                // image_rescale.copyTo(image_kp);
                // std::vector<cv::KeyPoint> kp_list = descr_kp.second;
                // for(int k = 0; k< kp_list.size(); k++){
                //     cv::circle(image_kp, kp_list[k].pt, 3, cv::Scalar(0,255,0), 1);
                // }
                // cv::imwrite(kRefImagesPath +"descr/" + list_paths[i] + "_" + std::to_string(j) + kImagesExt, image_kp);
            }
            descriptors.push_back(descr_output);
        }


        cv::FileStorage fsWrite(path, cv::FileStorage::WRITE);
        cv::write(fsWrite, "indexes", list_paths);
        for(int i = 0; i< descriptors.size();i++){
            cv::write(fsWrite, list_paths[i].substr(3), descriptors[i]);
            //Broken d'avoir un substr mais c'est parce que le nom des images est pas bien. 
            //Je ne peux pas commencer par un numérique
        }
        fsWrite.release();
    }
    
    


    private:
        cv::Mat output_image_;
        RangedParameter<int> objective_;
    };
}  // namespace proc_image_processing

#endif //PROC_IMAGE_PROCESSING_SIFT_MATCH_H



//Brainstorming IA

// Pour améliorer le sift, il faudrait nettoyer le document des descriptors pour en avoir moins parce qu'ils se superposent surement;
// Il y a aussi le fait que différentes ilmages ont les mêmes descripteurs et c'est pas bien. 