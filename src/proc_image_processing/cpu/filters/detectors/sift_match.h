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


//New ideas to improve the efficiency
//1) If there is an overlap between multiple image, keep the one with the greater rectangle (bcz more points)
//Good idea but cache misère, it doesn't make it work better, it just look like it's better
//2) TF-IDF idea
//I compare the descriptors of all the reference images to find the descriptors that are specific to one image
// I don't need to delete the others but the specific ones need to have more weight. 
//This solution will reduce the number of different rectangle I will have. 
//It's a start //https://www.kaggle.com/code/meuge672/tf-idf-and-knn-in-people-wikipedia-dataset/notebook

//I think the algo I want is the opposite of "bigger cluster"
// In that function I want the point in the center of a cluster.
// I want to see the points that are far away from concurrent points. So I compare only to the points from others images
// I can compare each descriptors to all ther others descriptors but O(n²) for lots of descriptors is not good.
// How to give more power to big points?




//Does ORB work well with out-plane rotation?





namespace proc_image_processing {

    class SiftMatch : public Filter {
    public:
        using Ptr = std::shared_ptr<SiftMatch>;
        cv::Ptr<cv::ORB> detector = cv::ORB::create(750, 1.2f, 16);
        std::vector<cv::Mat> ref_descriptors;
        std::vector<cv::Point> previous_means; 
        //Note sur previous_mean. Lorsque je vais calculer mon cam_shift, je vais utiliser la valeur précédente de la moyenne.
        //Si je suis sur deux valeurs, je prendrai les deux premiers éléments. 
        //Sinon Je prends les 10 éléments.
        //J'espère qu'il ne va pas y avoir d'overlap au moment du changement de paramètre entre 2 et 0

        explicit SiftMatch(const GlobalParameterHandler &globalParams)
                : Filter(globalParams),
                  objective_("Objective", 0, 0, 10, &parameters_, "0=ALL, 1=ChooseSide, 2=MakeGrade, 3=Collecting, 4=Shoutout, 5=CashSmash"){
            setName("SiftMatch");
            


            //All the "create reference_descriptors" should be in another file!!!!!!!!!!!!!!
            //Je constate que le code ici n'est effectué qu'une seule fois donc je met mon ouverture de fichier ici
            // Ce n'est peut-être pas le bon endroit
            std::string descr_path = kConfigPath + "/descriptors/Descriptors.yml";
            // create_reference_descriptors(descr_path); // Je peux le calculer à chaque fois mais je peux aussi le commenter pour gagner Quelques ms

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

                //Fill the previous means with "0" value
                previous_means.push_back(cv::Point(-1,-1)); 
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

            if(im_keypoints.size() < 2){ // si je n'ai pas de descripteurs, ça ne sert à rien de faire des calculs
                // ROS_WARN("Less than 2 key points on the image");
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
                std::pair<cv::Point,int>  mean_lgth = camshift(matching_points, j);
                cv::Point mean_camshift = mean_lgth.first;
                int lgth = mean_lgth.second;
                if(mean_camshift.x > 0){ // Verify the point exist
                    cv::Point left_up = mean_camshift - cv::Point(lgth/2, lgth/2);
                    cv::Point right_bottom = mean_camshift + cv::Point(lgth/2, lgth/2);
                    cv::rectangle(img_keypoints, left_up, right_bottom, colors[j], 2);
                }
                previous_means[j] = mean_camshift; // Sometimes the good value, sometimes -1
            }
            img_keypoints.copyTo(output_image_); // Just the points
            output_image_.copyTo(image);
        }



    //Fonction pour calculer le Camshift
    std::pair<cv::Point,int> camshift(std::vector<cv::Point> point_list, int index){
        int size = point_list.size();
        //Je ne cherche pas la moyenne si j'ai moins de 4 points parce que je considère que c'est du bruit
        if(size < 4){
            return std::make_pair(cv::Point(-1,-1), 0);
        }

        //Rectangle de vision
        int length = 200; // Taille initiale grande pour réduire l'impact de points trop loin
        std::vector<cv::Point> list_of_means;
        cv::Rect window;
        
        cv::Point mean = previous_means[index]; 
        if(mean.x >= 0){   
            //Sol2 : Je fais un seul essai en prenant la valeur du mean sur l'image précédent comme base
            //(lorsque je change mon "objective()", il va y avoir un overlap mais ça devrait se régler en quelques images)

            cv::Point old_mean; // entre deux itérations
            for(int i = 0; i< 8; i++){//Max 8 itérations
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
            list_of_means.push_back(mean);
        }else{

            //Sol 1: Je fais plusieurs essais avec des valeurs aléatoires. En cas de non previous mean
            for(int attempt = 0;  attempt< 5; attempt++){ // Valeur arbitraire
                int random_index = std::rand() % point_list.size();
                mean = point_list[random_index]; // starting value
                cv::Point old_mean; // entre deux itérations
                for(int i = 0; i< 8; i++){//Max 8 itérations
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
                list_of_means.push_back(mean);
            }
        }
        //Mtn je veux récupérer le meilleur mean de ma liste    
        cv::Point output_mean = bigger_cluster_point(list_of_means);

        return std::make_pair(output_mean,length);
    }


    //Fonction pour calculer le meanshift, version de base du camshift
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

  


    //Find the point in a cluster (the more neighbours it has, the better)
    cv::Point bigger_cluster_point(std::vector<cv::Point> point_list){
        //Expensive function, ok for small amount of points
        std::vector<double> all_distances;
        for(int i = 0; i<point_list.size();i++){
            double total_dist;
            for(int j = 0; j<point_list.size();j++){
                if(i == j){continue;}
                total_dist += sqdist(point_list[i],point_list[j]);
            }
            all_distances.push_back(total_dist);
        }
        return point_list[index_of_min(all_distances)];
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


    //Calculate and save the descriptors of the reference images
    void create_reference_descriptors(std::string path){
        std::vector<cv::Mat> descriptors;

        std::vector<std::string> list_paths({"01_chooseSide_gman","01_chooseSide_bootlegger", "02_makeGrade_badge",
        "02_makeGrade_tommyGun","03_collecting_gman_white","03_collecting_bootlegger_white","04_shootout_gman_red",
        "04_shootout_bootlegger_red","05_cashSmash_axe_orange","05_cashSmash_dollar_orange"}); //HARDCODED


        for(size_t i = 0; i< list_paths.size(); i++){
            std::string complete_path = kRefImagesPath + list_paths[i] + kImagesExt;
            cv::Mat image_for_calculation = cv::imread(complete_path);

            cv::Mat descr;
            descr = calculate_descriptors(image_for_calculation);
            
            //Enregistre l'image pour vérification
            // std::pair<cv::Mat,std::vector<cv::KeyPoint>> descr_kp = calculate_descriptors_and_kp(image_for_calculation);
            // cv::Mat image_kp;
            // cv::drawKeypoints(image_for_calculation,descr_kp.second,image_kp,cv::Scalar(0,255,0),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            // cv::imwrite(kRefImagesPath +"descr/" + list_paths[i] + "_1" + kImagesExt, image_kp);
            
            if(descr.empty()){//Petite vérification
                ROS_WARN("Ref image descriptors is empty for path : %s", list_paths[i].c_str());
            }

            descriptors.push_back(descr);
        }

        //Save the descriptors in the file
        cv::FileStorage fsWrite(path, cv::FileStorage::WRITE);
        cv::write(fsWrite, "indexes", list_paths);
        for(int i = 0; i< descriptors.size();i++){
            cv::write(fsWrite, list_paths[i].substr(3), descriptors[i]);
            //Broken d'avoir un substr mais c'est parce que le nom des images est pas bien. 
            //Je ne peux pas commencer par un numérique
        }
        fsWrite.release();
    }
    


    //General function that should probably go in another file

     //Square distance between two points (no square root calculation)
    double sqdist(cv::Point a, cv::Point b){
        return (a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y);
    }

    //Return index of smaller double in a vector
    int index_of_min(std::vector<double> my_list){
        int output_idx = 0; // valeur par défaut
        double smaller = 1E+100;
        for(int i = 0; i<my_list.size(); i++){
            if(my_list[i] <= smaller){
                output_idx = i;
                smaller = my_list[i];
            }
        }
        return output_idx;
    }
        //Calculate the mean value of some points
    cv::Point mean_points(std::vector<cv::Point> point_list){
        cv::Point output;
        int size = point_list.size();
        for(int i = 0; i< size; i++){
            output += point_list[i]/size;
        }
        return output;
    }

      //Find the points inside a rectangle.
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

    


    private:
        cv::Mat output_image_;
        RangedParameter<int> objective_;
    };
}  // namespace proc_image_processing

#endif //PROC_IMAGE_PROCESSING_SIFT_MATCH_H



//Brainstorming IA

// Pour améliorer le sift, il faudrait nettoyer le document des descriptors pour en avoir moins parce qu'ils se superposent surement;
// Il y a aussi le fait que différentes ilmages ont les mêmes descripteurs et c'est pas bien. 