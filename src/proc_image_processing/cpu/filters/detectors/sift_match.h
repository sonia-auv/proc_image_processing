// FACTORY_GENERATOR_CLASS_NAME=SiftMatch

#ifndef PROC_IMAGE_PROCESSING_SIFT_MATCH_H
#define PROC_IMAGE_PROCESSING_SIFT_MATCH_H

#include <proc_image_processing/cpu/config.h>
#include <proc_image_processing/cpu/filters/filter.h>
#include <proc_image_processing/cpu/algorithm/performance_evaluator.h>
#include <proc_image_processing/cpu/server/target.h>
#include "opencv2/core.hpp"
#include "opencv2/opencv_modules.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include<string>


// Define colors

#define RED cv::Scalar(0,0,255)
#define BLUE cv::Scalar(255,0,0)
#define YELLOW cv::Scalar(0,255,255)
#define WHITE cv::Scalar(255,255,255)
#define GRAY cv::Scalar(100,100,100)
#define ORANGE cv::Scalar(0,128,255)
#define GREEN cv::Scalar(50,250,50)
#define PURPLE cv::Scalar(250,0,200)
#define CYAN cv::Scalar(250,250,0)

//Verify what the size of the bottom camera is. If it's smaller than 400x600, I can reduce the reference image for bottom.
// We can add RANSAC to verify the keypoints. With this algo we can delete all the false positive. 



//This code doesn't use any pointers as I don't know how they work.



namespace proc_image_processing {
    class SiftMatch : public Filter {
    public:
        using Ptr = std::shared_ptr<SiftMatch>;
        using vecPoint = std::vector<cv::Point>;
        cv::Ptr<cv::ORB> detector = cv::ORB::create(1000, 1.2f, 8);
        std::vector<cv::Mat> ref_descriptors;
        std::vector<std::vector<cv::KeyPoint>> ref_keypoints;
        vecPoint previous_means; 
        std::vector<std::string> class_names;
        //Note sur previous_mean. Lorsque je vais calculer mon cam_shift, je vais utiliser la valeur précédente de la moyenne.
        //Si je suis sur deux valeurs, je prendrai les deux premiers éléments. 
        //Sinon Je prends les 10 éléments.
        //J'espère qu'il ne va pas y avoir d'overlap au moment du changement de paramètre entre 2 et 0


        //Constructor
        explicit SiftMatch(const GlobalParameterHandler &globalParams)
                : Filter(globalParams),
                show_points_("Show_points", false, &parameters_),
                  objective_("Objective", 0, 0, 5, &parameters_, "0=ALL, 1=ChooseSide, 2=MakeGrade, 3=Collecting, 4=Shoutout,5=CashSmash"){
            setName("SiftMatch");
            
            //Reading descriptors from reference images
            // load_descriptors(kConfigPath + "/descriptors/Descriptors_Pruned.yml");
            load_descriptors(kConfigPath + "/descriptors/Descriptors.yml");
            //DEBUG
            ROS_INFO_STREAM("Il y a : " + std::to_string(ref_descriptors.size()) + " images de references");
        }


        //Destructor
        ~SiftMatch() override = default;



        //Principal function
        void apply(cv::Mat &image) override {
            //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
            std::pair<cv::Mat,std::vector<cv::KeyPoint>> descriptors_keypoints = calculate_descriptors_and_kp(image);
            cv::Mat im_descriptors = descriptors_keypoints.first;
            std::vector<cv::KeyPoint> im_keypoints = descriptors_keypoints.second;

            if(im_keypoints.size() < 2){ // If not enough descriptors, no need to make calculation
                // ROS_WARN("Less than 2 key points on the image");
                return; 
            }

            
            //-- Step 2: Matching descriptor vectors with a FLANN based matcher
            std::vector<vecPoint> matching_points_list;
            std::vector<cv::Mat> temp_ref_descriptors;


        //Images merge
        /*
            switch(objective_()) {  
                case 1: // Choose Side and Shoutout
                    temp_ref_descriptors.push_back(ref_descriptors[0]);
                    temp_ref_descriptors.push_back(ref_descriptors[1]);
                    temp_ref_descriptors[0].push_back(ref_descriptors[6]); // j'ajoute les descripteurs à l'autre image pour combiner les deux ref semblables
                    temp_ref_descriptors[1].push_back(ref_descriptors[7]);
                    temp_ref_descriptors[0].push_back(ref_descriptors[3]);//Ajout du fusil pour aider
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
                case 4: // Cash Shmash
                    temp_ref_descriptors.push_back(ref_descriptors[8]);
                    temp_ref_descriptors.push_back(ref_descriptors[9]);
                    matching_points_list = create_matcher_list(temp_ref_descriptors, im_descriptors, im_keypoints);
                    break;
                default:// Cas par défaut, on affiche tous les points 
                    temp_ref_descriptors.push_back(ref_descriptors[0]); //Gman
                    temp_ref_descriptors.push_back(ref_descriptors[1]); // Bootlegger
                    temp_ref_descriptors.push_back(ref_descriptors[2]); //Badge
                    temp_ref_descriptors[1].push_back(ref_descriptors[3]);//Ajout du fusil pour aider
                    temp_ref_descriptors.push_back(ref_descriptors[4]); //Collect gman
                    temp_ref_descriptors.push_back(ref_descriptors[5]); //Collect bootlegger
                    temp_ref_descriptors[0].push_back(ref_descriptors[6]); // gman
                    temp_ref_descriptors[1].push_back(ref_descriptors[7]); // Bootlegger
                    temp_ref_descriptors.push_back(ref_descriptors[8]); //Cash axe
                    temp_ref_descriptors.push_back(ref_descriptors[9]); // Cash dollar
                    matching_points_list = create_matcher_list(temp_ref_descriptors, im_descriptors, im_keypoints);
            }

            // Color List for first case
            std::vector<cv::Scalar> colors; //BGR
            colors.push_back(RED); //"gman" 
            colors.push_back(BLUE); //"bootlegger" BLUE
            colors.push_back(YELLOW); //"badge" YELLOW
            colors.push_back(WHITE); //"collecting_gman_white" WHITE
            colors.push_back(GRAY); //"collecting_bootlegger_white" GRAY
            colors.push_back(ORANGE); //"cashSmash_axe_orange" ORANGE
            colors.push_back(GREEN); //"cashSmash_dollar_orange" GREEN
*/

        //Color List for second case
        std::vector<cv::Scalar> colors; //BGR

        // Images not merge
            switch(objective_()) {  
                case 1: // Choose Side 
                    temp_ref_descriptors.push_back(ref_descriptors[0]);
                    temp_ref_descriptors.push_back(ref_descriptors[1]);
                    matching_points_list = create_matcher_list(temp_ref_descriptors, im_descriptors, im_keypoints);
                    colors.push_back(RED); //"gman" 
                    colors.push_back(BLUE); //"bootlegger" BLUE
                    break;
                case 2: // Make Grade
                    temp_ref_descriptors.push_back(ref_descriptors[2]);
                    temp_ref_descriptors.push_back(ref_descriptors[3]);
                    matching_points_list = create_matcher_list(temp_ref_descriptors, im_descriptors, im_keypoints);
                    colors.push_back(YELLOW); //"badge" YELLOW
                    colors.push_back(BLUE); // "fusil"
                    break;
                case 3: // Collecting
                    temp_ref_descriptors.push_back(ref_descriptors[4]);
                    temp_ref_descriptors.push_back(ref_descriptors[5]);
                    matching_points_list = create_matcher_list(temp_ref_descriptors, im_descriptors, im_keypoints);
                    colors.push_back(WHITE); //"collecting_gman_white" WHITE
                    colors.push_back(GRAY); //"collecting_bootlegger_white" GRAY
                    break;
                case 4: //Shoutout
                    temp_ref_descriptors.push_back(ref_descriptors[6]);
                    temp_ref_descriptors.push_back(ref_descriptors[7]);
                    matching_points_list = create_matcher_list(temp_ref_descriptors, im_descriptors, im_keypoints);
                    colors.push_back(RED); //"gman" 
                    colors.push_back(BLUE); //"bootlegger" BLUE
                    break;
                case 5: // Cash Shmash
                    temp_ref_descriptors.push_back(ref_descriptors[8]);
                    temp_ref_descriptors.push_back(ref_descriptors[9]);
                    matching_points_list = create_matcher_list(temp_ref_descriptors, im_descriptors, im_keypoints);
                    colors.push_back(ORANGE); //"cashSmash_axe_orange" ORANGE
                    colors.push_back(GREEN); //"cashSmash_dollar_orange" GREEN
                    break;
                default:// Default case, points for all images are shown
                    matching_points_list = create_matcher_list(ref_descriptors, im_descriptors, im_keypoints);
                    colors.push_back(RED); //"gman" 
                    colors.push_back(BLUE); //"bootlegger" BLUE
                    colors.push_back(YELLOW); //"badge" YELLOW
                    colors.push_back(BLUE); // "fusil"
                    colors.push_back(WHITE); //"collecting_gman_white" WHITE
                    colors.push_back(GRAY); //"collecting_bootlegger_white" GRAY
                    colors.push_back(RED); //"gman" 
                    colors.push_back(BLUE); //"bootlegger" BLUE
                    colors.push_back(ORANGE); //"cashSmash_axe_orange" ORANGE
                    colors.push_back(GREEN); //"cashSmash_dollar_orange" GREEN

            }

        

            
            



            //Draw the point on the image
            cv::Mat img_keypoints;
            image.copyTo(img_keypoints);
            std::vector<cv::Rect> rectangles;
            std::vector<int> rect_color_index;
            for(size_t j = 0; j< matching_points_list.size(); j++){
                vecPoint matching_points = matching_points_list[j];
                if(show_points_()){
                    
                    for(size_t i = 0; i< matching_points.size(); i++){
                        cv::circle(img_keypoints, matching_points[i], 3, colors[j], 2);
                    }
                }

             //Mean Calculation
                // I NEED A NEW ALGO TO HAVE A GOOD RECTANGLE

                //camshift. Gives a Mean point and a rectangle 
                std::pair<cv::Point,int>  mean_lgth = camshift(matching_points, j);
                cv::Point mean_camshift = mean_lgth.first;
                int lgth = mean_lgth.second;
                cv::Size s = cv::Size(lgth, lgth);
                if(mean_camshift.x > 0){ // Verify the point exist
                    rectangles.push_back(cv::Rect(mean_camshift - cv::Point(s/2), s));
                    rect_color_index.push_back(j);
                }
                previous_means[j] = mean_camshift; // Sometimes the good value, sometimes -1
            }
            
            // Filtrer les rectangles : Je supprime une zone si elle est plus petite qu'une autre et qu'il y a overlap
            for(int i = rectangles.size()-1; i>=0 ; i--){
                cv::Point middleI = (rectangles[i].tl() + rectangles[i].br())/2;
                for(int j = rectangles.size()-1; j>=0 ; j--){
                    if(i==j)continue;
                    if(rectangles[j].x == -1)continue; //Le rectangle a déjà été traité

                    cv::Point middleJ = (rectangles[j].tl() + rectangles[j].br())/2;
                    if(rectangles[j].contains(middleI) && rectangles[j].width > rectangles[i].width){
                        //Suppression J (je ne le retire pas mais je change sa valeur)
                        rectangles[i].x = -1;
                    }
                    if(rectangles[i].contains(middleJ) && rectangles[i].width > rectangles[j].width){
                        //Suppression I
                        rectangles[j].x = -1;
                    }
                }
            }
            

            // Dessiner les rectangles
            for(int i = 0; i<rectangles.size() ; i++){
                if(rectangles[i].x < 0){continue;}
                cv::Rect rectangle = rectangles[i];
                cv::rectangle(img_keypoints, rectangle, colors[rect_color_index[i]], 2); 
                //Point au centre du rectangle
                cv::Point rect_center = cv::Point(rectangle.x + rectangle.width/2, rectangle.y + rectangle.height/2);
                cv::circle(img_keypoints, rect_center,2, colors[rect_color_index[i]], 2);


                // Envoyer la target à ROS:
                // Construire un objet target
                // Remplir cet objet
                // Faire notify(target) pour qu'il soit gérer par les autres fonctions 
                Target target;

                // buildTarget
                
                //I CHANGED THE FORMULA HERE TO PUT THE CENTER, without testing
                target.setCenter(rectangle.x - image.size().width/2 + rectangle.width/2,  image.size().height/2 - rectangle.y - rectangle.height/2);
                target.setSize(rectangle.width, rectangle.height);

                int index;
                //This condition should be the reserve of the switch case in the beginning
                if(objective_() <= 0 || objective_() >= 6){
                    index = rect_color_index[i];
                }else{
                    index = (objective_()-1) * 2 + i;
                }
                std::string class_name = class_names[index];
                target.setHeader(class_name);

                notify(target);
            }

            img_keypoints.copyTo(output_image_); // Just the points
            output_image_.copyTo(image);

        }


        //Camshift Calculation
    std::pair<cv::Point,int> camshift(vecPoint point_list, int index){
        int size = point_list.size();

        // If not enough points, I don't consider the images is there.
        if(size < 4) return std::make_pair(cv::Point(-1,-1), 0);
    
        //NEW FORMULA TO CALCULATE LENGTH OF RECTANGLE WOULD BE GOOD. It's ok it's working

        int length = 200; // Initial Size for vision rectangle (in which we look for points)
        cv::Rect window;
        
        cv::Point mean = previous_means[index]; 
         if(mean.x > 0){   
            //Sol2 : I take the old mean for a first value
           
            cv::Point old_mean; // entre deux itérations
            for(int i = 0; i< 8; i++){//Max 8 itérations
                window = cv::Rect(mean.x - length/2, mean.y - length/2, length, length);
                old_mean  = mean;
                //Trouver les points qui sont dans mon rectangle 
                vecPoint points_in_frame = points_inside_frame(window, point_list);
                //Calculer la moyenne des points dedans
                mean = mean_points(points_in_frame);
                length = 40 * sqrt(points_in_frame.size()); // JE NE SUIS PAS SUR DE LA FORMULE + HARDCODED
                
                if(cv::norm(mean-old_mean) < 5 || length == 0 || mean.x == -1){ // Plus petit que 5 pixels en distance euclidienne
                    break;
                }
            }
        }
        if(mean.x <= 0 || length < 41){ // If I don't have previous mean/bad results, i try again from random points
        //if(true){  
            vecPoint list_of_means;
            std::vector<int> list_of_length;

            //Sol 1: Multiple try from random values
            for(int attempt = 0;  attempt< 5; attempt++){ // 5: arbitrary value
                int random_index = std::rand() % point_list.size();
                mean = point_list[random_index]; // starting value
                cv::Point old_mean;
                for(int i = 0; i< 8; i++){//Max 8 itérations
                    window = cv::Rect(mean.x - length/2, mean.y - length/2, length, length);
                    old_mean  = mean;
                    vecPoint points_in_frame = points_inside_frame(window, point_list);
                    mean = mean_points(points_in_frame);
                    length = 40 * sqrt(points_in_frame.size()); // HARDCODED
                     if(cv::norm(mean-old_mean) < 5 || length == 0 || mean.x == -1){ // Plus petit que 5 pixels en distance euclidienne
                        break;
                    }
                }
                if(length > 40){ // Bigger than smaller size. I really want to keep only the confident guesses : >= 4 points in the window
                    list_of_means.push_back(mean);
                    list_of_length.push_back(length);
                }
            }

            
            if (list_of_length.empty()){
                length = 0;
            }else{
                int maxElementIndex = std::max_element(list_of_length.begin(),list_of_length.end()) - list_of_length.begin();
                mean = list_of_means[maxElementIndex];
                length = list_of_length[maxElementIndex];
            }
        }

        if(length == 0){ // If camshift didn't gives a point
            return std::make_pair(cv::Point(-1,-1), 0);
        }
        //DEBUG
        //ROS_INFO_STREAM("Index " + std::to_string(index) + "; length: " + std::to_string(length) + " bcz " + std::to_string(length/40 * length/40) + " points");
        return std::make_pair(mean,length);
    }

      //-- Step 2: Matching descriptor vectors with a FLANN based matcher
    std::vector<vecPoint> create_matcher_list(std::vector<cv::Mat> reference_descriptors,cv::Mat image_descriptors,std::vector<cv::KeyPoint> image_keypoints){        
        cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);

        std::vector<vecPoint> matching_points_list;
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


            
            //When I get the point, I also make sure I don't have duplicates in the points (it happened sometimes)
            std::set<int> s;
            //Get the points that match
            vecPoint matching_points;
            for(cv::DMatch aMatch : good_matches){
                int thisIndex = aMatch.trainIdx;
                if(s.find(thisIndex) == s.end()){
                    s.insert(thisIndex);
                    matching_points.push_back(image_keypoints[thisIndex].pt);
                }
            }
            matching_points_list.push_back(matching_points);
        }
        return matching_points_list;
    }

    void load_descriptors(std::string descr_path){
        //Load descriptors and keypoints
        cv::FileStorage fsRead;
        std::vector<std::string> list_paths;
        fsRead.open(descr_path, cv::FileStorage::READ);
        fsRead["indexes"] >> list_paths;
        for(int i = 0; i< list_paths.size();i++){
            //Load descriptors
            cv::Mat temp_descriptor;
            fsRead[list_paths[i].substr(3)] >> temp_descriptor;   
            ref_descriptors.push_back(temp_descriptor);

            //Load keypoints
            std::vector<cv::KeyPoint> temp_kp;
            fsRead[list_paths[i].substr(3)+"_kp"] >> temp_kp;
            ref_keypoints.push_back(temp_kp);

            //Fill the previous means with "0" value
            previous_means.push_back(cv::Point(-1,-1)); 

            //Save the name to send a ros msg
            class_names.push_back(list_paths[i].substr(3));
        }
        fsRead.release();
    }

    
    std::pair<cv::Mat,std::vector<cv::KeyPoint>> calculate_descriptors_and_kp(cv::Mat image){
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        detector->detectAndCompute(image,cv::noArray(),keypoints,descriptors);
        descriptors.convertTo(descriptors,CV_32F);
        return std::make_pair(descriptors,keypoints);
    }


    //General function 

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
    cv::Point mean_points(vecPoint point_list){
        cv::Point output = cv::Point(0,0);
        int size = point_list.size();
        if(size==0)return cv::Point(-1,-1);
        for(cv::Point p : point_list)
            output += p/size;
        return output;
    }

      //Find the points inside a rectangle.
    vecPoint points_inside_frame(cv::Rect window, vecPoint point_list){
        vecPoint output_list;
        for(cv::Point aPoint : point_list){
            if(window.contains(aPoint))
                output_list.push_back(aPoint);
        }
        return output_list;
    }

    private:
        cv::Mat output_image_;
        Parameter<bool> show_points_;
        RangedParameter<int> objective_;
    };
}  // namespace proc_image_processing

#endif //PROC_IMAGE_PROCESSING_SIFT_MATCH_H
