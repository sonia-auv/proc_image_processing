// FACTORY_GENERATOR_CLASS_NAME=SiftCalculator

#ifndef PROC_IMAGE_PROCESSING_SIFT_CALCULATOR_H
#define PROC_IMAGE_PROCESSING_SIFT_CALCULATOR_H

#include <proc_image_processing/cpu/config.h>
#include <proc_image_processing/cpu/filters/filter.h>
#include "opencv2/features2d.hpp"

//CREATE THIS FILTER TO UPDATE THE DESCRIPTORS
//Creating it is enough, this filter doesn't do anything to the image

namespace proc_image_processing {

    class SiftCalculator : public Filter {
    public:

        using Ptr = std::shared_ptr<SiftMatch>;
        cv::Ptr<cv::ORB> detector = cv::ORB::create(1000, 1.2f, 8);


        explicit SiftCalculator(const GlobalParameterHandler &globalParams)
                : Filter(globalParams){
            setName("SiftCalculator");


            std::string complete_descr_path = kConfigPath + "/descriptors/Descriptors.yml";
            std::string descr_path = kConfigPath + "/descriptors/Descriptors_Pruned.yml"; // New with less descriptors
            

            //In both cases, the descriptors are saved.
            create_reference_descriptors(complete_descr_path); // Je peux le calculer à chaque fois mais je peux aussi le commenter pour gagner Quelques ms
            remove_ambigous_descriptors(complete_descr_path, descr_path);//Should reduce the false positive
                

        }

        ~SiftCalculator() override = default;


    
        std::pair<cv::Mat,std::vector<cv::KeyPoint>> calculate_descriptors_and_kp(cv::Mat image){
            std::vector<cv::KeyPoint> keypoints;
            cv::Mat descriptors;
            detector->detectAndCompute(image,cv::noArray(),keypoints,descriptors);
            descriptors.convertTo(descriptors,CV_32F);
            return std::make_pair(descriptors,keypoints);
        }

        cv::Mat calculate_descriptors(cv::Mat image){
            return calculate_descriptors_and_kp(image).first;
        }


            
        //Calculate and save the descriptors (and keypoints) of the reference images
        void create_reference_descriptors(std::string path){
            std::vector<cv::Mat> descriptors;
            std::vector<std::vector<cv::KeyPoint>> keypoints;

            std::vector<std::string> list_paths({"G-Man","Bootlegger", "Badge",
            "Gun","Barrel","Whiskey","Phone","Notepad","Axe","Dollar"}); //HARDCODED
            
            for(int i = 0; i< list_paths.size(); i++){
                std::string complete_path = kRefImagesPath + list_paths[i] + kImagesExt;
                cv::Mat image_for_calculation = cv::imread(complete_path);

                
                //Draw Keypoints on image for verification
                std::pair<cv::Mat,std::vector<cv::KeyPoint>> descr_kp = calculate_descriptors_and_kp(image_for_calculation);
                cv::Mat descr;
                descr = descr_kp.first;
                std::vector<cv::KeyPoint> kp = descr_kp.second;
                cv::Mat image_kp;
                cv::drawKeypoints(image_for_calculation,kp,image_kp,cv::Scalar(0,255,0),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
                cv::imwrite(kRefImagesPath +"descr/" + list_paths[i]  + kImagesExt, image_kp);
                
                if(descr.empty()){
                    ROS_WARN("Ref image descriptors is empty for path : %s", list_paths[i].c_str());
                }

                descriptors.push_back(descr);
                keypoints.push_back(kp);
            }

            //Save the descriptors in the file
            cv::FileStorage fsWrite(path, cv::FileStorage::WRITE);
            cv::write(fsWrite, "indexes", list_paths);
            for(int i = 0; i< descriptors.size();i++){
                cv::write(fsWrite, list_paths[i], descriptors[i]); // no suffix for back compatibility
                cv::write(fsWrite, list_paths[i]+"_kp", keypoints[i]);
            }
            fsWrite.release();
        }
        


        void remove_ambigous_descriptors(std::string source_path, std::string output_path){
            std::vector<cv::Mat> source_descriptors;

            //Read files
            cv::FileStorage fsRead;
            std::vector<std::string> list_paths;
            fsRead.open(source_path, cv::FileStorage::READ);
            fsRead["indexes"] >> list_paths;
            for(int i = 0; i< list_paths.size();i++){
                cv::Mat temp_descriptor;
                fsRead[list_paths[i]] >> temp_descriptor;   
                source_descriptors.push_back(temp_descriptor);
            }
            fsRead.release();

            //Matching
            cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
            std::vector<std::vector<int>> indexes_to_remove(source_descriptors.size(), {0}); // list of ref, then list of indexes

            //Finding ambigous descriptors
            for(int i = 0; i<source_descriptors.size();i++){ 
                for(int j = i+1; j<source_descriptors.size(); j++){
                    if((i==0 && j==6) || (i==1 && j==7) ||(i==1 && j==3) || (i==3 && j==6)){
                        continue;
                    }

                    std::vector<std::vector<cv::DMatch>> knn_matches;
                    matcher->knnMatch( source_descriptors[i], source_descriptors[j], knn_matches, 2 );

                    //-- Filter matches using the Lowe's ratio test
                    const float ratio_thresh = 0.7f;
                    for (int k = 0; k < knn_matches.size(); k++)
                    {
                        if (knn_matches[k][0].distance < ratio_thresh * knn_matches[k][1].distance)
                        {
                            indexes_to_remove[i].push_back(knn_matches[k][0].queryIdx);
                            indexes_to_remove[j].push_back(knn_matches[k][0].trainIdx);
                        }
                    }
                }
            }


            //Create a new descriptors list without the indexes
            std::vector<cv::Mat> new_descriptors;
            for(int i = 0; i<source_descriptors.size();i++){
                std::vector<int> indexes = indexes_to_remove[i];
                cv::Mat temp_descr;
                for(int j = 0; j<source_descriptors[i].size().height;j++){ // For each descriptor

                    if ( std::find(indexes.begin(), indexes.end(), j) != indexes.end() ){
                        //Item founded
                    }else{
                        // Item not founded: That's what we want
                        temp_descr.push_back(source_descriptors[i].row(j));
                    }
                }
                new_descriptors.push_back(temp_descr);
            }

            //Save the new descriptors
            cv::FileStorage fsWrite(output_path, cv::FileStorage::WRITE);
            cv::write(fsWrite, "indexes", list_paths);
            for(int i = 0; i< new_descriptors.size();i++){
                cv::write(fsWrite, list_paths[i], new_descriptors[i]);
            }
            fsWrite.release();
        }

        
        void apply(cv::Mat &image){}
    private:
    };
}  // namespace proc_image_processing

#endif //PROC_IMAGE_PROCESSING_SIFT_CALCULATOR_H

