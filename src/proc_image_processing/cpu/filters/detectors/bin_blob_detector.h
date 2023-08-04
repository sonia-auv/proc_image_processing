// FACTORY_GENERATOR_CLASS_NAME=BinBlobDetector

#pragma once

#include <proc_image_processing/cpu/filters/filter.h>
#include <memory>
#include <sonia_common/FilterchainTarget.h>



namespace proc_image_processing {

    class BinBlobDetector : public Filter {
    public:
        using Ptr = std::shared_ptr<BinBlobDetector>;

        explicit BinBlobDetector(const GlobalParameterHandler &globalParams, ros::NodeHandlePtr nhp)
                : Filter(globalParams, nhp),
              m_area_filter("Enable area filter", false, &parameters_,"Choose to filter by blob area or not"),
              m_area_min("Min Area", 0, 0, INT_MAX, &parameters_,"Area of the blob"),
              m_area_max("Max Area", INT_MAX, 0, INT_MAX, &parameters_,"Area of the blob"),
              m_circ_min("Min Circularity", 0, 0, 1, &parameters_,"Shape of the blob (Circle=1, Square=0.785, Triangle=0.605)"),
              m_circ_max("Max Circularity", 1, 0, 1, &parameters_,"Shape of the blob (Circle=1, Square=0.785, Triangle=0.605)"),
              m_iner_min("Min Inertia", 0, 0, 1, &parameters_,"Elongation of the blob (Circle=1, Line =0)"),
              m_iner_max("Max Inertia", 1, 0, 1, &parameters_,"Elongation of the blob (Circle=1, Line =0)"),
              m_conv_min("Min Convexity", 0, 0, 1, &parameters_,"Area of the Blob / Area of it’s convex hull"),
              m_conv_max("Max Convexity", 1, 0, 1, &parameters_,"Area of the Blob / Area of it’s convex hull")
        {
            setName("BinBlobDetector");
            m_bin_pub = this->nhp_->advertise<sonia_common::FilterchainTarget>("/proc_image_processing/bin_target", 100);
        }


        ~BinBlobDetector() override = default;

        void apply(cv::Mat &image) override {
            cv::SimpleBlobDetector::Params params;

            params.filterByArea=m_area_filter();
            params.minArea=m_area_min();
            params.maxArea=m_area_max();

            params.filterByCircularity=true;
            params.minCircularity=m_circ_min();
            params.maxCircularity=m_circ_max();
            
            params.filterByInertia=true;
            params.minInertiaRatio=m_iner_min();
            params.maxInertiaRatio=m_iner_max();
            
            params.filterByConvexity=true;
            params.minConvexity=m_conv_min();
            params.maxConvexity=m_conv_max();


            cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

            std::vector<cv::KeyPoint> blobs;
            detector->detect(image, blobs);

            std::sort(
                    blobs.begin(),
                    blobs.end(),
                    [](cv::KeyPoint a, cv::KeyPoint b) -> bool {
                        return a.size > b.size;
                    }
            );

            if (blobs.size() >=2 ) {
                cv::KeyPoint object1 = blobs[0];
                cv::KeyPoint object2 = blobs[1];
                cv::Point center = (object1.pt + object2.pt) / 2;

                cv::circle(image, center, 3, CV_RGB(0, 255, 0), CV_FILLED);
                
                geometry_msgs::Pose2D objet;
                sonia_common::FilterchainTarget target;
                cv::Point distance = object1.pt - object2.pt;

                objet.x = center.x;
                objet.y = center.y;
                target.obj_ctr = objet;
                target.obj_size = sqrt(distance.x^2+distance.y^2);
                m_bin_pub.publish(target);
            }            
        }

    private:
       
        Parameter<bool> m_area_filter;
        RangedParameter<int> m_area_min;
        RangedParameter<int> m_area_max;
        RangedParameter<double> m_circ_min;
        RangedParameter<double> m_circ_max;
        RangedParameter<double> m_iner_min;
        RangedParameter<double> m_iner_max;
        RangedParameter<double> m_conv_min;
        RangedParameter<double> m_conv_max;

        ros::Publisher m_bin_pub;
    };

}  // namespace proc_image_processing