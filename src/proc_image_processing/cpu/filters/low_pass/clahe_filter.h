// FACTORY_GENERATOR_CLASS_NAME=CLAHEFilter

#ifndef PROC_IMAGE_PROCESSING_CLAHE_FILTER_H
#define PROC_IMAGE_PROCESSING_CLAHE_FILTER_H

#include "proc_image_processing/cpu/filters/filter.h"
#include "opencv2/imgproc/imgproc.hpp"

namespace proc_image_processing {

    class CLAHEFilter : public Filter {
    public:
        using Ptr = std::shared_ptr<CLAHEFilter>;

        explicit CLAHEFilter(const GlobalParamHandler &globalParams)
                : Filter(globalParams),
                  enable_("Enable", false, &parameters_),
                  greyscale_("Greyscale", true, &parameters_),
                  clip_limit_("ClipLimit", 40.0, &parameters_, "Threshold for contrast limiting."),
                  tile_width_("TileWidth", 8, &parameters_, "Width of grid for histogram equalization."),
                  tile_height_("TileHeight", 8, &parameters_, "Height of grid for histogram equalization.") {
            setName("CLAHEFilter");
        }

        ~CLAHEFilter() override = default;

        void apply(cv::Mat &image) override {
            if (enable_()) {
                auto width = image.cols < tile_width_() ? image.cols : tile_width_();
                auto height = image.rows < tile_height_() ? image.rows : tile_height_();
                auto clahe = cv::createCLAHE(clip_limit_(), cv::Size(width, height));

                if (greyscale_()) {
                    cv::Mat greyscale;
                    cv::cvtColor(image, greyscale, CV_BGR2GRAY);
                    clahe->apply(greyscale, image);
                } else {
                    // https://stackoverflow.com/questions/24341114/simple-illumination-correction-in-images-opencv-c
                    cv::Mat lab;
                    cv::cvtColor(image, lab, CV_BGR2Lab);

                    // Extract the L channel
                    std::vector<cv::Mat> lab_planes(3);
                    cv::split(lab, lab_planes);  // now we have the L image in lab_planes[0]

                    cv::Mat dst;
                    clahe->apply(lab_planes[0], dst);

                    // Merge the color planes back into an Lab image
                    dst.copyTo(lab_planes[0]);
                    cv::merge(lab_planes, lab);

                    cv::cvtColor(lab, image, CV_Lab2BGR);
                }
            }
        }

    private:
        Parameter<bool> enable_, greyscale_;
        Parameter<double> clip_limit_;
        Parameter<int> tile_width_, tile_height_;
    };

}  // namespace proc_image_processing

#endif //PROC_IMAGE_PROCESSING_CLAHE_FILTER_H
