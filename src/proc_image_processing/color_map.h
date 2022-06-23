#ifndef PROC_IMAGE_PROCESSING_COLOR_MAP_H_
#define PROC_IMAGE_PROCESSING_COLOR_MAP_H_

#include <stdlib.h>
#include <opencv2/opencv.hpp>

namespace proc_image_processing {
    // the color list follow the css color list, without black or white that could be difficultly visible on the telemetry.
    const std::map<std::string, cv::Scalar> COLOR_MAP_DEEP_LEARNING {
        {"RED", cv::Scalar(0xFF, 0x0, 0x0)},
        {"GREEN", cv::Scalar(0x0, 0xFF, 0x0)},
        {"BLUE", cv::Scalar(0x0, 0x0, 0xFF)},
        {"LAVENDER", cv::Scalar(0xE6, 0xE6, 0xFA)},
        {"MAROON", cv::Scalar(0x80, 0x0, 0x0)},
        {"MAGENTA", cv::Scalar(0xFF, 0x0, 0xFF)},
        {"OLIVE", cv::Scalar(0x80, 0x80, 0x0)},
        {"SALMON", cv::Scalar(0xFA, 0x80, 0x72)},
        {"SEASHELL", cv::Scalar(0xFF, 0xF5, 0xEE)},
        {"SILVER", cv::Scalar(0xC0, 0xC0, 0xC0)},
        {"YELLOW", cv::Scalar(0xFF, 0xFF, 0x0)},
        {"TEAL", cv::Scalar(0x0, 0x80, 0x80)},
        {"AQUAMARINE", cv::Scalar(0x7F, 0xFF, 0xD4)},
        {"ORANGE", cv::Scalar(0xFF, 0xA5, 0x00)}
    };
}

#endif
