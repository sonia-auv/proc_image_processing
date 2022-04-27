#ifndef PROC_IMAGE_PROCESSING_UTILS_CONFIG_H_
#define PROC_IMAGE_PROCESSING_UTILS_CONFIG_H_

#include <sonia_common/config.h>
#include <string>

namespace proc_image_processing {
    const std::string kRosNodeName = "/" + std::string(
        std::getenv("NODE_NAME") ?
        std::string("/") + std::getenv("NODE_NAME") :
        "/proc_image_processing"
    );

    const std::string kProjectPath = std::string(
        std::getenv("NODE_PATH") ?
        std::string("/") + std::getenv("NODE_PATH") :
        std::getenv("HOME") + std::string("/ros_sonia_ws/src") + kRosNodeName
    );

    const std::string kConfigPath = kProjectPath + std::string(
            std::getenv("NODE_CONFIG_PATH") ?
            std::string("/") + std::getenv("NODE_CONFIG_PATH") :
            "/config"
    );

    const std::string kFilterChainPath = kConfigPath + "/filterchain";

    const std::string kRefImagesPath = kConfigPath + "/ref_images";

    const std::string kFilterChainExt = ".yaml";
    
    const std::string kImagesExt = ".JPG";
}; // namespace proc_image_processing

#endif // PROC_IMAGE_PROCESSING_UTILS_CONFIG_H_