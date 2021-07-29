#ifndef PROC_IMAGE_PROCESSING_UTILS_CONFIG_H_
#define PROC_IMAGE_PROCESSING_UTILS_CONFIG_H_

#include <sonia_common/config.h>
#include <string>

namespace proc_image_processing {
    const std::string kRosNodeName = "/" + std::string(std::getenv("NODE_NAME"));

    const std::string kProjectPath = std::string(std::getenv("NODE_PATH"));

    const std::string kConfigPath = kProjectPath + "/config";

    const std::string kFilterChainPath = kConfigPath + "/filterchain";

    const std::string kFilterChainExt = ".yaml";

}; // namespace proc_image_processing

#endif // PROC_IMAGE_PROCESSING_UTILS_CONFIG_H_