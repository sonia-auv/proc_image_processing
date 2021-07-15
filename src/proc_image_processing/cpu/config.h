/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>


#ifndef PROVIDER_VISION_UTILS_CONFIG_H_
#define PROVIDER_VISION_UTILS_CONFIG_H_

#include <sonia_common/config.h>
#include <string>

namespace proc_image_processing {
    const std::string kRosNodeName = "/proc_image_processing/";

    const std::string kProjectPath = sonia_common::kWorkspaceRoot + "/src/proc_image_processing/";

    const std::string kConfigPath = kProjectPath + "/config/";

    const std::string kFilterChainPath = kConfigPath + "/filterchain/";

    const std::string kFilterChainExt = ".yaml";

}; // namespace proc_image_processing

#endif // PROVIDER_VISION_UTILS_CONFIG_H_
