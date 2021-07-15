/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>


#ifndef PROVIDER_VISION_SERVER_FILTERCHAIN_MANAGER_H_
#define PROVIDER_VISION_SERVER_FILTERCHAIN_MANAGER_H_

#include <sonia_common/ros/service_server_manager.h>
#include <proc_image_processing/cpu/filters/filter.h>
#include <functional>
#include <string>
#include <vector>
#include "proc_image_processing/cpu/config.h"
#include "filter_chain.h"

namespace proc_image_processing {

  /**
   * This class is the core module that stores and manages every FilterChains
   * within the vision server. Its job is to keep track of all changes occuring
   * to each Filterchain in addition to being charged of opening and closing them.
   * Also offers ROS services to allow filterchain managing.
   */
  class FilterchainManager {
  public:
    using Ptr = std::shared_ptr<FilterchainManager>;

    static const std::string FILTERCHAIN_MANAGER_TAG;

    FilterchainManager();

    ~FilterchainManager();

    /**
     * Get all available filterchains on the system.
     *
     * \param execution_name : string
     * \param filterchain : string
     * \param filter : string
     * \return vector<std::string>
     */
    std::vector<std::string> GetAllFilterchainName() const;

    /**
     * Get the list of all running filterchains
     */
    const std::vector<Filterchain::Ptr>& GetRunningFilterchains() const;

    /**
     * If the filterchain exists, this method will create
     * an ins
     *
     * \param filterchainName std::string
     * \return Filterchain*
     */
    Filterchain::Ptr InstanciateFilterchain(const std::string& filterchainName);

    /**
     * Pass through the list of all the filterchains and instanciate them
     *
     * \return The list of all filter chains
     */
    const std::vector<Filterchain::Ptr>& InstanciateAllFilterchains();

    /**
     * Get all available filterchains on the system.
     *
     * \param executionName std::string
     * \param filterchainName std::string
     * \return Filterchain*
     */
    void StopFilterchain(const Filterchain::Ptr& filterchain);

    /**
     * If the does not filterchain exists, create it.
     *
     * \param filterchain The name of the filterchain to create.
     * \return Either if the filterchain was added or not.
     */
    void CreateFilterchain(const std::string& filterchain);

    /**
     * If the filterchain exists, delete it.
     *
     * \param filterchain The name of the filterchain to delete.
     * \return Either if the filterchain was delete or not.
     */
    void EraseFilterchain(const std::string& filterchain);

    /**
     * Check if a filterchain exist or not.
     *
     * This will check on the list of the available filterchain provided by
     * GetAvailableFilterchain if the file exists or not.
     *
     * \param filterchain The name of the filterchain to check.
     * \return Either if the file exist or not
     */
    bool FilterchainExists(const std::string& filterchain);

    /**
     * With the constants defining the config directory path and the
     * extension, return the true path of a filterchain.
     */
    std::string GetFilterchainPath(const std::string& filterchain) const;

  private:
    /**
     * List of current instances of filterchains
     */
    std::vector<Filterchain::Ptr> running_filterchains_;

  };

}  // namespace proc_image_processing

#endif  // PROVIDER_VISION_SERVER_FILTERCHAIN_MANAGER_H_