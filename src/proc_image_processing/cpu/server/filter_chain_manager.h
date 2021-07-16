/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>


#ifndef PROC_IMAGE_PROCESSING_SERVER_FILTER_CHAIN_MANAGER_H_
#define PROC_IMAGE_PROCESSING_SERVER_FILTER_CHAIN_MANAGER_H_

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
     * to each FilterChain in addition to being charged of opening and closing them.
     * Also offers ROS services to allow filterchain managing.
     */
    class FilterChainManager {
    public:
        using Ptr = std::shared_ptr<FilterChainManager>;

        static const std::string FILTER_CHAIN_MANAGER_TAG;

        FilterChainManager();

        ~FilterChainManager();

        /**
         * Get all available filterchains on the system.
         *
         * \return vector<std::string>
         */
        static std::vector<std::string> getFilterChainsNames();

        /**
         * Get the list of all running filterchains
         */
        const std::vector<FilterChain::Ptr> &getRunningFilterChains() const;

        /**
         * If the filterchain exists, this method will create
         * an ins
         *
         * \param filterchainName std::string
         * \return FilterChain*
         */
        FilterChain::Ptr createFilterChain(const std::string &filterchainName);

        /**
         * Pass through the list of all the filterchains and instanciate them
         *
         * \return The list of all filter chains
         */
        const std::vector<FilterChain::Ptr> &createAllFilterChains();

        /**
         * Get all available filterchains on the system.
         *
         * \param filterchainName std::string
         * \return FilterChain*
         */
        void stopFilterChain(const FilterChain::Ptr &filter_chain);

        /**
         * If the does not filter_chain exists, create it.
         *
         * \param filter_chain The name of the filter_chain to create.
         * \return Either if the filter_chain was added or not.
         */
        void addFilterChain(const std::string &filter_chain);

        /**
         * If the filter_chain exists, delete it.
         *
         * \param filter_chain The name of the filter_chain to delete.
         * \return Either if the filter_chain was delete or not.
         */
        void deleteFilterChain(const std::string &filter_chain) const;

        /**
         * Check if a filter_chain exist or not.
         *
         * This will check on the list of the available filter_chain provided by
         * GetAvailableFilterchain if the file exists or not.
         *
         * \param filter_chain The name of the filter_chain to check.
         * \return Either if the file exist or not
         */
        static bool filterChainExists(const std::string &filter_chain);

        /**
         * With the constants defining the config directory path and the
         * extension, return the true path of a filter_chain.
         */
        static std::string getFilterChainPath(const std::string &filter_chain);

    private:
        /**
         * List of current instances of filterchains
         */
        std::vector<FilterChain::Ptr> running_filter_chains_;

    };

}  // namespace proc_image_processing

#endif  // PROC_IMAGE_PROCESSING_SERVER_FILTER_CHAIN_MANAGER_H_
