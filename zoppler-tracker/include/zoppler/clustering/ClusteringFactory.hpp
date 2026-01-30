/**
 * @file ClusteringFactory.hpp
 * @brief Factory for creating clustering algorithms
 * @copyright Defense-grade radar tracking system
 */

#ifndef ZOPPLER_CLUSTERING_FACTORY_HPP
#define ZOPPLER_CLUSTERING_FACTORY_HPP

#include "IClustering.hpp"
#include "../core/TrackerConfig.hpp"
#include <memory>
#include <string>

namespace zoppler {

/**
 * @brief Factory for creating clustering algorithm instances
 */
class ClusteringFactory {
public:
    /**
     * @brief Create clustering algorithm from configuration
     * 
     * @param config Tracker configuration
     * @return Unique pointer to clustering implementation
     */
    static std::unique_ptr<IClustering> create(const TrackerConfig& config);
    
    /**
     * @brief Create clustering algorithm by name
     * 
     * @param algorithmName Algorithm name (DBSCAN, CONTINUOUS_RANGE)
     * @return Unique pointer to clustering implementation with defaults
     */
    static std::unique_ptr<IClustering> create(const std::string& algorithmName);
    
    /**
     * @brief Check if algorithm name is valid
     */
    static bool isValidAlgorithm(const std::string& algorithmName);
    
    /**
     * @brief Get list of available algorithms
     */
    static std::vector<std::string> getAvailableAlgorithms();
    
private:
    ClusteringFactory() = delete;  // Static factory, no instantiation
};

} // namespace zoppler

#endif // ZOPPLER_CLUSTERING_FACTORY_HPP
