/**
 * @file ClusteringFactory.cpp
 * @brief Clustering factory implementation
 * @copyright Defense-grade radar tracking system
 */

#include "zoppler/clustering/ClusteringFactory.hpp"
#include "zoppler/clustering/DBSCANClustering.hpp"
#include "zoppler/clustering/ContinuousRangeClustering.hpp"
#include <algorithm>
#include <stdexcept>

namespace zoppler {

std::unique_ptr<IClustering> ClusteringFactory::create(const TrackerConfig& config) {
    ClusteringAlgorithm algo = config.getClusteringAlgorithm();
    
    switch (algo) {
        case ClusteringAlgorithm::DBSCAN:
            return std::make_unique<DBSCANClustering>(config);
            
        case ClusteringAlgorithm::CONTINUOUS_RANGE:
            return std::make_unique<ContinuousRangeClustering>(config);
            
        default:
            return std::make_unique<DBSCANClustering>(config);
    }
}

std::unique_ptr<IClustering> ClusteringFactory::create(const std::string& algorithmName) {
    std::string name = algorithmName;
    std::transform(name.begin(), name.end(), name.begin(), ::toupper);
    
    if (name == "DBSCAN") {
        return std::make_unique<DBSCANClustering>();
    } else if (name == "CONTINUOUS_RANGE" || name == "RANGE_BINS" || name == "CONTINUOUS") {
        return std::make_unique<ContinuousRangeClustering>();
    }
    
    throw std::invalid_argument("Unknown clustering algorithm: " + algorithmName);
}

bool ClusteringFactory::isValidAlgorithm(const std::string& algorithmName) {
    std::string name = algorithmName;
    std::transform(name.begin(), name.end(), name.begin(), ::toupper);
    
    return name == "DBSCAN" || 
           name == "CONTINUOUS_RANGE" || 
           name == "RANGE_BINS" ||
           name == "CONTINUOUS";
}

std::vector<std::string> ClusteringFactory::getAvailableAlgorithms() {
    return {"DBSCAN", "CONTINUOUS_RANGE"};
}

} // namespace zoppler
