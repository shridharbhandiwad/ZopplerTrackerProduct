/**
 * @file IClustering.hpp
 * @brief Clustering interface for radar detections
 * @copyright Defense-grade radar tracking system
 */

#ifndef ZOPPLER_CLUSTERING_ICLUSTERING_HPP
#define ZOPPLER_CLUSTERING_ICLUSTERING_HPP

#include "../core/Detection.hpp"
#include "../core/Cluster.hpp"
#include <vector>
#include <memory>

namespace zoppler {

/**
 * @brief Abstract interface for clustering algorithms
 * 
 * Defines the contract for all clustering implementations.
 * Clustering groups related detections that likely originate
 * from the same physical target.
 */
class IClustering {
public:
    /**
     * @brief Virtual destructor
     */
    virtual ~IClustering() = default;
    
    /**
     * @brief Cluster detections into groups
     * 
     * @param detections Input radar detections
     * @return Vector of clusters, each containing indices into detections
     */
    virtual std::vector<Cluster> cluster(const std::vector<Detection>& detections) = 0;
    
    /**
     * @brief Get algorithm name
     */
    virtual std::string getName() const = 0;
    
    /**
     * @brief Clone the clustering algorithm
     */
    virtual std::unique_ptr<IClustering> clone() const = 0;
    
protected:
    IClustering() = default;
    
    // Prevent copying through base class
    IClustering(const IClustering&) = default;
    IClustering& operator=(const IClustering&) = default;
};

} // namespace zoppler

#endif // ZOPPLER_CLUSTERING_ICLUSTERING_HPP
