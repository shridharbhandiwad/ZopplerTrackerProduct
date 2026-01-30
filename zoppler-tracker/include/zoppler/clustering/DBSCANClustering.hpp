/**
 * @file DBSCANClustering.hpp
 * @brief DBSCAN clustering algorithm for radar detections
 * @copyright Defense-grade radar tracking system
 */

#ifndef ZOPPLER_CLUSTERING_DBSCAN_HPP
#define ZOPPLER_CLUSTERING_DBSCAN_HPP

#include "IClustering.hpp"
#include "../core/TrackerConfig.hpp"

namespace zoppler {

/**
 * @brief DBSCAN (Density-Based Spatial Clustering) implementation
 * 
 * Well-suited for clustering radar detections with arbitrary shapes.
 * Robust to noise and can identify outliers as noise points.
 */
class DBSCANClustering : public IClustering {
public:
    /**
     * @brief Construct DBSCAN with parameters
     * 
     * @param epsilon Neighborhood radius
     * @param minPoints Minimum points to form a cluster
     * @param use3D Whether to use 3D distance
     * @param useDoppler Whether to include Doppler in distance
     */
    DBSCANClustering(double epsilon = 10.0, int minPoints = 2,
                     bool use3D = true, bool useDoppler = false);
    
    /**
     * @brief Construct from config
     */
    explicit DBSCANClustering(const TrackerConfig& config);
    
    /**
     * @brief Destructor
     */
    ~DBSCANClustering() override = default;
    
    /**
     * @brief Cluster detections using DBSCAN
     */
    std::vector<Cluster> cluster(const std::vector<Detection>& detections) override;
    
    /**
     * @brief Get algorithm name
     */
    std::string getName() const override { return "DBSCAN"; }
    
    /**
     * @brief Clone the algorithm
     */
    std::unique_ptr<IClustering> clone() const override;
    
    // Configuration setters
    void setEpsilon(double eps) { epsilon_ = eps; }
    void setMinPoints(int minPts) { minPoints_ = minPts; }
    void setUse3D(bool use3D) { use3D_ = use3D; }
    void setUseDoppler(bool useDop) { useDoppler_ = useDop; }
    void setDopplerWeight(double weight) { dopplerWeight_ = weight; }
    
private:
    /**
     * @brief Compute distance between two detections
     */
    double computeDistance(const Detection& a, const Detection& b) const;
    
    /**
     * @brief Find neighbors within epsilon
     */
    std::vector<size_t> findNeighbors(const std::vector<Detection>& detections,
                                       size_t pointIdx) const;
    
    /**
     * @brief Expand cluster from seed point
     */
    void expandCluster(const std::vector<Detection>& detections,
                       size_t pointIdx,
                       std::vector<size_t>& neighbors,
                       int clusterId,
                       std::vector<int>& labels) const;
    
    double epsilon_;        // Neighborhood radius
    int minPoints_;         // Minimum points for core point
    bool use3D_;            // Use 3D distance calculation
    bool useDoppler_;       // Include Doppler in distance
    double dopplerWeight_;  // Doppler velocity weight factor
    
    // Precomputed distance matrix (optional, for performance)
    mutable std::vector<std::vector<double>> distanceMatrix_;
    mutable bool useDistanceMatrix_ = false;
};

} // namespace zoppler

#endif // ZOPPLER_CLUSTERING_DBSCAN_HPP
