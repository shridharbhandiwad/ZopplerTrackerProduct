/**
 * @file ContinuousRangeClustering.hpp
 * @brief Range-bin based clustering for radar detections
 * @copyright Defense-grade radar tracking system
 */

#ifndef ZOPPLER_CLUSTERING_CONTINUOUSRANGE_HPP
#define ZOPPLER_CLUSTERING_CONTINUOUSRANGE_HPP

#include "IClustering.hpp"
#include "../core/TrackerConfig.hpp"

namespace zoppler {

/**
 * @brief Continuous Range Clustering implementation
 * 
 * Clusters detections based on range bins with angular gating.
 * Efficient for traditional surveillance radars with range cells.
 * Uses sequential gating in range, then azimuth, then elevation.
 */
class ContinuousRangeClustering : public IClustering {
public:
    /**
     * @brief Construct with parameters
     * 
     * @param rangeGate Range gate size (meters)
     * @param azimuthGate Azimuth gate size (radians)
     * @param elevationGate Elevation gate size (radians)
     * @param dopplerGate Doppler gate size (m/s)
     */
    ContinuousRangeClustering(double rangeGate = 5.0,
                               double azimuthGate = 0.02,
                               double elevationGate = 0.05,
                               double dopplerGate = 2.0);
    
    /**
     * @brief Construct from config
     */
    explicit ContinuousRangeClustering(const TrackerConfig& config);
    
    /**
     * @brief Destructor
     */
    ~ContinuousRangeClustering() override = default;
    
    /**
     * @brief Cluster detections using range-bin approach
     */
    std::vector<Cluster> cluster(const std::vector<Detection>& detections) override;
    
    /**
     * @brief Get algorithm name
     */
    std::string getName() const override { return "CONTINUOUS_RANGE"; }
    
    /**
     * @brief Clone the algorithm
     */
    std::unique_ptr<IClustering> clone() const override;
    
    // Configuration setters
    void setRangeGate(double gate) { rangeGate_ = gate; }
    void setAzimuthGate(double gate) { azimuthGate_ = gate; }
    void setElevationGate(double gate) { elevationGate_ = gate; }
    void setDopplerGate(double gate) { dopplerGate_ = gate; }
    void setUse3D(bool use3D) { use3D_ = use3D; }
    void setUseDoppler(bool useDop) { useDoppler_ = useDop; }
    
private:
    /**
     * @brief Check if two detections are in same cluster gate
     */
    bool inSameGate(const Detection& a, const Detection& b) const;
    
    /**
     * @brief Find cluster for detection using union-find
     */
    int findRoot(std::vector<int>& parent, int idx) const;
    
    /**
     * @brief Union two clusters
     */
    void unionClusters(std::vector<int>& parent, std::vector<int>& rank,
                       int a, int b) const;
    
    double rangeGate_;      // Range gate size (meters)
    double azimuthGate_;    // Azimuth gate size (radians)
    double elevationGate_;  // Elevation gate size (radians)
    double dopplerGate_;    // Doppler gate size (m/s)
    bool use3D_;            // Use elevation gating
    bool useDoppler_;       // Use Doppler gating
};

} // namespace zoppler

#endif // ZOPPLER_CLUSTERING_CONTINUOUSRANGE_HPP
