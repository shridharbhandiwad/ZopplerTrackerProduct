/**
 * @file GNNAssociation.hpp
 * @brief Global Nearest Neighbor data association
 * @copyright Defense-grade radar tracking system
 */

#ifndef ZOPPLER_ASSOCIATION_GNN_HPP
#define ZOPPLER_ASSOCIATION_GNN_HPP

#include "IDataAssociation.hpp"

namespace zoppler {

/**
 * @brief Global Nearest Neighbor (GNN) Association
 * 
 * Uses Hungarian algorithm to find optimal one-to-one assignment
 * that minimizes total association cost. Efficient and deterministic.
 * 
 * Best for: Low clutter, well-separated targets
 */
class GNNAssociation : public IDataAssociation {
public:
    /**
     * @brief Construct GNN with parameters
     * 
     * @param gateThreshold Chi-squared gate threshold
     * @param maxDistance Maximum association distance (meters)
     */
    GNNAssociation(double gateThreshold = 9.21, double maxDistance = 100.0);
    
    /**
     * @brief Construct from config
     */
    explicit GNNAssociation(const TrackerConfig& config);
    
    /**
     * @brief Destructor
     */
    ~GNNAssociation() override = default;
    
    /**
     * @brief Associate using GNN algorithm
     */
    AssociationResult associate(
        const std::vector<Track>& tracks,
        const std::vector<Detection>& detections) override;
    
    /**
     * @brief Get algorithm name
     */
    std::string getName() const override { return "GNN"; }
    
    /**
     * @brief Clone the algorithm
     */
    std::unique_ptr<IDataAssociation> clone() const override;
    
    // Configuration
    void setMaxDistance(double dist) { maxDistance_ = dist; }
    void setUseStatisticalDistance(bool use) { useStatisticalDistance_ = use; }
    
private:
    /**
     * @brief Build cost matrix for Hungarian algorithm
     */
    std::vector<std::vector<double>> buildCostMatrix(
        const std::vector<Track>& tracks,
        const std::vector<Detection>& detections) const;
    
    double maxDistance_;
    bool useStatisticalDistance_;  // Use Mahalanobis vs Euclidean
};

} // namespace zoppler

#endif // ZOPPLER_ASSOCIATION_GNN_HPP
