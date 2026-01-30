/**
 * @file IDataAssociation.hpp
 * @brief Data association interface
 * @copyright Defense-grade radar tracking system
 */

#ifndef ZOPPLER_ASSOCIATION_IDATAASSOCIATION_HPP
#define ZOPPLER_ASSOCIATION_IDATAASSOCIATION_HPP

#include "../core/Track.hpp"
#include "../core/Detection.hpp"
#include "../core/TrackerConfig.hpp"
#include <vector>
#include <memory>

namespace zoppler {

/**
 * @brief Abstract interface for data association algorithms
 * 
 * Defines the contract for all data association implementations.
 * Associates detections with existing tracks and identifies
 * unassociated detections for track initiation.
 */
class IDataAssociation {
public:
    /**
     * @brief Virtual destructor
     */
    virtual ~IDataAssociation() = default;
    
    /**
     * @brief Associate detections with tracks
     * 
     * @param tracks Current tracks (read-only for association)
     * @param detections Clustered detections (centroids)
     * @return Association result with track-detection mapping
     */
    virtual AssociationResult associate(
        const std::vector<Track>& tracks,
        const std::vector<Detection>& detections) = 0;
    
    /**
     * @brief Get algorithm name
     */
    virtual std::string getName() const = 0;
    
    /**
     * @brief Clone the algorithm
     */
    virtual std::unique_ptr<IDataAssociation> clone() const = 0;
    
    /**
     * @brief Set gating threshold
     */
    virtual void setGateThreshold(double threshold) { gateThreshold_ = threshold; }
    
    /**
     * @brief Get gating threshold
     */
    double getGateThreshold() const { return gateThreshold_; }
    
protected:
    IDataAssociation() : gateThreshold_(9.21) {}  // Default: 99% for 2 DOF
    
    /**
     * @brief Compute gating distance (Mahalanobis or Euclidean)
     */
    virtual double computeGatingDistance(const Track& track, 
                                          const Detection& detection) const;
    
    /**
     * @brief Check if detection passes gate for track
     */
    virtual bool passesGate(const Track& track, 
                            const Detection& detection,
                            double& distance) const;
    
    double gateThreshold_;
};

} // namespace zoppler

#endif // ZOPPLER_ASSOCIATION_IDATAASSOCIATION_HPP
