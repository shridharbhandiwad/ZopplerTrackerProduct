/**
 * @file GNNAssociation.cpp
 * @brief Global Nearest Neighbor implementation
 * @copyright Defense-grade radar tracking system
 */

#include "zoppler/association/GNNAssociation.hpp"
#include "zoppler/tracking/ITrackerModel.hpp"
#include "zoppler/utils/MathUtils.hpp"
#include <algorithm>
#include <limits>
#include <cmath>

namespace zoppler {

// IDataAssociation base class implementations
double IDataAssociation::computeGatingDistance(const Track& track, 
                                                const Detection& detection) const {
    TrackState state = track.getState();
    
    // Simple Euclidean distance (for non-statistical gating)
    double dx = state.x - detection.x;
    double dy = state.y - detection.y;
    double dz = (isValid(state.z) && isValid(detection.z)) 
                ? (state.z - detection.z) : 0.0;
    
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool IDataAssociation::passesGate(const Track& track, 
                                   const Detection& detection,
                                   double& distance) const {
    distance = computeGatingDistance(track, detection);
    return distance <= gateThreshold_;
}

// GNNAssociation implementations
GNNAssociation::GNNAssociation(double gateThreshold, double maxDistance)
    : maxDistance_(maxDistance)
    , useStatisticalDistance_(false) {
    gateThreshold_ = gateThreshold;
}

GNNAssociation::GNNAssociation(const TrackerConfig& config)
    : maxDistance_(config.associationParams.maxAssociationDistance)
    , useStatisticalDistance_(true) {
    gateThreshold_ = config.trackerProfile.gateChiSq;
}

std::unique_ptr<IDataAssociation> GNNAssociation::clone() const {
    auto copy = std::make_unique<GNNAssociation>(gateThreshold_, maxDistance_);
    copy->useStatisticalDistance_ = useStatisticalDistance_;
    return copy;
}

std::vector<std::vector<double>> GNNAssociation::buildCostMatrix(
    const std::vector<Track>& tracks,
    const std::vector<Detection>& detections) const {
    
    size_t nTracks = tracks.size();
    size_t nDets = detections.size();
    
    // Large cost for invalid assignments
    constexpr double INF_COST = 1e9;
    
    std::vector<std::vector<double>> costMatrix(nTracks, 
                                                 std::vector<double>(nDets, INF_COST));
    
    for (size_t i = 0; i < nTracks; ++i) {
        if (!tracks[i].isActive()) continue;
        
        TrackState state = tracks[i].getState();
        
        for (size_t j = 0; j < nDets; ++j) {
            const Detection& det = detections[j];
            
            // Compute distance
            double dx = state.x - det.x;
            double dy = state.y - det.y;
            double dz = (isValid(state.z) && isValid(det.z)) 
                        ? (state.z - det.z) : 0.0;
            
            double euclidDist = std::sqrt(dx * dx + dy * dy + dz * dz);
            
            if (useStatisticalDistance_) {
                // Compute statistical distance if covariance available
                double sigmaX = (state.sigmaX > 0) ? state.sigmaX : 10.0;
                double sigmaY = (state.sigmaY > 0) ? state.sigmaY : 10.0;
                double sigmaZ = (state.sigmaZ > 0) ? state.sigmaZ : 20.0;
                
                double statDist = (dx * dx) / (sigmaX * sigmaX)
                                + (dy * dy) / (sigmaY * sigmaY)
                                + (dz * dz) / (sigmaZ * sigmaZ);
                
                // Gate check
                if (statDist <= gateThreshold_ && euclidDist <= maxDistance_) {
                    costMatrix[i][j] = statDist;
                }
            } else {
                // Simple Euclidean distance
                if (euclidDist <= maxDistance_) {
                    costMatrix[i][j] = euclidDist;
                }
            }
        }
    }
    
    return costMatrix;
}

AssociationResult GNNAssociation::associate(
    const std::vector<Track>& tracks,
    const std::vector<Detection>& detections) {
    
    AssociationResult result;
    size_t nTracks = tracks.size();
    size_t nDets = detections.size();
    
    result.trackToDetection.resize(nTracks, -1);
    result.detectionToTrack.resize(nDets, -1);
    
    if (nTracks == 0 || nDets == 0) {
        // All detections are unassociated
        for (size_t j = 0; j < nDets; ++j) {
            result.unassociatedDetections.push_back(j);
        }
        for (size_t i = 0; i < nTracks; ++i) {
            if (tracks[i].isActive()) {
                result.unassociatedTracks.push_back(i);
            }
        }
        return result;
    }
    
    // Build cost matrix
    auto costMatrix = buildCostMatrix(tracks, detections);
    
    // Run Hungarian algorithm
    std::vector<int> assignment = MathUtils::hungarianAssignment(costMatrix);
    
    // Process assignment results
    constexpr double INF_COST = 1e9;
    
    for (size_t i = 0; i < nTracks; ++i) {
        int assignedDet = (i < assignment.size()) ? assignment[i] : -1;
        
        if (assignedDet >= 0 && 
            static_cast<size_t>(assignedDet) < nDets &&
            costMatrix[i][assignedDet] < INF_COST - 1) {
            // Valid assignment
            result.trackToDetection[i] = assignedDet;
            result.detectionToTrack[assignedDet] = static_cast<int>(i);
            result.associationCosts.push_back(costMatrix[i][assignedDet]);
        } else {
            // Track not associated
            if (tracks[i].isActive()) {
                result.unassociatedTracks.push_back(i);
            }
        }
    }
    
    // Find unassociated detections
    for (size_t j = 0; j < nDets; ++j) {
        if (result.detectionToTrack[j] < 0) {
            result.unassociatedDetections.push_back(j);
        }
    }
    
    return result;
}

} // namespace zoppler
