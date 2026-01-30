/**
 * @file JPDAAssociation.cpp
 * @brief JPDA implementation
 * @copyright Defense-grade radar tracking system
 */

#include "zoppler/association/JPDAAssociation.hpp"
#include "zoppler/tracking/ITrackerModel.hpp"
#include "zoppler/utils/MathUtils.hpp"
#include "zoppler/utils/Matrix.hpp"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <functional>

namespace zoppler {

JPDAAssociation::JPDAAssociation(double pD, double clutterRate, double gateThreshold)
    : pD_(pD)
    , clutterRate_(clutterRate)
    , maxHypotheses_(100) {
    gateThreshold_ = gateThreshold;
}

JPDAAssociation::JPDAAssociation(const TrackerConfig& config)
    : pD_(config.associationParams.jpdaPD)
    , clutterRate_(config.associationParams.jpdaClutterRate)
    , maxHypotheses_(config.associationParams.jpdaMaxHypotheses) {
    gateThreshold_ = config.trackerProfile.gateChiSq;
}

std::unique_ptr<IDataAssociation> JPDAAssociation::clone() const {
    auto copy = std::make_unique<JPDAAssociation>(pD_, clutterRate_, gateThreshold_);
    copy->maxHypotheses_ = maxHypotheses_;
    return copy;
}

double JPDAAssociation::computeLikelihood(const Track& track, 
                                           const Detection& detection) const {
    TrackState state = track.getState();
    
    // Innovation (measurement residual)
    double dx = detection.x - state.x;
    double dy = detection.y - state.y;
    double dz = (isValid(detection.z) && isValid(state.z)) 
                ? (detection.z - state.z) : 0.0;
    
    // Use track covariance for likelihood computation
    double sigmaX = (state.sigmaX > 0) ? state.sigmaX : 10.0;
    double sigmaY = (state.sigmaY > 0) ? state.sigmaY : 10.0;
    double sigmaZ = (state.sigmaZ > 0) ? state.sigmaZ : 20.0;
    
    // Measurement noise (add to track prediction covariance)
    double measNoise = 5.0;  // Measurement noise std
    sigmaX = std::sqrt(sigmaX * sigmaX + measNoise * measNoise);
    sigmaY = std::sqrt(sigmaY * sigmaY + measNoise * measNoise);
    sigmaZ = std::sqrt(sigmaZ * sigmaZ + measNoise * measNoise);
    
    // Gaussian likelihood
    double mahalDist = (dx * dx) / (sigmaX * sigmaX)
                     + (dy * dy) / (sigmaY * sigmaY)
                     + (dz * dz) / (sigmaZ * sigmaZ);
    
    // Check gate
    if (mahalDist > gateThreshold_) {
        return 0.0;
    }
    
    // Multivariate Gaussian PDF
    int dim = isValid(detection.z) ? 3 : 2;
    double det = sigmaX * sigmaX * sigmaY * sigmaY;
    if (dim == 3) det *= sigmaZ * sigmaZ;
    
    double normConst = std::pow(2.0 * PI, -dim / 2.0) * std::pow(det, -0.5);
    double likelihood = normConst * std::exp(-0.5 * mahalDist);
    
    return likelihood;
}

void JPDAAssociation::enumerateJointEvents(
    const std::vector<std::vector<bool>>& validAssignments,
    std::vector<std::vector<int>>& jointEvents) const {
    
    size_t nTracks = validAssignments.size();
    if (nTracks == 0) return;
    
    size_t nDets = validAssignments[0].size();
    
    // Recursive enumeration with pruning
    std::function<void(size_t, std::vector<int>&, std::vector<bool>&)> enumerate;
    
    enumerate = [&](size_t trackIdx, std::vector<int>& event, std::vector<bool>& usedDets) {
        if (jointEvents.size() >= static_cast<size_t>(maxHypotheses_)) {
            return;  // Pruning
        }
        
        if (trackIdx == nTracks) {
            jointEvents.push_back(event);
            return;
        }
        
        // Option 1: Track not detected (missed)
        event[trackIdx] = -1;
        enumerate(trackIdx + 1, event, usedDets);
        
        // Option 2: Track associated with each valid detection
        for (size_t j = 0; j < nDets; ++j) {
            if (validAssignments[trackIdx][j] && !usedDets[j]) {
                event[trackIdx] = static_cast<int>(j);
                usedDets[j] = true;
                enumerate(trackIdx + 1, event, usedDets);
                usedDets[j] = false;
            }
        }
    };
    
    std::vector<int> event(nTracks, -1);
    std::vector<bool> usedDets(nDets, false);
    enumerate(0, event, usedDets);
}

double JPDAAssociation::computeJointEventProbability(
    const std::vector<Track>& tracks,
    const std::vector<Detection>& detections,
    const std::vector<int>& event,
    const std::vector<std::vector<double>>& likelihoods) const {
    
    double prob = 1.0;
    int nDetected = 0;
    int nFalseAlarms = static_cast<int>(detections.size());
    
    for (size_t t = 0; t < event.size(); ++t) {
        int detIdx = event[t];
        
        if (detIdx >= 0) {
            // Track detected
            prob *= pD_ * likelihoods[t][detIdx];
            nDetected++;
            nFalseAlarms--;
        } else {
            // Track missed
            prob *= (1.0 - pD_);
        }
    }
    
    // False alarm probability (Poisson)
    double clutterProb = std::pow(clutterRate_, nFalseAlarms);
    prob *= clutterProb;
    
    return prob;
}

AssociationResult JPDAAssociation::associate(
    const std::vector<Track>& tracks,
    const std::vector<Detection>& detections) {
    
    AssociationResult result;
    size_t nTracks = tracks.size();
    size_t nDets = detections.size();
    
    result.trackToDetection.resize(nTracks, -1);
    result.detectionToTrack.resize(nDets, -1);
    
    if (nTracks == 0) {
        for (size_t j = 0; j < nDets; ++j) {
            result.unassociatedDetections.push_back(j);
        }
        return result;
    }
    
    if (nDets == 0) {
        for (size_t i = 0; i < nTracks; ++i) {
            if (tracks[i].isActive()) {
                result.unassociatedTracks.push_back(i);
            }
        }
        return result;
    }
    
    // Compute likelihoods and valid assignments
    std::vector<std::vector<double>> likelihoods(nTracks, std::vector<double>(nDets, 0.0));
    std::vector<std::vector<bool>> validAssignments(nTracks, std::vector<bool>(nDets, false));
    
    for (size_t i = 0; i < nTracks; ++i) {
        if (!tracks[i].isActive()) continue;
        
        for (size_t j = 0; j < nDets; ++j) {
            double likelihood = computeLikelihood(tracks[i], detections[j]);
            likelihoods[i][j] = likelihood;
            validAssignments[i][j] = (likelihood > 0);
        }
    }
    
    // Enumerate joint events
    std::vector<std::vector<int>> jointEvents;
    enumerateJointEvents(validAssignments, jointEvents);
    
    if (jointEvents.empty()) {
        // No valid assignments - all tracks missed, all detections unassociated
        for (size_t i = 0; i < nTracks; ++i) {
            if (tracks[i].isActive()) {
                result.unassociatedTracks.push_back(i);
            }
        }
        for (size_t j = 0; j < nDets; ++j) {
            result.unassociatedDetections.push_back(j);
        }
        return result;
    }
    
    // Compute joint event probabilities
    std::vector<double> eventProbs(jointEvents.size());
    double totalProb = 0.0;
    
    for (size_t e = 0; e < jointEvents.size(); ++e) {
        eventProbs[e] = computeJointEventProbability(tracks, detections, 
                                                      jointEvents[e], likelihoods);
        totalProb += eventProbs[e];
    }
    
    // Normalize probabilities
    if (totalProb > 0) {
        for (auto& p : eventProbs) {
            p /= totalProb;
        }
    }
    
    // Compute marginal association probabilities
    associationProbabilities_.clear();
    associationProbabilities_.resize(nTracks, std::vector<double>(nDets + 1, 0.0));
    // Index 0 is probability of missed detection
    // Indices 1..nDets are probabilities for each detection
    
    for (size_t e = 0; e < jointEvents.size(); ++e) {
        for (size_t t = 0; t < nTracks; ++t) {
            int detIdx = jointEvents[e][t];
            if (detIdx >= 0) {
                associationProbabilities_[t][detIdx + 1] += eventProbs[e];
            } else {
                associationProbabilities_[t][0] += eventProbs[e];
            }
        }
    }
    
    // Store in result
    result.associationProbabilities = associationProbabilities_;
    
    // Make hard assignments based on maximum probability
    // (for interface compatibility, though JPDA typically uses soft association)
    std::vector<bool> detUsed(nDets, false);
    
    for (size_t i = 0; i < nTracks; ++i) {
        if (!tracks[i].isActive()) continue;
        
        int bestDet = -1;
        double bestProb = associationProbabilities_[i][0];  // Missed detection prob
        
        for (size_t j = 0; j < nDets; ++j) {
            if (!detUsed[j] && associationProbabilities_[i][j + 1] > bestProb) {
                bestProb = associationProbabilities_[i][j + 1];
                bestDet = static_cast<int>(j);
            }
        }
        
        if (bestDet >= 0) {
            result.trackToDetection[i] = bestDet;
            result.detectionToTrack[bestDet] = static_cast<int>(i);
            detUsed[bestDet] = true;
        } else {
            result.unassociatedTracks.push_back(i);
        }
    }
    
    // Find unassociated detections
    for (size_t j = 0; j < nDets; ++j) {
        if (!detUsed[j]) {
            result.unassociatedDetections.push_back(j);
        }
    }
    
    return result;
}

} // namespace zoppler
