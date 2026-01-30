/**
 * @file MHTAssociation.cpp
 * @brief MHT implementation
 * @copyright Defense-grade radar tracking system
 */

#include "zoppler/association/MHTAssociation.hpp"
#include "zoppler/tracking/ITrackerModel.hpp"
#include "zoppler/utils/MathUtils.hpp"
#include <algorithm>
#include <cmath>
#include <queue>
#include <functional>

namespace zoppler {

MHTAssociation::MHTAssociation(int nScan, int maxHypotheses, double pruneThreshold)
    : nScan_(nScan)
    , maxHypotheses_(maxHypotheses)
    , pruneThreshold_(pruneThreshold)
    , pD_(0.9)
    , clutterRate_(1e-6)
    , newTrackProb_(0.1) {
    gateThreshold_ = 11.34;  // 99% for 3 DOF
}

MHTAssociation::MHTAssociation(const TrackerConfig& config)
    : nScan_(config.associationParams.mhtNScan)
    , maxHypotheses_(config.associationParams.mhtMaxHypotheses)
    , pruneThreshold_(config.associationParams.mhtPruneThreshold)
    , pD_(config.radar.pD)
    , clutterRate_(config.radar.clutterDensity)
    , newTrackProb_(0.1) {
    gateThreshold_ = config.trackerProfile.gateChiSq3D;
}

std::unique_ptr<IDataAssociation> MHTAssociation::clone() const {
    auto copy = std::make_unique<MHTAssociation>(nScan_, maxHypotheses_, pruneThreshold_);
    copy->pD_ = pD_;
    copy->clutterRate_ = clutterRate_;
    copy->newTrackProb_ = newTrackProb_;
    copy->gateThreshold_ = gateThreshold_;
    return copy;
}

void MHTAssociation::reset() {
    hypotheses_.clear();
    hypothesisHistory_.clear();
    nextHypothesisId_ = 0;
    scanCount_ = 0;
}

double MHTAssociation::computeHypothesisLikelihood(
    const std::vector<Track>& tracks,
    const std::vector<Detection>& detections,
    const MHTHypothesis& hypothesis) const {
    
    double logLikelihood = 0.0;
    int nFalseAlarms = 0;
    
    // Track-detection associations
    for (size_t t = 0; t < hypothesis.trackToDetection.size(); ++t) {
        int detIdx = hypothesis.trackToDetection[t];
        
        if (detIdx >= 0 && t < tracks.size()) {
            // Track associated with detection
            const Track& track = tracks[t];
            const Detection& det = detections[detIdx];
            
            TrackState state = track.getState();
            
            double dx = det.x - state.x;
            double dy = det.y - state.y;
            double dz = (isValid(det.z) && isValid(state.z)) 
                        ? (det.z - state.z) : 0.0;
            
            double sigmaX = (state.sigmaX > 0) ? state.sigmaX : 10.0;
            double sigmaY = (state.sigmaY > 0) ? state.sigmaY : 10.0;
            double sigmaZ = (state.sigmaZ > 0) ? state.sigmaZ : 20.0;
            
            double mahalDist = (dx * dx) / (sigmaX * sigmaX)
                             + (dy * dy) / (sigmaY * sigmaY)
                             + (dz * dz) / (sigmaZ * sigmaZ);
            
            if (mahalDist > gateThreshold_) {
                return -1e10;  // Invalid assignment
            }
            
            // Log likelihood contribution
            logLikelihood += std::log(pD_) - 0.5 * mahalDist 
                           - std::log(sigmaX * sigmaY * sigmaZ);
        } else if (t < tracks.size() && tracks[t].isActive()) {
            // Track missed
            logLikelihood += std::log(1.0 - pD_);
        }
    }
    
    // Count false alarms (unassociated detections that are not new tracks)
    std::vector<bool> detUsed(detections.size(), false);
    for (int detIdx : hypothesis.trackToDetection) {
        if (detIdx >= 0 && static_cast<size_t>(detIdx) < detUsed.size()) {
            detUsed[detIdx] = true;
        }
    }
    for (int detIdx : hypothesis.newTrackDetections) {
        if (detIdx >= 0 && static_cast<size_t>(detIdx) < detUsed.size()) {
            detUsed[detIdx] = true;
        }
    }
    
    for (size_t j = 0; j < detections.size(); ++j) {
        if (!detUsed[j]) {
            nFalseAlarms++;
        }
    }
    
    // False alarm likelihood (Poisson)
    logLikelihood += nFalseAlarms * std::log(clutterRate_);
    
    // New track likelihood
    logLikelihood += hypothesis.newTrackDetections.size() * std::log(newTrackProb_);
    
    return logLikelihood;
}

void MHTAssociation::generateHypotheses(
    const std::vector<Track>& tracks,
    const std::vector<Detection>& detections) {
    
    size_t nTracks = tracks.size();
    size_t nDets = detections.size();
    
    std::list<MHTHypothesis> newHypotheses;
    
    // If no existing hypotheses, create initial hypothesis
    if (hypotheses_.empty()) {
        MHTHypothesis initial;
        initial.id = nextHypothesisId_++;
        initial.probability = 1.0;
        initial.trackToDetection.resize(nTracks, -1);
        hypotheses_.push_back(initial);
    }
    
    // Expand each existing hypothesis
    for (const auto& parentHyp : hypotheses_) {
        // Find valid assignments (gated)
        std::vector<std::vector<int>> validDetsForTrack(nTracks);
        
        for (size_t t = 0; t < nTracks; ++t) {
            if (!tracks[t].isActive()) continue;
            
            validDetsForTrack[t].push_back(-1);  // Missed detection option
            
            for (size_t j = 0; j < nDets; ++j) {
                TrackState state = tracks[t].getState();
                const Detection& det = detections[j];
                
                double dx = det.x - state.x;
                double dy = det.y - state.y;
                double dz = (isValid(det.z) && isValid(state.z)) 
                            ? (det.z - state.z) : 0.0;
                
                double sigmaX = (state.sigmaX > 0) ? state.sigmaX : 10.0;
                double sigmaY = (state.sigmaY > 0) ? state.sigmaY : 10.0;
                double sigmaZ = (state.sigmaZ > 0) ? state.sigmaZ : 20.0;
                
                double mahalDist = (dx * dx) / (sigmaX * sigmaX)
                                 + (dy * dy) / (sigmaY * sigmaY)
                                 + (dz * dz) / (sigmaZ * sigmaZ);
                
                if (mahalDist <= gateThreshold_) {
                    validDetsForTrack[t].push_back(static_cast<int>(j));
                }
            }
        }
        
        // Generate child hypotheses using recursive enumeration
        std::function<void(size_t, MHTHypothesis&, std::vector<bool>&)> enumerate;
        
        enumerate = [&](size_t trackIdx, MHTHypothesis& hyp, std::vector<bool>& usedDets) {
            if (newHypotheses.size() >= static_cast<size_t>(maxHypotheses_)) {
                return;
            }
            
            if (trackIdx == nTracks) {
                // Determine new track detections
                hyp.newTrackDetections.clear();
                for (size_t j = 0; j < nDets; ++j) {
                    if (!usedDets[j]) {
                        // Could be false alarm or new track
                        // For simplicity, consider as potential new track
                        hyp.newTrackDetections.push_back(static_cast<int>(j));
                    }
                }
                
                // Compute likelihood
                double logLikelihood = computeHypothesisLikelihood(tracks, detections, hyp);
                hyp.probability = parentHyp.probability * std::exp(logLikelihood);
                hyp.id = nextHypothesisId_++;
                
                newHypotheses.push_back(hyp);
                return;
            }
            
            if (!tracks[trackIdx].isActive()) {
                hyp.trackToDetection[trackIdx] = -1;
                enumerate(trackIdx + 1, hyp, usedDets);
                return;
            }
            
            for (int detIdx : validDetsForTrack[trackIdx]) {
                if (detIdx >= 0 && usedDets[detIdx]) {
                    continue;  // Detection already used
                }
                
                hyp.trackToDetection[trackIdx] = detIdx;
                if (detIdx >= 0) {
                    usedDets[detIdx] = true;
                }
                
                enumerate(trackIdx + 1, hyp, usedDets);
                
                if (detIdx >= 0) {
                    usedDets[detIdx] = false;
                }
            }
        };
        
        MHTHypothesis childHyp;
        childHyp.trackToDetection.resize(nTracks, -1);
        std::vector<bool> usedDets(nDets, false);
        enumerate(0, childHyp, usedDets);
    }
    
    hypotheses_ = std::move(newHypotheses);
}

void MHTAssociation::normalizeHypotheses() {
    if (hypotheses_.empty()) return;
    
    double totalProb = 0.0;
    for (const auto& hyp : hypotheses_) {
        totalProb += hyp.probability;
    }
    
    if (totalProb > 0) {
        for (auto& hyp : hypotheses_) {
            hyp.probability /= totalProb;
        }
    }
}

void MHTAssociation::pruneHypotheses() {
    // Remove low-probability hypotheses
    hypotheses_.remove_if([this](const MHTHypothesis& hyp) {
        return hyp.probability < pruneThreshold_;
    });
    
    // Keep only top maxHypotheses
    if (hypotheses_.size() > static_cast<size_t>(maxHypotheses_)) {
        hypotheses_.sort();
        auto it = hypotheses_.begin();
        std::advance(it, maxHypotheses_);
        hypotheses_.erase(it, hypotheses_.end());
    }
    
    // Ensure at least one hypothesis
    if (hypotheses_.empty()) {
        MHTHypothesis defaultHyp;
        defaultHyp.id = nextHypothesisId_++;
        defaultHyp.probability = 1.0;
        hypotheses_.push_back(defaultHyp);
    }
    
    normalizeHypotheses();
}

void MHTAssociation::nScanPrune() {
    // Store current hypotheses in history
    hypothesisHistory_.push_back(hypotheses_);
    
    // Keep only last N scans
    while (hypothesisHistory_.size() > static_cast<size_t>(nScan_)) {
        hypothesisHistory_.erase(hypothesisHistory_.begin());
    }
    
    scanCount_++;
}

const MHTHypothesis& MHTAssociation::getBestHypothesis() const {
    static MHTHypothesis empty;
    
    if (hypotheses_.empty()) {
        return empty;
    }
    
    auto best = std::max_element(hypotheses_.begin(), hypotheses_.end(),
        [](const MHTHypothesis& a, const MHTHypothesis& b) {
            return a.probability < b.probability;
        });
    
    return *best;
}

AssociationResult MHTAssociation::associate(
    const std::vector<Track>& tracks,
    const std::vector<Detection>& detections) {
    
    AssociationResult result;
    size_t nTracks = tracks.size();
    size_t nDets = detections.size();
    
    result.trackToDetection.resize(nTracks, -1);
    result.detectionToTrack.resize(nDets, -1);
    
    if (nTracks == 0 && nDets == 0) {
        return result;
    }
    
    // Generate new hypotheses
    generateHypotheses(tracks, detections);
    
    // Prune hypotheses
    pruneHypotheses();
    
    // Apply N-scan pruning
    nScanPrune();
    
    // Get best hypothesis for output
    const MHTHypothesis& bestHyp = getBestHypothesis();
    
    // Build result from best hypothesis
    for (size_t t = 0; t < std::min(nTracks, bestHyp.trackToDetection.size()); ++t) {
        int detIdx = bestHyp.trackToDetection[t];
        
        if (detIdx >= 0 && static_cast<size_t>(detIdx) < nDets) {
            result.trackToDetection[t] = detIdx;
            result.detectionToTrack[detIdx] = static_cast<int>(t);
        } else if (tracks[t].isActive()) {
            result.unassociatedTracks.push_back(t);
        }
    }
    
    // Handle remaining unassociated tracks
    for (size_t t = bestHyp.trackToDetection.size(); t < nTracks; ++t) {
        if (tracks[t].isActive()) {
            result.unassociatedTracks.push_back(t);
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
