/**
 * @file TrackManager.cpp
 * @brief Track manager implementation
 * @copyright Defense-grade radar tracking system
 */

#include "zoppler/engine/TrackManager.hpp"
#include <algorithm>

namespace zoppler {

TrackManager::TrackManager(const TrackerConfig& config)
    : config_(config) {
    
    const auto& profile = config.trackerProfile;
    
    confirmM_ = profile.confirmHits;
    confirmN_ = profile.confirmWindow;
    maxMisses_ = profile.maxMisses;
    tentativeMaxMisses_ = profile.tentativeMaxMisses;
    
    confidenceGain_ = profile.confidenceGain;
    confidenceLoss_ = profile.confidenceLoss;
    initialConfidence_ = profile.initialConfidence;
    minConfidence_ = profile.minConfidence;
    minInitSNR_ = profile.minInitSNR;
    
    maxTracks_ = config.trackManagement.maxTracks;
}

TrackId TrackManager::allocateTrackId() {
    return nextTrackId_++;
}

Measurement TrackManager::createMeasurement(const Detection& detection) const {
    return Measurement::fromDetection(
        detection,
        config_.radar.xStd,
        config_.radar.yStd,
        config_.radar.zStd,
        config_.trackingParams.usePolarMeasurement
    );
}

bool TrackManager::passesInitiationCriteria(const Detection& detection) const {
    // Check SNR threshold
    if (isValid(detection.snr) && detection.snr < minInitSNR_) {
        return false;
    }
    
    // Check if detection is within radar coverage
    if (detection.hasValidPolar()) {
        if (detection.range < config_.radar.minRange || 
            detection.range > config_.radar.maxRange) {
            return false;
        }
    }
    
    // Check quality threshold
    if (detection.quality < 0.3) {
        return false;
    }
    
    return true;
}

int TrackManager::initiateTracksFromDetections(const std::vector<Detection>& detections,
                                                Timestamp timestamp) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    int initiated = 0;
    
    for (const auto& detection : detections) {
        // Check capacity
        if (tracks_.size() >= maxTracks_) {
            break;
        }
        
        // Check initiation criteria
        if (!passesInitiationCriteria(detection)) {
            continue;
        }
        
        // Create new track
        Track track;
        track.id = allocateTrackId();
        track.status = TrackStatus::TENTATIVE;
        track.creationTime = timestamp;
        track.lastUpdateTime = timestamp;
        track.lastPredictionTime = timestamp;
        track.confidence = initialConfidence_;
        track.confirmWindowSize = confirmN_;
        track.confirmThreshold = confirmM_;
        
        // Create tracker model
        track.model = TrackerFactory::create(config_);
        
        // Initialize from detection
        Measurement meas = createMeasurement(detection);
        track.model->initialize(meas, 
                               config_.trackerProfile.initVelStd,
                               config_.trackerProfile.initAccStd);
        
        // Record initial hit
        track.recordHit(confidenceGain_);
        track.lastDetectionId = detection.clusterId;
        
        // Add state to history
        track.addToHistory(track.model->getState());
        
        tracks_.push_back(std::move(track));
        initiated++;
    }
    
    return initiated;
}

void TrackManager::updateTrackStatus(size_t trackIdx, bool associated, Timestamp timestamp) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (trackIdx >= tracks_.size()) return;
    
    Track& track = tracks_[trackIdx];
    
    if (associated) {
        track.recordHit(confidenceGain_);
        track.lastUpdateTime = timestamp;
        
        // Add state to history
        track.addToHistory(track.model->getState());
    } else {
        track.recordMiss(confidenceLoss_);
    }
    
    track.lastPredictionTime = timestamp;
}

int TrackManager::performMaintenance(Timestamp currentTime) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    int deleted = 0;
    
    auto it = tracks_.begin();
    while (it != tracks_.end()) {
        Track& track = *it;
        
        // Check deletion criteria
        bool shouldDelete = false;
        
        // Consecutive misses threshold
        if (track.status == TrackStatus::TENTATIVE) {
            if (track.misses > tentativeMaxMisses_) {
                shouldDelete = true;
            }
        } else {
            if (track.misses > maxMisses_) {
                shouldDelete = true;
            }
        }
        
        // Confidence threshold
        if (track.confidence < minConfidence_) {
            shouldDelete = true;
        }
        
        // Time since last update
        double timeSinceUpdate = track.timeSinceUpdate(currentTime);
        if (timeSinceUpdate > config_.trackerProfile.maxExtrapolationTime) {
            shouldDelete = true;
        }
        
        // Check if out of radar range
        if (config_.trackManagement.deleteOutOfRange) {
            TrackState state = track.getState();
            double range = state.getRange();
            if (range > config_.radar.maxRange * 1.2 || range < config_.radar.minRange * 0.5) {
                shouldDelete = true;
            }
        }
        
        // Already marked for deletion
        if (track.status == TrackStatus::DELETED) {
            shouldDelete = true;
        }
        
        if (shouldDelete) {
            it = tracks_.erase(it);
            deleted++;
        } else {
            ++it;
        }
    }
    
    return deleted;
}

Track* TrackManager::getTrackById(TrackId id) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = std::find_if(tracks_.begin(), tracks_.end(),
        [id](const Track& t) { return t.id == id; });
    
    return (it != tracks_.end()) ? &(*it) : nullptr;
}

const Track* TrackManager::getTrackById(TrackId id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = std::find_if(tracks_.begin(), tracks_.end(),
        [id](const Track& t) { return t.id == id; });
    
    return (it != tracks_.end()) ? &(*it) : nullptr;
}

size_t TrackManager::getNumActiveTracks() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    return std::count_if(tracks_.begin(), tracks_.end(),
        [](const Track& t) { return t.isActive(); });
}

size_t TrackManager::getNumConfirmedTracks() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    return std::count_if(tracks_.begin(), tracks_.end(),
        [](const Track& t) { return t.isConfirmed(); });
}

void TrackManager::clearTracks() {
    std::lock_guard<std::mutex> lock(mutex_);
    tracks_.clear();
}

void TrackManager::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    tracks_.clear();
    nextTrackId_ = 1;
}

void TrackManager::setConfirmationLogic(int M, int N) {
    std::lock_guard<std::mutex> lock(mutex_);
    confirmM_ = M;
    confirmN_ = N;
    
    // Update existing tracks
    for (auto& track : tracks_) {
        track.confirmThreshold = M;
        track.confirmWindowSize = N;
    }
}

void TrackManager::setMaxMisses(int maxMisses) {
    std::lock_guard<std::mutex> lock(mutex_);
    maxMisses_ = maxMisses;
}

void TrackManager::setMinConfidence(double minConfidence) {
    std::lock_guard<std::mutex> lock(mutex_);
    minConfidence_ = minConfidence;
}

void TrackManager::setConfidenceParameters(double gain, double loss, double initial) {
    std::lock_guard<std::mutex> lock(mutex_);
    confidenceGain_ = gain;
    confidenceLoss_ = loss;
    initialConfidence_ = initial;
}

// Track implementation for getState()
TrackState Track::getState() const {
    if (model) {
        return model->getState();
    }
    return TrackState();
}

TrackState Track::getPredictedState(Timestamp targetTime) const {
    if (!model) return TrackState();
    
    TrackState current = model->getState();
    double dt = (targetTime - lastPredictionTime) / 1e6;  // Convert to seconds
    
    return current.extrapolate(dt);
}

double Track::distanceTo(const Detection& det) const {
    if (!model) return INVALID_VALUE;
    
    TrackState state = model->getState();
    
    double dx = state.x - det.x;
    double dy = state.y - det.y;
    double dz = (isValid(state.z) && isValid(det.z)) 
                ? (state.z - det.z) : 0.0;
    
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

} // namespace zoppler
