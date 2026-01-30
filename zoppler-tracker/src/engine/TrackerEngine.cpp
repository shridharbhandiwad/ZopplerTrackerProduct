/**
 * @file TrackerEngine.cpp
 * @brief Tracker engine implementation
 * @copyright Defense-grade radar tracking system
 */

#include "zoppler/engine/TrackerEngine.hpp"
#include "zoppler/clustering/ClusteringFactory.hpp"
#include "zoppler/association/AssociationFactory.hpp"
#include <chrono>
#include <algorithm>

namespace zoppler {

TrackerEngine::TrackerEngine(const TrackerConfig& config)
    : config_(config) {
    
    // Create algorithm components
    clustering_ = ClusteringFactory::create(config);
    association_ = AssociationFactory::create(config);
    trackManager_ = std::make_unique<TrackManager>(config);
}

void TrackerEngine::reconfigure(const TrackerConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    config_ = config;
    
    // Recreate algorithm components
    clustering_ = ClusteringFactory::create(config);
    association_ = AssociationFactory::create(config);
    trackManager_ = std::make_unique<TrackManager>(config);
    
    // Reset state
    lastClusters_.clear();
    lastAssociation_ = AssociationResult();
    lastProcessTime_ = 0;
    firstScan_ = true;
}

void TrackerEngine::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    trackManager_->reset();
    lastClusters_.clear();
    lastAssociation_ = AssociationResult();
    lastProcessTime_ = 0;
    firstScan_ = true;
    stats_ = Statistics();
}

Measurement TrackerEngine::createMeasurement(const Detection& centroid) const {
    return Measurement::fromDetection(
        centroid,
        config_.radar.xStd,
        config_.radar.yStd,
        config_.radar.zStd,
        config_.trackingParams.usePolarMeasurement
    );
}

void TrackerEngine::performClustering(const std::vector<Detection>& detections) {
    if (detections.empty()) {
        lastClusters_.clear();
        return;
    }
    
    lastClusters_ = clustering_->cluster(detections);
    stats_.totalClusters += lastClusters_.size();
}

void TrackerEngine::performPrediction(double dt) {
    auto& tracks = trackManager_->getTracks();
    
    for (auto& track : tracks) {
        if (track.isActive() && track.model) {
            track.model->predict(dt);
        }
    }
}

void TrackerEngine::performAssociation(const std::vector<Detection>& clusterCentroids) {
    const auto& tracks = trackManager_->getTracks();
    
    if (tracks.empty() || clusterCentroids.empty()) {
        // No tracks or no detections
        lastAssociation_ = AssociationResult();
        lastAssociation_.trackToDetection.resize(tracks.size(), -1);
        lastAssociation_.detectionToTrack.resize(clusterCentroids.size(), -1);
        
        for (size_t i = 0; i < tracks.size(); ++i) {
            if (tracks[i].isActive()) {
                lastAssociation_.unassociatedTracks.push_back(i);
            }
        }
        for (size_t j = 0; j < clusterCentroids.size(); ++j) {
            lastAssociation_.unassociatedDetections.push_back(j);
        }
        return;
    }
    
    lastAssociation_ = association_->associate(tracks, clusterCentroids);
}

void TrackerEngine::performUpdate(const std::vector<Detection>& clusterCentroids,
                                   Timestamp timestamp) {
    auto& tracks = trackManager_->getTracks();
    
    // Update associated tracks
    for (size_t i = 0; i < tracks.size(); ++i) {
        int detIdx = lastAssociation_.getDetectionForTrack(i);
        
        if (detIdx >= 0 && static_cast<size_t>(detIdx) < clusterCentroids.size()) {
            // Track associated - perform measurement update
            if (tracks[i].model) {
                Measurement meas = createMeasurement(clusterCentroids[detIdx]);
                tracks[i].model->update(meas);
            }
            
            trackManager_->updateTrackStatus(i, true, timestamp);
            tracks[i].lastClusterId = clusterCentroids[detIdx].clusterId;
        } else {
            // Track not associated - coast
            trackManager_->updateTrackStatus(i, false, timestamp);
        }
    }
}

void TrackerEngine::initiateNewTracks(const std::vector<Detection>& clusterCentroids,
                                       Timestamp timestamp) {
    // Collect unassociated detections
    std::vector<Detection> unassociatedDets;
    unassociatedDets.reserve(lastAssociation_.unassociatedDetections.size());
    
    for (size_t idx : lastAssociation_.unassociatedDetections) {
        if (idx < clusterCentroids.size()) {
            unassociatedDets.push_back(clusterCentroids[idx]);
        }
    }
    
    // Initiate tracks
    int initiated = trackManager_->initiateTracksFromDetections(unassociatedDets, timestamp);
    stats_.tracksInitiated += initiated;
}

void TrackerEngine::performMaintenance(Timestamp timestamp) {
    int deleted = trackManager_->performMaintenance(timestamp);
    stats_.tracksDeleted += deleted;
}

void TrackerEngine::processDetections(const std::vector<Detection>& detections,
                                       Timestamp timestamp) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto startTime = std::chrono::high_resolution_clock::now();
    
    stats_.totalScans++;
    stats_.totalDetections += detections.size();
    stats_.lastTimestamp = timestamp;
    
    // Compute time delta
    double dt = 0.0;
    if (!firstScan_ && lastProcessTime_ > 0) {
        dt = (timestamp - lastProcessTime_) / 1e6;  // Convert to seconds
        
        // Sanity check on dt
        if (dt < 0) {
            dt = config_.radar.updateTime;  // Use nominal if timestamp goes backward
        } else if (dt > 10.0) {
            dt = 10.0;  // Cap at 10 seconds
        }
    } else {
        dt = config_.radar.updateTime;  // Use nominal for first scan
        firstScan_ = false;
    }
    
    // 1. Clustering
    performClustering(detections);
    
    // Extract cluster centroids
    std::vector<Detection> clusterCentroids;
    clusterCentroids.reserve(lastClusters_.size());
    for (const auto& cluster : lastClusters_) {
        clusterCentroids.push_back(cluster.getCentroidDetection());
    }
    
    // 2. Prediction (existing tracks)
    performPrediction(dt);
    
    // 3. Data Association
    performAssociation(clusterCentroids);
    
    // 4. Measurement Update
    performUpdate(clusterCentroids, timestamp);
    
    // 5. Track Initiation
    initiateNewTracks(clusterCentroids, timestamp);
    
    // 6. Track Maintenance
    performMaintenance(timestamp);
    
    // Update timing
    lastProcessTime_ = timestamp;
    
    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
    stats_.lastProcessingTimeMs = duration.count() / 1000.0;
}

const std::vector<Track>& TrackerEngine::getTracks() const {
    return trackManager_->getTracks();
}

std::vector<const Track*> TrackerEngine::getConfirmedTracks() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<const Track*> confirmed;
    for (const auto& track : trackManager_->getTracks()) {
        if (track.isConfirmed()) {
            confirmed.push_back(&track);
        }
    }
    return confirmed;
}

std::vector<TrackState> TrackerEngine::getTrackStates() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<TrackState> states;
    for (const auto& track : trackManager_->getTracks()) {
        if (track.isActive() && track.model) {
            states.push_back(track.model->getState());
        }
    }
    return states;
}

const Track* TrackerEngine::getTrackById(TrackId id) const {
    return trackManager_->getTrackById(id);
}

size_t TrackerEngine::getNumTracks() const {
    return trackManager_->getNumActiveTracks();
}

size_t TrackerEngine::getNumConfirmedTracks() const {
    return trackManager_->getNumConfirmedTracks();
}

} // namespace zoppler
