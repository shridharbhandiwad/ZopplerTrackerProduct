/**
 * @file Track.hpp
 * @brief Track structure for target tracking
 * @copyright Defense-grade radar tracking system
 */

#ifndef ZOPPLER_CORE_TRACK_HPP
#define ZOPPLER_CORE_TRACK_HPP

#include "Types.hpp"
#include "TrackState.hpp"
#include "Detection.hpp"
#include <memory>
#include <deque>

namespace zoppler {

// Forward declaration
class ITrackerModel;

/**
 * @brief Represents a tracked target
 * 
 * Contains the tracker filter, track state, and management information.
 */
struct Track {
    // Unique track identifier
    TrackId id = -1;
    
    // Tracker model (owns the filter)
    std::unique_ptr<ITrackerModel> model;
    
    // Track management counters
    int age = 0;                        // Total updates (hits + misses)
    int hits = 0;                       // Number of successful updates
    int misses = 0;                     // Number of consecutive misses
    int totalMisses = 0;                // Total misses over track lifetime
    
    // Track status
    TrackStatus status = TrackStatus::TENTATIVE;
    
    // Track quality metrics
    double confidence = 0.3;            // Track confidence [0, 1]
    double quality = 0.5;               // Track quality score [0, 1]
    
    // Target classification
    TargetType targetType = TargetType::UNKNOWN;
    double classificationConfidence = 0.0;
    
    // Timestamps
    Timestamp creationTime = 0;         // Track creation timestamp
    Timestamp lastUpdateTime = 0;       // Last successful update
    Timestamp lastPredictionTime = 0;   // Last prediction time
    
    // Associated detection info
    int lastDetectionId = -1;
    int lastClusterId = -1;
    
    // Track history (optional, for visualization/analysis)
    std::deque<TrackState> history;
    size_t maxHistorySize = 100;
    
    // M/N confirmation logic
    std::deque<bool> updateHistory;     // History of hits (true) / misses (false)
    int confirmWindowSize = 5;          // N in M/N logic
    int confirmThreshold = 3;           // M in M/N logic
    
    /**
     * @brief Default constructor
     */
    Track() = default;
    
    /**
     * @brief Move constructor
     */
    Track(Track&& other) noexcept = default;
    
    /**
     * @brief Move assignment
     */
    Track& operator=(Track&& other) noexcept = default;
    
    // Delete copy operations (due to unique_ptr)
    Track(const Track&) = delete;
    Track& operator=(const Track&) = delete;
    
    /**
     * @brief Record a hit (successful update)
     */
    void recordHit(double confidenceGain = 0.1) {
        age++;
        hits++;
        misses = 0;  // Reset consecutive misses
        
        updateHistory.push_back(true);
        if (updateHistory.size() > static_cast<size_t>(confirmWindowSize)) {
            updateHistory.pop_front();
        }
        
        // Update confidence
        confidence = std::min(1.0, confidence + confidenceGain);
        
        // Check confirmation status
        updateConfirmationStatus();
    }
    
    /**
     * @brief Record a miss (no update this scan)
     */
    void recordMiss(double confidenceLoss = 0.1) {
        age++;
        misses++;
        totalMisses++;
        
        updateHistory.push_back(false);
        if (updateHistory.size() > static_cast<size_t>(confirmWindowSize)) {
            updateHistory.pop_front();
        }
        
        // Update confidence
        confidence = std::max(0.0, confidence - confidenceLoss);
        
        // Update status
        if (status == TrackStatus::CONFIRMED) {
            status = TrackStatus::COASTING;
        }
        
        updateConfirmationStatus();
    }
    
    /**
     * @brief Update confirmation status based on M/N logic
     */
    void updateConfirmationStatus() {
        if (status == TrackStatus::DELETED) return;
        
        int recentHits = 0;
        for (bool hit : updateHistory) {
            if (hit) recentHits++;
        }
        
        if (recentHits >= confirmThreshold && 
            updateHistory.size() >= static_cast<size_t>(confirmThreshold)) {
            if (status == TrackStatus::TENTATIVE) {
                status = TrackStatus::CONFIRMED;
            } else if (status == TrackStatus::COASTING && misses == 0) {
                status = TrackStatus::CONFIRMED;
            }
        }
    }
    
    /**
     * @brief Check if track should be deleted
     */
    bool shouldDelete(int maxMisses, double minConfidence) const {
        return status == TrackStatus::DELETED ||
               misses > maxMisses ||
               confidence < minConfidence ||
               (status == TrackStatus::TENTATIVE && misses > 1);
    }
    
    /**
     * @brief Mark track for deletion
     */
    void markDeleted() {
        status = TrackStatus::DELETED;
    }
    
    /**
     * @brief Check if track is confirmed
     */
    bool isConfirmed() const {
        return status == TrackStatus::CONFIRMED || 
               status == TrackStatus::COASTING;
    }
    
    /**
     * @brief Check if track is active (not deleted)
     */
    bool isActive() const {
        return status != TrackStatus::DELETED;
    }
    
    /**
     * @brief Get time since last update
     */
    double timeSinceUpdate(Timestamp currentTime) const {
        return (currentTime - lastUpdateTime) / 1e6;  // Convert to seconds
    }
    
    /**
     * @brief Add state to history
     */
    void addToHistory(const TrackState& state) {
        history.push_back(state);
        if (history.size() > maxHistorySize) {
            history.pop_front();
        }
    }
    
    /**
     * @brief Get current state from model
     */
    TrackState getState() const;
    
    /**
     * @brief Get predicted position at given time
     */
    TrackState getPredictedState(Timestamp targetTime) const;
    
    /**
     * @brief Compute distance to detection
     */
    double distanceTo(const Detection& det) const;
    
    /**
     * @brief Get track summary string (for debugging)
     */
    std::string getSummary() const {
        std::string statusStr;
        switch (status) {
            case TrackStatus::TENTATIVE: statusStr = "TENT"; break;
            case TrackStatus::CONFIRMED: statusStr = "CONF"; break;
            case TrackStatus::COASTING: statusStr = "COAST"; break;
            case TrackStatus::DELETED: statusStr = "DEL"; break;
        }
        
        return "Track " + std::to_string(id) + 
               " [" + statusStr + "]" +
               " age=" + std::to_string(age) +
               " hits=" + std::to_string(hits) +
               " miss=" + std::to_string(misses) +
               " conf=" + std::to_string(static_cast<int>(confidence * 100)) + "%";
    }
};

/**
 * @brief Result of data association
 */
struct AssociationResult {
    // Track index -> Detection index mapping (-1 if no association)
    std::vector<int> trackToDetection;
    
    // Detection index -> Track index mapping (-1 if no association)
    std::vector<int> detectionToTrack;
    
    // Unassociated detection indices (for track initiation)
    std::vector<size_t> unassociatedDetections;
    
    // Unassociated track indices (for coasting)
    std::vector<size_t> unassociatedTracks;
    
    // Association probabilities (for JPDA)
    std::vector<std::vector<double>> associationProbabilities;
    
    // Association costs/distances
    std::vector<double> associationCosts;
    
    /**
     * @brief Check if track has association
     */
    bool hasAssociation(size_t trackIdx) const {
        return trackIdx < trackToDetection.size() && 
               trackToDetection[trackIdx] >= 0;
    }
    
    /**
     * @brief Get associated detection for track
     */
    int getDetectionForTrack(size_t trackIdx) const {
        return (trackIdx < trackToDetection.size()) 
               ? trackToDetection[trackIdx] : -1;
    }
    
    /**
     * @brief Get associated track for detection
     */
    int getTrackForDetection(size_t detIdx) const {
        return (detIdx < detectionToTrack.size()) 
               ? detectionToTrack[detIdx] : -1;
    }
};

} // namespace zoppler

#endif // ZOPPLER_CORE_TRACK_HPP
