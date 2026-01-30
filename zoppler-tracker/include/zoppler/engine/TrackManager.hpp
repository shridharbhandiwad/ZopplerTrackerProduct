/**
 * @file TrackManager.hpp
 * @brief Track lifecycle management
 * @copyright Defense-grade radar tracking system
 */

#ifndef ZOPPLER_ENGINE_TRACKMANAGER_HPP
#define ZOPPLER_ENGINE_TRACKMANAGER_HPP

#include "../core/Track.hpp"
#include "../core/Detection.hpp"
#include "../core/TrackerConfig.hpp"
#include "../tracking/TrackerFactory.hpp"
#include <vector>
#include <memory>
#include <mutex>

namespace zoppler {

/**
 * @brief Manages track lifecycle: initiation, confirmation, deletion
 * 
 * Handles M/N confirmation logic, confidence scoring,
 * track aging, and track deletion policies.
 */
class TrackManager {
public:
    /**
     * @brief Construct with configuration
     */
    explicit TrackManager(const TrackerConfig& config);
    
    /**
     * @brief Default destructor
     */
    ~TrackManager() = default;
    
    // Non-copyable
    TrackManager(const TrackManager&) = delete;
    TrackManager& operator=(const TrackManager&) = delete;
    
    // Movable
    TrackManager(TrackManager&&) = default;
    TrackManager& operator=(TrackManager&&) = default;
    
    /**
     * @brief Initiate new tracks from unassociated detections
     * 
     * @param detections Unassociated detections for initiation
     * @param timestamp Current timestamp
     * @return Number of tracks initiated
     */
    int initiateTracksFromDetections(const std::vector<Detection>& detections,
                                     Timestamp timestamp);
    
    /**
     * @brief Update track status after association
     * 
     * @param trackIdx Track index
     * @param associated Whether track was associated this scan
     * @param timestamp Current timestamp
     */
    void updateTrackStatus(size_t trackIdx, bool associated, Timestamp timestamp);
    
    /**
     * @brief Perform maintenance: delete stale tracks
     * 
     * @param currentTime Current timestamp
     * @return Number of tracks deleted
     */
    int performMaintenance(Timestamp currentTime);
    
    /**
     * @brief Get reference to tracks
     */
    std::vector<Track>& getTracks() { return tracks_; }
    const std::vector<Track>& getTracks() const { return tracks_; }
    
    /**
     * @brief Get track by ID
     */
    Track* getTrackById(TrackId id);
    const Track* getTrackById(TrackId id) const;
    
    /**
     * @brief Get number of active tracks
     */
    size_t getNumActiveTracks() const;
    
    /**
     * @brief Get number of confirmed tracks
     */
    size_t getNumConfirmedTracks() const;
    
    /**
     * @brief Clear all tracks
     */
    void clearTracks();
    
    /**
     * @brief Reset track ID counter
     */
    void reset();
    
    // Configuration updates
    void setConfirmationLogic(int M, int N);
    void setMaxMisses(int maxMisses);
    void setMinConfidence(double minConfidence);
    void setConfidenceParameters(double gain, double loss, double initial);
    
private:
    /**
     * @brief Create measurement from detection
     */
    Measurement createMeasurement(const Detection& detection) const;
    
    /**
     * @brief Check if detection passes initiation criteria
     */
    bool passesInitiationCriteria(const Detection& detection) const;
    
    /**
     * @brief Allocate new track ID
     */
    TrackId allocateTrackId();
    
    TrackerConfig config_;
    std::vector<Track> tracks_;
    
    // Track ID allocation
    TrackId nextTrackId_ = 1;
    
    // M/N confirmation logic
    int confirmM_ = 3;          // Hits required (M)
    int confirmN_ = 5;          // Window size (N)
    int maxMisses_ = 5;         // Max consecutive misses
    int tentativeMaxMisses_ = 2; // Max misses for tentative tracks
    
    // Confidence parameters
    double confidenceGain_ = 0.2;
    double confidenceLoss_ = 0.1;
    double initialConfidence_ = 0.3;
    double minConfidence_ = 0.1;
    
    // Initiation criteria
    double minInitSNR_ = 5.0;
    
    // Max tracks limit
    size_t maxTracks_ = 500;
    
    // Thread safety
    mutable std::mutex mutex_;
};

} // namespace zoppler

#endif // ZOPPLER_ENGINE_TRACKMANAGER_HPP
