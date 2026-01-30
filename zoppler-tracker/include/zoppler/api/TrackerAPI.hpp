/**
 * @file TrackerAPI.hpp
 * @brief Qt-friendly public API for the tracker
 * @copyright Defense-grade radar tracking system
 */

#ifndef ZOPPLER_API_TRACKERAPI_HPP
#define ZOPPLER_API_TRACKERAPI_HPP

#include "../core/TrackerConfig.hpp"
#include "../core/Detection.hpp"
#include "../core/TrackState.hpp"
#include "../engine/TrackerEngine.hpp"
#include <memory>
#include <vector>
#include <mutex>

namespace zoppler {

/**
 * @brief Qt-friendly API wrapper for the tracker engine
 * 
 * Provides a simple, thread-safe interface suitable for integration
 * with Qt applications using signals/slots.
 * 
 * Thread-safety: All public methods are thread-safe.
 * Memory: No shared mutable state with client code.
 * Qt Integration: Compatible with QVector, QObject signals/slots.
 */
class TrackerAPI {
public:
    /**
     * @brief Default constructor (uninitialized)
     */
    TrackerAPI();
    
    /**
     * @brief Construct and initialize with configuration
     */
    explicit TrackerAPI(const TrackerConfig& config);
    
    /**
     * @brief Destructor
     */
    ~TrackerAPI();
    
    // Non-copyable
    TrackerAPI(const TrackerAPI&) = delete;
    TrackerAPI& operator=(const TrackerAPI&) = delete;
    
    // Movable
    TrackerAPI(TrackerAPI&&) noexcept;
    TrackerAPI& operator=(TrackerAPI&&) noexcept;
    
    /**
     * @brief Initialize or reinitialize the tracker
     * 
     * @param config Tracker configuration
     * @return true if successful
     */
    bool initialize(const TrackerConfig& config);
    
    /**
     * @brief Check if tracker is initialized
     */
    bool isInitialized() const;
    
    /**
     * @brief Process detections (main entry point)
     * 
     * Thread-safe. Can be called from any thread.
     * 
     * @param detections Vector of detections (value semantics for Qt compatibility)
     * @param timestamp Timestamp in microseconds since epoch
     */
    void processDetections(const std::vector<Detection>& detections, int64_t timestamp);
    
    /**
     * @brief Get current track states
     * 
     * Returns by value for thread safety and Qt compatibility.
     * 
     * @return Vector of track states
     */
    std::vector<TrackState> getTracks() const;
    
    /**
     * @brief Get confirmed track states only
     */
    std::vector<TrackState> getConfirmedTracks() const;
    
    /**
     * @brief Get track IDs
     */
    std::vector<int> getTrackIds() const;
    
    /**
     * @brief Get track state by ID
     * 
     * @param trackId Track identifier
     * @param state Output track state
     * @return true if track found
     */
    bool getTrackById(int trackId, TrackState& state) const;
    
    /**
     * @brief Get number of active tracks
     */
    int getNumTracks() const;
    
    /**
     * @brief Get number of confirmed tracks
     */
    int getNumConfirmedTracks() const;
    
    /**
     * @brief Reset the tracker (clear all tracks)
     */
    void reset();
    
    /**
     * @brief Get current configuration
     */
    TrackerConfig getConfig() const;
    
    /**
     * @brief Update configuration
     * 
     * This will reinitialize the tracker.
     */
    void setConfig(const TrackerConfig& config);
    
    // Convenience methods for configuration
    
    /**
     * @brief Set clustering algorithm
     * @param algorithm "DBSCAN" or "CONTINUOUS_RANGE"
     */
    void setClusteringAlgorithm(const std::string& algorithm);
    
    /**
     * @brief Set association algorithm
     * @param algorithm "GNN", "JPDA", or "MHT"
     */
    void setAssociationAlgorithm(const std::string& algorithm);
    
    /**
     * @brief Set tracking algorithm
     * @param algorithm "EKF", "UKF", "IMM", or "PARTICLE_FILTER"
     */
    void setTrackingAlgorithm(const std::string& algorithm);
    
    /**
     * @brief Set motion model
     * @param model "CV", "CA", or "CT"
     */
    void setMotionModel(const std::string& model);
    
    /**
     * @brief Set profile
     * @param profile "SHORT_RANGE", "MEDIUM_RANGE", "LONG_RANGE", or "CLUTTER_HEAVY"
     */
    void setProfile(const std::string& profile);
    
    /**
     * @brief Statistics structure
     */
    struct Statistics {
        uint64_t totalScans;
        uint64_t totalDetections;
        uint64_t tracksInitiated;
        uint64_t tracksDeleted;
        double lastProcessingTimeMs;
        int64_t lastTimestamp;
    };
    
    /**
     * @brief Get processing statistics
     */
    Statistics getStatistics() const;
    
    /**
     * @brief Load configuration from JSON file
     * 
     * @param filepath Path to JSON file
     * @return true if successful
     */
    bool loadConfigFromFile(const std::string& filepath);
    
    /**
     * @brief Save current configuration to JSON file
     * 
     * @param filepath Path to JSON file
     * @return true if successful
     */
    bool saveConfigToFile(const std::string& filepath) const;
    
    /**
     * @brief Load configuration from JSON string
     * 
     * @param jsonString JSON configuration string
     * @return true if successful
     */
    bool loadConfigFromString(const std::string& jsonString);
    
    /**
     * @brief Get current configuration as JSON string
     */
    std::string getConfigAsJson() const;
    
private:
    class Impl;
    std::unique_ptr<Impl> pImpl_;
};

} // namespace zoppler

#endif // ZOPPLER_API_TRACKERAPI_HPP
