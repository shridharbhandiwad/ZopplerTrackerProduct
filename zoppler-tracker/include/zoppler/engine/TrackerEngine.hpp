/**
 * @file TrackerEngine.hpp
 * @brief Main tracker engine orchestrating the tracking pipeline
 * @copyright Defense-grade radar tracking system
 */

#ifndef ZOPPLER_ENGINE_TRACKERENGINE_HPP
#define ZOPPLER_ENGINE_TRACKERENGINE_HPP

#include "../core/TrackerConfig.hpp"
#include "../core/Detection.hpp"
#include "../core/Track.hpp"
#include "../clustering/IClustering.hpp"
#include "../association/IDataAssociation.hpp"
#include "TrackManager.hpp"
#include <memory>
#include <mutex>

namespace zoppler {

/**
 * @brief Main tracker engine implementing the tracking pipeline
 * 
 * Pipeline: Detections → Clustering → Association → Predict → Update → Track Management
 * 
 * Thread-safe design suitable for integration with Qt or other frameworks.
 */
class TrackerEngine {
public:
    /**
     * @brief Construct tracker engine with configuration
     */
    explicit TrackerEngine(const TrackerConfig& config);
    
    /**
     * @brief Default destructor
     */
    ~TrackerEngine() = default;
    
    // Non-copyable
    TrackerEngine(const TrackerEngine&) = delete;
    TrackerEngine& operator=(const TrackerEngine&) = delete;
    
    // Movable
    TrackerEngine(TrackerEngine&&) = default;
    TrackerEngine& operator=(TrackerEngine&&) = default;
    
    /**
     * @brief Process new detections through the tracking pipeline
     * 
     * @param detections Raw radar detections
     * @param timestamp Current timestamp (microseconds since epoch)
     */
    void processDetections(const std::vector<Detection>& detections,
                          Timestamp timestamp);
    
    /**
     * @brief Get current tracks
     */
    const std::vector<Track>& getTracks() const;
    
    /**
     * @brief Get confirmed tracks only
     */
    std::vector<const Track*> getConfirmedTracks() const;
    
    /**
     * @brief Get track states (convenience method)
     */
    std::vector<TrackState> getTrackStates() const;
    
    /**
     * @brief Get track by ID
     */
    const Track* getTrackById(TrackId id) const;
    
    /**
     * @brief Get number of active tracks
     */
    size_t getNumTracks() const;
    
    /**
     * @brief Get number of confirmed tracks
     */
    size_t getNumConfirmedTracks() const;
    
    /**
     * @brief Reset the tracker (clear all tracks)
     */
    void reset();
    
    /**
     * @brief Reconfigure the tracker
     * 
     * @param config New configuration
     */
    void reconfigure(const TrackerConfig& config);
    
    /**
     * @brief Get current configuration
     */
    const TrackerConfig& getConfig() const { return config_; }
    
    /**
     * @brief Get last clusters (for debugging/visualization)
     */
    const std::vector<Cluster>& getLastClusters() const { return lastClusters_; }
    
    /**
     * @brief Get last association result (for debugging)
     */
    const AssociationResult& getLastAssociation() const { return lastAssociation_; }
    
    /**
     * @brief Get processing statistics
     */
    struct Statistics {
        uint64_t totalScans = 0;
        uint64_t totalDetections = 0;
        uint64_t totalClusters = 0;
        uint64_t tracksInitiated = 0;
        uint64_t tracksDeleted = 0;
        double lastProcessingTimeMs = 0.0;
        Timestamp lastTimestamp = 0;
    };
    
    const Statistics& getStatistics() const { return stats_; }
    
private:
    /**
     * @brief Perform clustering on raw detections
     */
    void performClustering(const std::vector<Detection>& detections);
    
    /**
     * @brief Perform prediction step for all tracks
     */
    void performPrediction(double dt);
    
    /**
     * @brief Perform data association
     */
    void performAssociation(const std::vector<Detection>& clusterCentroids);
    
    /**
     * @brief Perform measurement update for associated tracks
     */
    void performUpdate(const std::vector<Detection>& clusterCentroids,
                      Timestamp timestamp);
    
    /**
     * @brief Initiate new tracks from unassociated detections
     */
    void initiateNewTracks(const std::vector<Detection>& clusterCentroids,
                          Timestamp timestamp);
    
    /**
     * @brief Perform track maintenance
     */
    void performMaintenance(Timestamp timestamp);
    
    /**
     * @brief Create measurement from cluster centroid
     */
    Measurement createMeasurement(const Detection& centroid) const;
    
    TrackerConfig config_;
    
    // Algorithm components
    std::unique_ptr<IClustering> clustering_;
    std::unique_ptr<IDataAssociation> association_;
    std::unique_ptr<TrackManager> trackManager_;
    
    // Cached results
    std::vector<Cluster> lastClusters_;
    AssociationResult lastAssociation_;
    
    // Timing
    Timestamp lastProcessTime_ = 0;
    bool firstScan_ = true;
    
    // Statistics
    Statistics stats_;
    
    // Thread safety
    mutable std::mutex mutex_;
};

} // namespace zoppler

#endif // ZOPPLER_ENGINE_TRACKERENGINE_HPP
