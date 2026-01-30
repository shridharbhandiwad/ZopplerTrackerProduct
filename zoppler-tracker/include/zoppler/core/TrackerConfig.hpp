/**
 * @file TrackerConfig.hpp
 * @brief Main tracker configuration structure
 * @copyright Defense-grade radar tracking system
 */

#ifndef ZOPPLER_CORE_TRACKERCONFIG_HPP
#define ZOPPLER_CORE_TRACKERCONFIG_HPP

#include "Types.hpp"
#include "RadarConfig.hpp"
#include "TrackerProfile.hpp"
#include <string>

namespace zoppler {

/**
 * @brief Complete tracker configuration
 * 
 * This is the main configuration structure that maps directly
 * to the UI and determines the complete tracker pipeline.
 */
struct TrackerConfig {
    // Algorithm selections (as strings for UI mapping)
    std::string clusteringAlgo = "DBSCAN";
    std::string associationAlgo = "GNN";
    std::string trackingAlgo = "EKF";
    std::string motionModel = "CV";
    std::string targetType = "DRONE";
    std::string profile = "SHORT_RANGE";
    
    // Radar configuration
    RadarConfig radar;
    
    // Resolved tracker profile
    TrackerProfile trackerProfile;
    
    // Clustering-specific parameters
    struct ClusteringParams {
        // DBSCAN parameters
        double epsilon = 10.0;          // DBSCAN neighborhood radius (meters)
        int minPoints = 2;              // DBSCAN minimum points
        
        // Continuous range clustering parameters
        double rangeGate = 5.0;         // Range gate size (meters)
        double azimuthGate = 0.02;      // Azimuth gate size (radians)
        double elevationGate = 0.05;    // Elevation gate size (radians)
        double dopplerGate = 2.0;       // Doppler gate size (m/s)
        
        // General parameters
        int maxClusters = 100;          // Maximum number of clusters
        bool use3D = true;              // Use 3D clustering
        bool useDoppler = true;         // Use Doppler in clustering
    } clusteringParams;
    
    // Association-specific parameters
    struct AssociationParams {
        // GNN parameters
        double maxAssociationDistance = 50.0;  // Max distance for association (meters)
        
        // JPDA parameters
        double jpdaPD = 0.9;            // Detection probability for JPDA
        double jpdaClutterRate = 1e-6;  // Clutter rate for JPDA
        int jpdaMaxHypotheses = 100;    // Max hypotheses to consider
        
        // MHT parameters
        int mhtNScan = 3;               // N-scan pruning depth
        int mhtMaxHypotheses = 1000;    // Max hypotheses
        int mhtMaxTracksPerHypothesis = 50;
        double mhtPruneThreshold = 0.01; // Hypothesis pruning threshold
    } associationParams;
    
    // Tracking-specific parameters
    struct TrackingParams {
        // EKF/UKF parameters
        int stateDimension = 6;         // State vector dimension
        int measurementDimension = 3;   // Measurement dimension
        
        // UKF parameters
        double ukfAlpha = 0.001;        // Sigma point spread
        double ukfBeta = 2.0;           // Prior knowledge (2 for Gaussian)
        double ukfKappa = 0.0;          // Secondary scaling
        
        // IMM parameters
        std::vector<std::string> immModels = {"CV", "CA"};  // Models for IMM
        std::vector<double> immInitialProbs = {0.5, 0.5};   // Initial probabilities
        // Markov transition matrix (row-major)
        std::vector<double> immTransitionMatrix = {0.98, 0.02, 0.02, 0.98};
        
        // Particle filter parameters
        int pfNumParticles = 500;       // Number of particles
        double pfResampleThreshold = 0.5; // Effective sample size threshold
        
        // General tracking parameters
        bool usePolarMeasurement = false;  // Use polar coordinates for update
    } trackingParams;
    
    // Track management parameters
    struct TrackManagementParams {
        int minHitsToConfirm = 3;       // M in M/N logic
        int confirmWindow = 5;          // N in M/N logic
        int maxConsecutiveMisses = 5;   // Misses before deletion
        double minConfidence = 0.1;     // Minimum track confidence
        double maxExtrapolationTime = 10.0;  // Max coast time (seconds)
        int maxTracks = 500;            // Maximum simultaneous tracks
        bool deleteOutOfRange = true;   // Delete tracks outside radar range
    } trackManagement;
    
    /**
     * @brief Default constructor
     */
    TrackerConfig() {
        resolveProfile();
    }
    
    /**
     * @brief Resolve profile and update parameters
     */
    void resolveProfile() {
        trackerProfile = TrackerProfile::fromString(profile);
        
        // Update track management from profile
        trackManagement.minHitsToConfirm = trackerProfile.confirmHits;
        trackManagement.confirmWindow = trackerProfile.confirmWindow;
        trackManagement.maxConsecutiveMisses = trackerProfile.maxMisses;
        trackManagement.maxExtrapolationTime = trackerProfile.maxExtrapolationTime;
    }
    
    /**
     * @brief Get resolved clustering algorithm enum
     */
    ClusteringAlgorithm getClusteringAlgorithm() const {
        if (clusteringAlgo == "CONTINUOUS_RANGE" || clusteringAlgo == "RANGE_BINS") {
            return ClusteringAlgorithm::CONTINUOUS_RANGE;
        }
        return ClusteringAlgorithm::DBSCAN;
    }
    
    /**
     * @brief Get resolved association algorithm enum
     */
    AssociationAlgorithm getAssociationAlgorithm() const {
        if (associationAlgo == "JPDA") return AssociationAlgorithm::JPDA;
        if (associationAlgo == "MHT") return AssociationAlgorithm::MHT;
        return AssociationAlgorithm::GNN;
    }
    
    /**
     * @brief Get resolved tracking algorithm enum
     */
    TrackingAlgorithm getTrackingAlgorithm() const {
        if (trackingAlgo == "CV") return TrackingAlgorithm::CV;
        if (trackingAlgo == "CA") return TrackingAlgorithm::CA;
        if (trackingAlgo == "CT") return TrackingAlgorithm::CT;
        if (trackingAlgo == "IMM") return TrackingAlgorithm::IMM;
        if (trackingAlgo == "UKF") return TrackingAlgorithm::UKF;
        if (trackingAlgo == "PARTICLE_FILTER" || trackingAlgo == "PF") {
            return TrackingAlgorithm::PARTICLE_FILTER;
        }
        return TrackingAlgorithm::EKF;
    }
    
    /**
     * @brief Get resolved motion model enum
     */
    MotionModel getMotionModel() const {
        return stringToMotionModel(motionModel);
    }
    
    /**
     * @brief Get resolved target type enum
     */
    TargetType getTargetType() const {
        return stringToTargetType(targetType);
    }
    
    /**
     * @brief Validate configuration
     */
    bool isValid() const {
        return radar.isValid() &&
               trackManagement.maxTracks > 0 &&
               trackManagement.minHitsToConfirm > 0;
    }
    
    /**
     * @brief Create preset for Short-Range FMCW / CUAS
     */
    static TrackerConfig shortRangeCUAS() {
        TrackerConfig cfg;
        cfg.clusteringAlgo = "DBSCAN";
        cfg.associationAlgo = "GNN";
        cfg.trackingAlgo = "EKF";
        cfg.motionModel = "CA";
        cfg.targetType = "DRONE";
        cfg.profile = "SHORT_RANGE";
        cfg.radar = RadarConfig::shortRangeFMCW();
        cfg.resolveProfile();
        return cfg;
    }
    
    /**
     * @brief Create preset for Medium-Range Surveillance
     */
    static TrackerConfig mediumRangeSurveillance() {
        TrackerConfig cfg;
        cfg.clusteringAlgo = "CONTINUOUS_RANGE";
        cfg.associationAlgo = "JPDA";
        cfg.trackingAlgo = "IMM";
        cfg.motionModel = "IMM";
        cfg.trackingParams.immModels = {"CV", "CA"};
        cfg.targetType = "AIRCRAFT";
        cfg.profile = "MEDIUM_RANGE";
        cfg.radar = RadarConfig::mediumRangeSurveillance();
        cfg.resolveProfile();
        return cfg;
    }
    
    /**
     * @brief Create preset for Long-Range / Crossing Targets
     */
    static TrackerConfig longRangeCrossing() {
        TrackerConfig cfg;
        cfg.clusteringAlgo = "CONTINUOUS_RANGE";
        cfg.associationAlgo = "MHT";
        cfg.trackingAlgo = "IMM";
        cfg.motionModel = "IMM";
        cfg.trackingParams.immModels = {"CV", "CA", "CT"};
        cfg.trackingParams.immInitialProbs = {0.4, 0.3, 0.3};
        cfg.targetType = "AIRCRAFT";
        cfg.profile = "LONG_RANGE";
        cfg.radar = RadarConfig::longRange();
        cfg.resolveProfile();
        return cfg;
    }
    
    /**
     * @brief Create preset for Clutter-Heavy (Birds)
     */
    static TrackerConfig clutterHeavyBirds() {
        TrackerConfig cfg;
        cfg.clusteringAlgo = "DBSCAN";
        cfg.associationAlgo = "JPDA";
        cfg.trackingAlgo = "PARTICLE_FILTER";
        cfg.motionModel = "CV";
        cfg.trackingParams.pfNumParticles = 200;
        cfg.targetType = "BIRD";
        cfg.profile = "CLUTTER_HEAVY";
        cfg.radar = RadarConfig::mediumRangeSurveillance();
        cfg.radar.pD = 0.7;  // Lower pD for birds
        cfg.resolveProfile();
        return cfg;
    }
};

} // namespace zoppler

#endif // ZOPPLER_CORE_TRACKERCONFIG_HPP
