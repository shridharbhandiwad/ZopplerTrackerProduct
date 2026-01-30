/**
 * @file TrackerProfile.hpp
 * @brief Tracker parameter profiles for different operational scenarios
 * @copyright Defense-grade radar tracking system
 */

#ifndef ZOPPLER_CORE_TRACKERPROFILE_HPP
#define ZOPPLER_CORE_TRACKERPROFILE_HPP

#include "Types.hpp"
#include <string>

namespace zoppler {

/**
 * @brief Profile containing tuned tracker parameters
 * 
 * Pre-configured parameter sets for different operational scenarios.
 * These have been validated for typical use cases.
 */
struct TrackerProfile {
    // Profile identification
    ProfileType type = ProfileType::SHORT_RANGE;
    std::string name = "SHORT_RANGE";
    
    // Process noise parameters (m^2/s^4 or appropriate units)
    double qPos = 1.0;                  // Position process noise
    double qVel = 10.0;                 // Velocity process noise
    double qAcc = 50.0;                 // Acceleration process noise (for CA model)
    double qOmega = 0.1;                // Turn rate process noise (for CT model)
    
    // Gating parameters
    double gateChiSq = 9.21;            // Chi-squared gate threshold (99% for 2 DOF)
    double gateChiSq3D = 11.34;         // Chi-squared gate threshold (99% for 3 DOF)
    double maxGateDistance = 100.0;     // Maximum gate distance (meters)
    
    // Track management parameters
    int confirmHits = 3;                // Hits required for confirmation (M)
    int confirmWindow = 5;              // Window for M/N logic (N)
    int maxMisses = 5;                  // Max consecutive misses before deletion
    int tentativeMaxMisses = 2;         // Max misses for tentative tracks
    double minConfidence = 0.1;         // Minimum confidence before deletion
    
    // Initiation parameters
    double initVelStd = 50.0;           // Initial velocity uncertainty (m/s)
    double initAccStd = 10.0;           // Initial acceleration uncertainty (m/s^2)
    double minInitSNR = 10.0;           // Minimum SNR for track initiation (dB)
    
    // Confidence parameters
    double confidenceGain = 0.2;        // Confidence increase per hit
    double confidenceLoss = 0.1;        // Confidence decrease per miss
    double initialConfidence = 0.3;     // Initial track confidence
    
    // Prediction parameters
    double maxExtrapolationTime = 10.0; // Max time to coast without updates (s)
    double predictionCoastFactor = 1.5; // Increase in process noise during coast
    
    /**
     * @brief Default constructor
     */
    TrackerProfile() = default;
    
    /**
     * @brief Create SHORT_RANGE profile
     * 
     * Optimized for: FMCW radars, CUAS applications, fast-moving targets
     * - Aggressive initiation
     * - Higher process noise for maneuvering
     * - Fast deletion of stale tracks
     */
    static TrackerProfile shortRange() {
        TrackerProfile p;
        p.type = ProfileType::SHORT_RANGE;
        p.name = "SHORT_RANGE";
        
        // Higher process noise for maneuvering targets
        p.qPos = 2.0;
        p.qVel = 50.0;
        p.qAcc = 100.0;
        p.qOmega = 0.5;
        
        // Wider gates for fast updates
        p.gateChiSq = 12.0;
        p.gateChiSq3D = 14.0;
        p.maxGateDistance = 50.0;
        
        // Aggressive track management
        p.confirmHits = 2;
        p.confirmWindow = 3;
        p.maxMisses = 3;
        p.tentativeMaxMisses = 1;
        
        // Quick initiation
        p.initVelStd = 100.0;
        p.initAccStd = 50.0;
        p.minInitSNR = 5.0;
        
        // Fast response confidence
        p.confidenceGain = 0.3;
        p.confidenceLoss = 0.15;
        p.initialConfidence = 0.4;
        
        p.maxExtrapolationTime = 3.0;
        p.predictionCoastFactor = 2.0;
        
        return p;
    }
    
    /**
     * @brief Create MEDIUM_RANGE profile
     * 
     * Balanced profile for general surveillance
     */
    static TrackerProfile mediumRange() {
        TrackerProfile p;
        p.type = ProfileType::MEDIUM_RANGE;
        p.name = "MEDIUM_RANGE";
        
        p.qPos = 1.0;
        p.qVel = 20.0;
        p.qAcc = 50.0;
        p.qOmega = 0.2;
        
        p.gateChiSq = 9.21;
        p.gateChiSq3D = 11.34;
        p.maxGateDistance = 200.0;
        
        p.confirmHits = 3;
        p.confirmWindow = 5;
        p.maxMisses = 5;
        p.tentativeMaxMisses = 2;
        
        p.initVelStd = 50.0;
        p.initAccStd = 20.0;
        p.minInitSNR = 8.0;
        
        p.confidenceGain = 0.2;
        p.confidenceLoss = 0.1;
        p.initialConfidence = 0.3;
        
        p.maxExtrapolationTime = 10.0;
        p.predictionCoastFactor = 1.5;
        
        return p;
    }
    
    /**
     * @brief Create LONG_RANGE profile
     * 
     * Optimized for: Large surveillance radars, stable tracks
     * - Conservative gating
     * - Lower process noise for stable tracks
     * - Longer coast times between updates
     */
    static TrackerProfile longRange() {
        TrackerProfile p;
        p.type = ProfileType::LONG_RANGE;
        p.name = "LONG_RANGE";
        
        // Lower process noise for stable targets
        p.qPos = 0.5;
        p.qVel = 5.0;
        p.qAcc = 10.0;
        p.qOmega = 0.05;
        
        // Conservative gates
        p.gateChiSq = 7.38;    // 95% for 2 DOF
        p.gateChiSq3D = 9.35;  // 95% for 3 DOF
        p.maxGateDistance = 500.0;
        
        // Conservative track management
        p.confirmHits = 4;
        p.confirmWindow = 6;
        p.maxMisses = 8;
        p.tentativeMaxMisses = 3;
        
        // Conservative initiation
        p.initVelStd = 30.0;
        p.initAccStd = 5.0;
        p.minInitSNR = 12.0;
        
        // Slow confidence changes for stability
        p.confidenceGain = 0.15;
        p.confidenceLoss = 0.05;
        p.initialConfidence = 0.25;
        
        p.maxExtrapolationTime = 30.0;
        p.predictionCoastFactor = 1.2;
        
        return p;
    }
    
    /**
     * @brief Create CLUTTER_HEAVY profile
     * 
     * Optimized for: Bird detection, high clutter environments
     * - Tighter gating to reduce false associations
     * - More conservative initiation
     * - Higher thresholds for confirmation
     */
    static TrackerProfile clutterHeavy() {
        TrackerProfile p;
        p.type = ProfileType::CLUTTER_HEAVY;
        p.name = "CLUTTER_HEAVY";
        
        // Moderate process noise
        p.qPos = 1.5;
        p.qVel = 15.0;
        p.qAcc = 30.0;
        p.qOmega = 0.3;
        
        // Tight gates to reduce clutter association
        p.gateChiSq = 6.63;    // 90% for 2 DOF
        p.gateChiSq3D = 7.81;  // 90% for 3 DOF
        p.maxGateDistance = 100.0;
        
        // More stringent confirmation
        p.confirmHits = 4;
        p.confirmWindow = 6;
        p.maxMisses = 4;
        p.tentativeMaxMisses = 1;
        
        // Conservative initiation
        p.initVelStd = 40.0;
        p.initAccStd = 15.0;
        p.minInitSNR = 15.0;  // Higher threshold in clutter
        
        // Moderate confidence parameters
        p.confidenceGain = 0.15;
        p.confidenceLoss = 0.2;  // Faster confidence decay
        p.initialConfidence = 0.2;
        
        p.maxExtrapolationTime = 5.0;
        p.predictionCoastFactor = 1.8;
        
        return p;
    }
    
    /**
     * @brief Get profile by type
     */
    static TrackerProfile fromType(ProfileType type) {
        switch (type) {
            case ProfileType::SHORT_RANGE: return shortRange();
            case ProfileType::MEDIUM_RANGE: return mediumRange();
            case ProfileType::LONG_RANGE: return longRange();
            case ProfileType::CLUTTER_HEAVY: return clutterHeavy();
            default: return shortRange();
        }
    }
    
    /**
     * @brief Get profile by string name
     */
    static TrackerProfile fromString(const std::string& name) {
        return fromType(stringToProfileType(name));
    }
};

} // namespace zoppler

#endif // ZOPPLER_CORE_TRACKERPROFILE_HPP
