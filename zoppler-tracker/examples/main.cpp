/**
 * @file main.cpp
 * @brief Zoppler Tracker demo application
 * @copyright Defense-grade radar tracking system
 * 
 * This demo demonstrates the complete tracking pipeline:
 * - Configuration setup
 * - Detection processing
 * - Track management
 * - JSON save/load
 */

#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <random>

#include "zoppler/api/TrackerAPI.hpp"
#include "zoppler/config/JsonLoader.hpp"
#include "zoppler/config/JsonSaver.hpp"

using namespace zoppler;

/**
 * @brief Generate simulated detections for a target
 */
std::vector<Detection> generateDetections(
    double targetX, double targetY, double targetZ,
    double vx, double vy, double vz,
    int numDetections,
    double dt,
    Timestamp startTime,
    double noiseStd = 2.0,
    double detectionProb = 0.9) {
    
    std::vector<Detection> detections;
    std::mt19937 rng(42);
    std::normal_distribution<double> noise(0.0, noiseStd);
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    
    double x = targetX;
    double y = targetY;
    double z = targetZ;
    
    for (int i = 0; i < numDetections; ++i) {
        // Update true position
        x += vx * dt;
        y += vy * dt;
        z += vz * dt;
        
        // Simulate missed detection
        if (uniform(rng) > detectionProb) {
            continue;
        }
        
        // Create detection with noise
        Detection det;
        det.x = x + noise(rng);
        det.y = y + noise(rng);
        det.z = z + noise(rng);
        det.timestamp = startTime + static_cast<Timestamp>(i * dt * 1e6);
        det.snr = 15.0 + noise(rng) * 0.5;
        det.quality = 0.9;
        
        det.computePolarFromCartesian();
        
        detections.push_back(det);
    }
    
    return detections;
}

/**
 * @brief Print track state
 */
void printTrackState(const TrackState& state, int id) {
    std::cout << "  Track " << id << ": "
              << "pos=(" << std::fixed << std::setprecision(1)
              << state.x << ", " << state.y << ", " << state.z << ") "
              << "vel=(" << std::setprecision(1)
              << state.vx << ", " << state.vy << ", " << state.vz << ") "
              << "speed=" << std::setprecision(1) << state.getSpeed() << " m/s"
              << std::endl;
}

/**
 * @brief Demo 1: Basic tracking with default configuration
 */
void demoBasicTracking() {
    std::cout << "\n=== Demo 1: Basic Tracking ===" << std::endl;
    
    // Create tracker with default short-range CUAS preset
    TrackerConfig config = TrackerConfig::shortRangeCUAS();
    TrackerAPI tracker(config);
    
    std::cout << "Tracker initialized with:" << std::endl;
    std::cout << "  Clustering: " << config.clusteringAlgo << std::endl;
    std::cout << "  Association: " << config.associationAlgo << std::endl;
    std::cout << "  Tracking: " << config.trackingAlgo << std::endl;
    std::cout << "  Motion Model: " << config.motionModel << std::endl;
    std::cout << "  Profile: " << config.profile << std::endl;
    
    // Generate simulated target
    // Target 1: Moving northeast
    auto detections1 = generateDetections(
        100.0, 100.0, 50.0,   // Initial position
        20.0, 15.0, 0.5,      // Velocity (m/s)
        50,                    // Number of scans
        0.1,                   // Update time (s)
        1000000,               // Start timestamp
        2.0,                   // Noise std
        0.9                    // Detection probability
    );
    
    // Target 2: Moving east
    auto detections2 = generateDetections(
        -200.0, 50.0, 100.0,
        30.0, 0.0, -1.0,
        50,
        0.1,
        1000000,
        2.0,
        0.85
    );
    
    // Process detections scan by scan
    std::cout << "\nProcessing " << (detections1.size() + detections2.size()) 
              << " detections..." << std::endl;
    
    Timestamp currentTime = 1000000;
    double dt = 0.1;
    
    for (int scan = 0; scan < 50; ++scan) {
        std::vector<Detection> scanDetections;
        
        // Add detections from this scan
        for (const auto& det : detections1) {
            if (det.timestamp >= currentTime && 
                det.timestamp < currentTime + static_cast<Timestamp>(dt * 1e6)) {
                scanDetections.push_back(det);
            }
        }
        for (const auto& det : detections2) {
            if (det.timestamp >= currentTime && 
                det.timestamp < currentTime + static_cast<Timestamp>(dt * 1e6)) {
                scanDetections.push_back(det);
            }
        }
        
        // Process
        tracker.processDetections(scanDetections, currentTime);
        currentTime += static_cast<Timestamp>(dt * 1e6);
        
        // Print status every 10 scans
        if ((scan + 1) % 10 == 0) {
            std::cout << "Scan " << (scan + 1) << ": "
                      << tracker.getNumTracks() << " tracks ("
                      << tracker.getNumConfirmedTracks() << " confirmed)" 
                      << std::endl;
        }
    }
    
    // Final results
    std::cout << "\nFinal tracks:" << std::endl;
    auto tracks = tracker.getTracks();
    for (size_t i = 0; i < tracks.size(); ++i) {
        printTrackState(tracks[i], static_cast<int>(i + 1));
    }
    
    // Statistics
    auto stats = tracker.getStatistics();
    std::cout << "\nStatistics:" << std::endl;
    std::cout << "  Total scans: " << stats.totalScans << std::endl;
    std::cout << "  Total detections: " << stats.totalDetections << std::endl;
    std::cout << "  Tracks initiated: " << stats.tracksInitiated << std::endl;
    std::cout << "  Tracks deleted: " << stats.tracksDeleted << std::endl;
    std::cout << "  Last processing time: " << stats.lastProcessingTimeMs << " ms" << std::endl;
}

/**
 * @brief Demo 2: Different algorithm combinations
 */
void demoAlgorithmCombinations() {
    std::cout << "\n=== Demo 2: Algorithm Combinations ===" << std::endl;
    
    // Generate a single target trajectory
    auto detections = generateDetections(
        0.0, 0.0, 100.0,
        25.0, 25.0, 0.0,
        30,
        0.1,
        2000000
    );
    
    // Test different combinations
    struct Combination {
        std::string name;
        std::string clustering;
        std::string association;
        std::string tracking;
        std::string motion;
    };
    
    std::vector<Combination> combinations = {
        {"DBSCAN + GNN + EKF(CV)", "DBSCAN", "GNN", "EKF", "CV"},
        {"DBSCAN + JPDA + EKF(CA)", "DBSCAN", "JPDA", "EKF", "CA"},
        {"RANGE + GNN + UKF(CV)", "CONTINUOUS_RANGE", "GNN", "UKF", "CV"},
        {"DBSCAN + GNN + IMM", "DBSCAN", "GNN", "IMM", "IMM"},
    };
    
    for (const auto& combo : combinations) {
        TrackerConfig config;
        config.clusteringAlgo = combo.clustering;
        config.associationAlgo = combo.association;
        config.trackingAlgo = combo.tracking;
        config.motionModel = combo.motion;
        config.profile = "SHORT_RANGE";
        config.resolveProfile();
        
        TrackerAPI tracker(config);
        
        // Process all detections
        for (const auto& det : detections) {
            std::vector<Detection> scan = {det};
            tracker.processDetections(scan, det.timestamp);
        }
        
        std::cout << combo.name << ": " 
                  << tracker.getNumConfirmedTracks() << " confirmed tracks" 
                  << std::endl;
        
        auto tracks = tracker.getConfirmedTracks();
        if (!tracks.empty()) {
            std::cout << "  Final position: (" 
                      << std::fixed << std::setprecision(1)
                      << tracks[0].x << ", " 
                      << tracks[0].y << ", " 
                      << tracks[0].z << ")" << std::endl;
        }
    }
}

/**
 * @brief Demo 3: JSON configuration
 */
void demoJsonConfig() {
    std::cout << "\n=== Demo 3: JSON Configuration ===" << std::endl;
    
    // Create a configuration
    TrackerConfig config = TrackerConfig::mediumRangeSurveillance();
    
    // Save to string
    std::string jsonStr = JsonSaver::saveToString(config);
    std::cout << "Generated JSON configuration:" << std::endl;
    std::cout << jsonStr.substr(0, 500) << "..." << std::endl;
    
    // Load back
    TrackerConfig loadedConfig = JsonLoader::loadFromString(jsonStr);
    
    std::cout << "\nLoaded configuration:" << std::endl;
    std::cout << "  Clustering: " << loadedConfig.clusteringAlgo << std::endl;
    std::cout << "  Association: " << loadedConfig.associationAlgo << std::endl;
    std::cout << "  Tracking: " << loadedConfig.trackingAlgo << std::endl;
    
    // Initialize tracker with loaded config
    TrackerAPI tracker(loadedConfig);
    std::cout << "\nTracker initialized from JSON: " 
              << (tracker.isInitialized() ? "SUCCESS" : "FAILED") << std::endl;
}

/**
 * @brief Demo 4: Preset configurations
 */
void demoPresets() {
    std::cout << "\n=== Demo 4: Preset Configurations ===" << std::endl;
    
    std::cout << "\n1. Short-Range FMCW / CUAS:" << std::endl;
    TrackerConfig cuas = TrackerConfig::shortRangeCUAS();
    std::cout << "   " << cuas.clusteringAlgo << " + " 
              << cuas.associationAlgo << " + "
              << cuas.trackingAlgo << "(" << cuas.motionModel << ")" << std::endl;
    
    std::cout << "\n2. Medium-Range Surveillance:" << std::endl;
    TrackerConfig medium = TrackerConfig::mediumRangeSurveillance();
    std::cout << "   " << medium.clusteringAlgo << " + " 
              << medium.associationAlgo << " + "
              << medium.trackingAlgo << std::endl;
    
    std::cout << "\n3. Long-Range / Crossing Targets:" << std::endl;
    TrackerConfig longRange = TrackerConfig::longRangeCrossing();
    std::cout << "   " << longRange.clusteringAlgo << " + " 
              << longRange.associationAlgo << " + "
              << longRange.trackingAlgo << std::endl;
    
    std::cout << "\n4. Clutter-Heavy (Birds):" << std::endl;
    TrackerConfig clutter = TrackerConfig::clutterHeavyBirds();
    std::cout << "   " << clutter.clusteringAlgo << " + " 
              << clutter.associationAlgo << " + "
              << clutter.trackingAlgo << std::endl;
}

/**
 * @brief Main entry point
 */
int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "   Zoppler Tracker - Demo Application   " << std::endl;
    std::cout << "   Defense-Grade Multi-Target Tracker   " << std::endl;
    std::cout << "========================================" << std::endl;
    
    try {
        demoBasicTracking();
        demoAlgorithmCombinations();
        demoJsonConfig();
        demoPresets();
        
        std::cout << "\n========================================" << std::endl;
        std::cout << "   All demos completed successfully!    " << std::endl;
        std::cout << "========================================" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
