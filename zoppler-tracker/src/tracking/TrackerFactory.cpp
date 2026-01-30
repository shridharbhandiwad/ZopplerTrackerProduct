/**
 * @file TrackerFactory.cpp
 * @brief Tracker factory implementation
 * @copyright Defense-grade radar tracking system
 */

#include "zoppler/tracking/TrackerFactory.hpp"
#include "zoppler/tracking/EKFTracker.hpp"
#include "zoppler/tracking/UKFTracker.hpp"
#include "zoppler/tracking/IMMTracker.hpp"
#include "zoppler/tracking/ParticleFilterTracker.hpp"
#include <algorithm>
#include <stdexcept>

namespace zoppler {

std::unique_ptr<ITrackerModel> TrackerFactory::create(const TrackerConfig& config) {
    TrackingAlgorithm algo = config.getTrackingAlgorithm();
    
    switch (algo) {
        case TrackingAlgorithm::CV:
            return std::make_unique<CVTracker>(config);
            
        case TrackingAlgorithm::CA:
            return std::make_unique<CATracker>(config);
            
        case TrackingAlgorithm::CT:
            return std::make_unique<CTTracker>(config);
            
        case TrackingAlgorithm::EKF:
            return std::make_unique<EKFTracker>(config);
            
        case TrackingAlgorithm::UKF:
            return std::make_unique<UKFTracker>(config);
            
        case TrackingAlgorithm::IMM:
            return std::make_unique<IMMTracker>(config);
            
        case TrackingAlgorithm::PARTICLE_FILTER:
            return std::make_unique<ParticleFilterTracker>(config);
            
        default:
            return std::make_unique<EKFTracker>(config);
    }
}

std::unique_ptr<ITrackerModel> TrackerFactory::create(const std::string& algorithmName,
                                                       const std::string& motionModelStr) {
    std::string algo = algorithmName;
    std::transform(algo.begin(), algo.end(), algo.begin(), ::toupper);
    
    MotionModel motionModel = stringToMotionModel(motionModelStr);
    bool is3D = true;
    
    if (algo == "CV") {
        return std::make_unique<CVTracker>(is3D);
    } else if (algo == "CA") {
        return std::make_unique<CATracker>(is3D);
    } else if (algo == "CT") {
        return std::make_unique<CTTracker>(is3D);
    } else if (algo == "EKF") {
        return std::make_unique<EKFTracker>(motionModel, is3D);
    } else if (algo == "UKF") {
        return std::make_unique<UKFTracker>(motionModel, is3D);
    } else if (algo == "IMM") {
        return std::make_unique<IMMTracker>();
    } else if (algo == "PARTICLE_FILTER" || algo == "PF") {
        return std::make_unique<ParticleFilterTracker>(500, motionModel, is3D);
    }
    
    throw std::invalid_argument("Unknown tracking algorithm: " + algorithmName);
}

bool TrackerFactory::isValidAlgorithm(const std::string& algorithmName) {
    std::string name = algorithmName;
    std::transform(name.begin(), name.end(), name.begin(), ::toupper);
    
    return name == "CV" || name == "CA" || name == "CT" ||
           name == "EKF" || name == "UKF" || name == "IMM" ||
           name == "PARTICLE_FILTER" || name == "PF";
}

std::vector<std::string> TrackerFactory::getAvailableAlgorithms() {
    return {"CV", "CA", "CT", "EKF", "UKF", "IMM", "PARTICLE_FILTER"};
}

std::vector<std::string> TrackerFactory::getAvailableMotionModels() {
    return {"CV", "CA", "CT"};
}

} // namespace zoppler
