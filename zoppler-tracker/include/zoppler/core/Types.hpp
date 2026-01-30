/**
 * @file Types.hpp
 * @brief Core type definitions for Zoppler Tracker
 * @copyright Defense-grade radar tracking system
 */

#ifndef ZOPPLER_CORE_TYPES_HPP
#define ZOPPLER_CORE_TYPES_HPP

#include <cstdint>
#include <string>
#include <vector>
#include <memory>
#include <limits>
#include <cmath>

namespace zoppler {

// Forward declarations
struct Detection;
struct Track;
struct Cluster;
struct Measurement;
struct TrackState;
struct RadarConfig;
struct TrackerConfig;
struct TrackerProfile;

// Type aliases for clarity
using TrackId = int;
using Timestamp = uint64_t;
using Probability = double;

// Constants
constexpr double PI = 3.14159265358979323846;
constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / PI;
constexpr double INVALID_VALUE = std::numeric_limits<double>::quiet_NaN();

// Target type enumeration
enum class TargetType {
    UNKNOWN,
    DRONE,
    BIRD,
    VEHICLE,
    AIRCRAFT,
    MISSILE,
    CLUTTER
};

// Motion model enumeration
enum class MotionModel {
    CV,     // Constant Velocity
    CA,     // Constant Acceleration
    CT,     // Coordinated Turn
    IMM     // Interacting Multiple Model
};

// Clustering algorithm enumeration
enum class ClusteringAlgorithm {
    CONTINUOUS_RANGE,
    DBSCAN
};

// Association algorithm enumeration
enum class AssociationAlgorithm {
    GNN,    // Global Nearest Neighbor
    JPDA,   // Joint Probabilistic Data Association
    MHT     // Multiple Hypothesis Tracking
};

// Tracking algorithm enumeration  
enum class TrackingAlgorithm {
    CV,
    CA,
    CT,
    IMM,
    EKF,
    UKF,
    PARTICLE_FILTER
};

// Profile type enumeration
enum class ProfileType {
    SHORT_RANGE,
    MEDIUM_RANGE,
    LONG_RANGE,
    CLUTTER_HEAVY
};

// Track status enumeration
enum class TrackStatus {
    TENTATIVE,      // Not yet confirmed
    CONFIRMED,      // M/N logic confirmed
    COASTING,       // No updates, coasting
    DELETED         // Marked for deletion
};

// Helper functions for enum conversions
inline std::string targetTypeToString(TargetType type) {
    switch (type) {
        case TargetType::DRONE: return "DRONE";
        case TargetType::BIRD: return "BIRD";
        case TargetType::VEHICLE: return "VEHICLE";
        case TargetType::AIRCRAFT: return "AIRCRAFT";
        case TargetType::MISSILE: return "MISSILE";
        case TargetType::CLUTTER: return "CLUTTER";
        default: return "UNKNOWN";
    }
}

inline TargetType stringToTargetType(const std::string& str) {
    if (str == "DRONE") return TargetType::DRONE;
    if (str == "BIRD") return TargetType::BIRD;
    if (str == "VEHICLE") return TargetType::VEHICLE;
    if (str == "AIRCRAFT") return TargetType::AIRCRAFT;
    if (str == "MISSILE") return TargetType::MISSILE;
    if (str == "CLUTTER") return TargetType::CLUTTER;
    return TargetType::UNKNOWN;
}

inline std::string motionModelToString(MotionModel model) {
    switch (model) {
        case MotionModel::CV: return "CV";
        case MotionModel::CA: return "CA";
        case MotionModel::CT: return "CT";
        case MotionModel::IMM: return "IMM";
        default: return "CV";
    }
}

inline MotionModel stringToMotionModel(const std::string& str) {
    if (str == "CA") return MotionModel::CA;
    if (str == "CT") return MotionModel::CT;
    if (str == "IMM") return MotionModel::IMM;
    return MotionModel::CV;
}

inline std::string profileTypeToString(ProfileType profile) {
    switch (profile) {
        case ProfileType::SHORT_RANGE: return "SHORT_RANGE";
        case ProfileType::MEDIUM_RANGE: return "MEDIUM_RANGE";
        case ProfileType::LONG_RANGE: return "LONG_RANGE";
        case ProfileType::CLUTTER_HEAVY: return "CLUTTER_HEAVY";
        default: return "SHORT_RANGE";
    }
}

inline ProfileType stringToProfileType(const std::string& str) {
    if (str == "MEDIUM_RANGE") return ProfileType::MEDIUM_RANGE;
    if (str == "LONG_RANGE") return ProfileType::LONG_RANGE;
    if (str == "CLUTTER_HEAVY") return ProfileType::CLUTTER_HEAVY;
    return ProfileType::SHORT_RANGE;
}

// Utility function to check if value is valid
inline bool isValid(double value) {
    return !std::isnan(value) && !std::isinf(value);
}

} // namespace zoppler

#endif // ZOPPLER_CORE_TYPES_HPP
