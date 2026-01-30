/**
 * @file Detection.hpp
 * @brief Detection structure for radar measurements
 * @copyright Defense-grade radar tracking system
 */

#ifndef ZOPPLER_CORE_DETECTION_HPP
#define ZOPPLER_CORE_DETECTION_HPP

#include "Types.hpp"
#include <cmath>

namespace zoppler {

/**
 * @brief Represents a single radar detection
 * 
 * Supports both Cartesian and polar coordinate systems.
 * Missing measurements are represented by NaN values.
 */
struct Detection {
    // Cartesian coordinates (meters)
    double x = INVALID_VALUE;
    double y = INVALID_VALUE;
    double z = INVALID_VALUE;
    
    // Polar coordinates
    double range = INVALID_VALUE;       // meters
    double azimuth = INVALID_VALUE;     // radians
    double elevation = INVALID_VALUE;   // radians
    
    // Doppler velocity (m/s)
    double doppler = INVALID_VALUE;
    
    // Timestamp in microseconds since epoch
    Timestamp timestamp = 0;
    
    // Signal characteristics
    double snr = INVALID_VALUE;         // Signal-to-noise ratio (dB)
    double rcs = INVALID_VALUE;         // Radar cross section (dBsm)
    
    // Cluster assignment (-1 if unassigned)
    int clusterId = -1;
    
    // Detection quality score [0, 1]
    double quality = 1.0;
    
    /**
     * @brief Default constructor
     */
    Detection() = default;
    
    /**
     * @brief Construct from Cartesian coordinates
     */
    Detection(double x_, double y_, double z_, Timestamp ts_)
        : x(x_), y(y_), z(z_), timestamp(ts_) {
        computePolarFromCartesian();
    }
    
    /**
     * @brief Construct from polar coordinates
     */
    static Detection fromPolar(double range_, double azimuth_, double elevation_, 
                                Timestamp ts_) {
        Detection det;
        det.range = range_;
        det.azimuth = azimuth_;
        det.elevation = elevation_;
        det.timestamp = ts_;
        det.computeCartesianFromPolar();
        return det;
    }
    
    /**
     * @brief Check if detection has valid Cartesian coordinates
     */
    bool hasValidCartesian() const {
        return isValid(x) && isValid(y);
    }
    
    /**
     * @brief Check if detection has valid polar coordinates
     */
    bool hasValidPolar() const {
        return isValid(range) && isValid(azimuth);
    }
    
    /**
     * @brief Check if detection has valid 3D data
     */
    bool is3D() const {
        return isValid(z) || isValid(elevation);
    }
    
    /**
     * @brief Check if detection has Doppler data
     */
    bool hasDoppler() const {
        return isValid(doppler);
    }
    
    /**
     * @brief Compute polar coordinates from Cartesian
     */
    void computePolarFromCartesian() {
        if (!hasValidCartesian()) return;
        
        range = std::sqrt(x * x + y * y + (isValid(z) ? z * z : 0.0));
        azimuth = std::atan2(y, x);
        
        if (isValid(z) && range > 0) {
            elevation = std::asin(z / range);
        }
    }
    
    /**
     * @brief Compute Cartesian coordinates from polar
     */
    void computeCartesianFromPolar() {
        if (!hasValidPolar()) return;
        
        double cosEl = isValid(elevation) ? std::cos(elevation) : 1.0;
        x = range * std::cos(azimuth) * cosEl;
        y = range * std::sin(azimuth) * cosEl;
        
        if (isValid(elevation)) {
            z = range * std::sin(elevation);
        }
    }
    
    /**
     * @brief Compute Euclidean distance to another detection
     */
    double distanceTo(const Detection& other) const {
        if (!hasValidCartesian() || !other.hasValidCartesian()) {
            return INVALID_VALUE;
        }
        
        double dx = x - other.x;
        double dy = y - other.y;
        double dz = (isValid(z) && isValid(other.z)) ? (z - other.z) : 0.0;
        
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    
    /**
     * @brief Get position as a vector [x, y, z]
     */
    std::vector<double> getPosition() const {
        if (is3D()) {
            return {x, y, isValid(z) ? z : 0.0};
        }
        return {x, y};
    }
};

} // namespace zoppler

#endif // ZOPPLER_CORE_DETECTION_HPP
