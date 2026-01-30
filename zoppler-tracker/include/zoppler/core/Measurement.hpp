/**
 * @file Measurement.hpp
 * @brief Measurement structure for filter updates
 * @copyright Defense-grade radar tracking system
 */

#ifndef ZOPPLER_CORE_MEASUREMENT_HPP
#define ZOPPLER_CORE_MEASUREMENT_HPP

#include "Types.hpp"
#include "Detection.hpp"
#include <vector>

namespace zoppler {

/**
 * @brief Represents a measurement for Kalman filter updates
 * 
 * This is a processed detection ready for filter consumption.
 * Includes the measurement vector and covariance information.
 */
struct Measurement {
    // Measurement vector (typically [x, y, z] or [r, az, el])
    std::vector<double> z;
    
    // Measurement noise covariance (diagonal elements)
    std::vector<double> R;
    
    // Full measurement covariance matrix (row-major, if needed)
    std::vector<double> R_full;
    
    // Timestamp
    Timestamp timestamp = 0;
    
    // Measurement dimension
    int dimension = 0;
    
    // Whether this is a polar or Cartesian measurement
    bool isPolar = false;
    
    // Doppler velocity (if available)
    double doppler = INVALID_VALUE;
    
    // Source detection ID for traceability
    int sourceDetectionId = -1;
    
    /**
     * @brief Default constructor
     */
    Measurement() = default;
    
    /**
     * @brief Construct Cartesian measurement
     */
    static Measurement fromCartesian(double x, double y, double z_val,
                                      double sigmaX, double sigmaY, double sigmaZ,
                                      Timestamp ts) {
        Measurement m;
        m.isPolar = false;
        m.timestamp = ts;
        
        if (zoppler::isValid(z_val)) {
            m.z = {x, y, z_val};
            m.R = {sigmaX * sigmaX, sigmaY * sigmaY, sigmaZ * sigmaZ};
            m.dimension = 3;
        } else {
            m.z = {x, y};
            m.R = {sigmaX * sigmaX, sigmaY * sigmaY};
            m.dimension = 2;
        }
        
        return m;
    }
    
    /**
     * @brief Construct polar measurement
     */
    static Measurement fromPolar(double range, double azimuth, double elevation,
                                  double sigmaR, double sigmaAz, double sigmaEl,
                                  Timestamp ts) {
        Measurement m;
        m.isPolar = true;
        m.timestamp = ts;
        
        if (zoppler::isValid(elevation)) {
            m.z = {range, azimuth, elevation};
            m.R = {sigmaR * sigmaR, sigmaAz * sigmaAz, sigmaEl * sigmaEl};
            m.dimension = 3;
        } else {
            m.z = {range, azimuth};
            m.R = {sigmaR * sigmaR, sigmaAz * sigmaAz};
            m.dimension = 2;
        }
        
        return m;
    }
    
    /**
     * @brief Create measurement from Detection
     */
    static Measurement fromDetection(const Detection& det, 
                                      double sigmaX, double sigmaY, double sigmaZ,
                                      bool usePolar = false) {
        if (usePolar && det.hasValidPolar()) {
            return fromPolar(det.range, det.azimuth, det.elevation,
                           sigmaX, sigmaY, sigmaZ, det.timestamp);
        } else if (det.hasValidCartesian()) {
            return fromCartesian(det.x, det.y, det.z,
                               sigmaX, sigmaY, sigmaZ, det.timestamp);
        }
        return Measurement();
    }
    
    /**
     * @brief Set Doppler measurement
     */
    void setDoppler(double dop, double sigmaDop) {
        doppler = dop;
        // Extend measurement vector if needed
    }
    
    /**
     * @brief Check if measurement is valid
     */
    bool isValid() const {
        return dimension > 0 && z.size() == static_cast<size_t>(dimension);
    }
    
    /**
     * @brief Get measurement value at index
     */
    double operator[](size_t idx) const {
        return (idx < z.size()) ? z[idx] : INVALID_VALUE;
    }
};

} // namespace zoppler

#endif // ZOPPLER_CORE_MEASUREMENT_HPP
