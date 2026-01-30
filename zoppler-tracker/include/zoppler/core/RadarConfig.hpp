/**
 * @file RadarConfig.hpp
 * @brief Radar configuration parameters
 * @copyright Defense-grade radar tracking system
 */

#ifndef ZOPPLER_CORE_RADARCONFIG_HPP
#define ZOPPLER_CORE_RADARCONFIG_HPP

#include "Types.hpp"

namespace zoppler {

/**
 * @brief Configuration for radar sensor parameters
 * 
 * Defines the radar characteristics including dimensionality,
 * measurement capabilities, and accuracy specifications.
 */
struct RadarConfig {
    // Radar dimensionality
    bool is3D = true;                   // True for 3D radar (azimuth + elevation)
    bool hasDoppler = true;             // True if Doppler velocity available
    
    // Timing
    double updateTime = 0.1;            // Radar update period (seconds)
    double maxAge = 10.0;               // Max track age before deletion (seconds)
    
    // Measurement noise standard deviations
    double rangeStd = 1.0;              // Range accuracy (meters)
    double azimuthStd = 0.01;           // Azimuth accuracy (radians)
    double elevationStd = 0.02;         // Elevation accuracy (radians)
    double dopplerStd = 0.5;            // Doppler accuracy (m/s)
    
    // Converted to Cartesian (computed from polar based on typical range)
    double xStd = 1.0;                  // X position accuracy (meters)
    double yStd = 1.0;                  // Y position accuracy (meters)
    double zStd = 2.0;                  // Z position accuracy (meters)
    
    // Coverage parameters
    double maxRange = 50000.0;          // Maximum detection range (meters)
    double minRange = 50.0;             // Minimum detection range (meters)
    double maxAzimuth = PI;             // Maximum azimuth angle (radians)
    double minAzimuth = -PI;            // Minimum azimuth angle (radians)
    double maxElevation = PI / 4;       // Maximum elevation angle (radians)
    double minElevation = -PI / 12;     // Minimum elevation angle (radians)
    
    // Detection parameters
    double pD = 0.9;                    // Probability of detection
    double pFA = 1e-6;                  // Probability of false alarm
    double clutterDensity = 1e-8;       // Clutter density (per m^3)
    
    // Radar type identifier
    std::string radarType = "GENERIC";
    
    /**
     * @brief Default constructor with sensible defaults
     */
    RadarConfig() = default;
    
    /**
     * @brief Create short-range FMCW radar config
     */
    static RadarConfig shortRangeFMCW() {
        RadarConfig cfg;
        cfg.radarType = "SHORT_RANGE_FMCW";
        cfg.is3D = true;
        cfg.hasDoppler = true;
        cfg.updateTime = 0.05;          // 20 Hz
        cfg.rangeStd = 0.5;
        cfg.azimuthStd = 0.005;
        cfg.elevationStd = 0.008;
        cfg.dopplerStd = 0.2;
        cfg.maxRange = 5000.0;
        cfg.minRange = 10.0;
        cfg.pD = 0.95;
        cfg.computeCartesianStd(500.0);  // Typical range
        return cfg;
    }
    
    /**
     * @brief Create medium-range surveillance radar config
     */
    static RadarConfig mediumRangeSurveillance() {
        RadarConfig cfg;
        cfg.radarType = "MEDIUM_RANGE_SURVEILLANCE";
        cfg.is3D = true;
        cfg.hasDoppler = true;
        cfg.updateTime = 1.0;           // 1 Hz (rotating)
        cfg.rangeStd = 15.0;
        cfg.azimuthStd = 0.003;
        cfg.elevationStd = 0.015;
        cfg.dopplerStd = 1.0;
        cfg.maxRange = 100000.0;
        cfg.minRange = 500.0;
        cfg.pD = 0.85;
        cfg.computeCartesianStd(30000.0);
        return cfg;
    }
    
    /**
     * @brief Create long-range radar config
     */
    static RadarConfig longRange() {
        RadarConfig cfg;
        cfg.radarType = "LONG_RANGE";
        cfg.is3D = true;
        cfg.hasDoppler = true;
        cfg.updateTime = 4.0;           // 0.25 Hz (large rotating)
        cfg.rangeStd = 50.0;
        cfg.azimuthStd = 0.002;
        cfg.elevationStd = 0.02;
        cfg.dopplerStd = 2.0;
        cfg.maxRange = 400000.0;
        cfg.minRange = 2000.0;
        cfg.pD = 0.8;
        cfg.computeCartesianStd(100000.0);
        return cfg;
    }
    
    /**
     * @brief Create 2D radar config (no elevation)
     */
    static RadarConfig radar2D() {
        RadarConfig cfg;
        cfg.radarType = "2D_SURVEILLANCE";
        cfg.is3D = false;
        cfg.hasDoppler = false;
        cfg.updateTime = 2.0;
        cfg.rangeStd = 20.0;
        cfg.azimuthStd = 0.005;
        cfg.elevationStd = 0.0;
        cfg.dopplerStd = 0.0;
        cfg.maxRange = 200000.0;
        cfg.pD = 0.9;
        cfg.computeCartesianStd(50000.0);
        return cfg;
    }
    
    /**
     * @brief Compute Cartesian standard deviations from polar at given range
     */
    void computeCartesianStd(double typicalRange) {
        // Cross-range error = range * angle_error
        double crossRangeStd = typicalRange * azimuthStd;
        
        // Combine range and cross-range errors
        xStd = std::sqrt(rangeStd * rangeStd + crossRangeStd * crossRangeStd);
        yStd = xStd;
        
        if (is3D) {
            double verticalStd = typicalRange * elevationStd;
            zStd = std::sqrt(rangeStd * rangeStd + verticalStd * verticalStd);
        } else {
            zStd = 0.0;
        }
    }
    
    /**
     * @brief Get measurement dimension
     */
    int getMeasurementDimension() const {
        int dim = 2;  // range + azimuth minimum
        if (is3D) dim++;
        if (hasDoppler) dim++;
        return dim;
    }
    
    /**
     * @brief Validate configuration
     */
    bool isValid() const {
        return updateTime > 0 &&
               rangeStd > 0 &&
               azimuthStd > 0 &&
               maxRange > minRange &&
               pD > 0 && pD <= 1;
    }
};

} // namespace zoppler

#endif // ZOPPLER_CORE_RADARCONFIG_HPP
