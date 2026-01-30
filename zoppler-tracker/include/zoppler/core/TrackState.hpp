/**
 * @file TrackState.hpp
 * @brief Track state structure for filter output
 * @copyright Defense-grade radar tracking system
 */

#ifndef ZOPPLER_CORE_TRACKSTATE_HPP
#define ZOPPLER_CORE_TRACKSTATE_HPP

#include "Types.hpp"
#include <vector>
#include <cmath>

namespace zoppler {

/**
 * @brief Represents the estimated state of a tracked target
 * 
 * Contains position, velocity, acceleration estimates along with
 * covariance information for uncertainty quantification.
 */
struct TrackState {
    // Position (Cartesian)
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    
    // Velocity (m/s)
    double vx = 0.0;
    double vy = 0.0;
    double vz = 0.0;
    
    // Acceleration (m/s^2) - optional, depends on motion model
    double ax = 0.0;
    double ay = 0.0;
    double az = 0.0;
    
    // Turn rate (rad/s) - for CT model
    double omega = 0.0;
    
    // State covariance (diagonal elements for position/velocity)
    double sigmaX = 0.0;
    double sigmaY = 0.0;
    double sigmaZ = 0.0;
    double sigmaVx = 0.0;
    double sigmaVy = 0.0;
    double sigmaVz = 0.0;
    
    // Full state covariance matrix (row-major)
    std::vector<double> covariance;
    
    // State dimension
    int dimension = 6;  // Default: [x, y, z, vx, vy, vz]
    
    // Timestamp
    Timestamp timestamp = 0;
    
    // Motion model that produced this state
    MotionModel motionModel = MotionModel::CV;
    
    // Model probabilities for IMM
    std::vector<double> modelProbabilities;
    
    /**
     * @brief Get speed magnitude
     */
    double getSpeed() const {
        return std::sqrt(vx * vx + vy * vy + vz * vz);
    }
    
    /**
     * @brief Get 2D speed (horizontal plane)
     */
    double getSpeed2D() const {
        return std::sqrt(vx * vx + vy * vy);
    }
    
    /**
     * @brief Get heading angle (radians, from X-axis)
     */
    double getHeading() const {
        return std::atan2(vy, vx);
    }
    
    /**
     * @brief Get climb angle (radians)
     */
    double getClimbAngle() const {
        double speed2D = getSpeed2D();
        return (speed2D > 0.001) ? std::atan2(vz, speed2D) : 0.0;
    }
    
    /**
     * @brief Get range from origin
     */
    double getRange() const {
        return std::sqrt(x * x + y * y + z * z);
    }
    
    /**
     * @brief Get azimuth angle (radians)
     */
    double getAzimuth() const {
        return std::atan2(y, x);
    }
    
    /**
     * @brief Get elevation angle (radians)
     */
    double getElevation() const {
        double range = getRange();
        return (range > 0.001) ? std::asin(z / range) : 0.0;
    }
    
    /**
     * @brief Get position vector
     */
    std::vector<double> getPosition() const {
        return {x, y, z};
    }
    
    /**
     * @brief Get velocity vector
     */
    std::vector<double> getVelocity() const {
        return {vx, vy, vz};
    }
    
    /**
     * @brief Get state vector based on motion model
     */
    std::vector<double> getStateVector() const {
        switch (motionModel) {
            case MotionModel::CV:
                return {x, y, z, vx, vy, vz};
            case MotionModel::CA:
                return {x, y, z, vx, vy, vz, ax, ay, az};
            case MotionModel::CT:
                return {x, y, z, vx, vy, vz, omega};
            default:
                return {x, y, z, vx, vy, vz};
        }
    }
    
    /**
     * @brief Set state from vector
     */
    void setFromVector(const std::vector<double>& state, MotionModel model) {
        motionModel = model;
        
        if (state.size() >= 6) {
            x = state[0]; y = state[1]; z = state[2];
            vx = state[3]; vy = state[4]; vz = state[5];
        }
        
        if (model == MotionModel::CA && state.size() >= 9) {
            ax = state[6]; ay = state[7]; az = state[8];
            dimension = 9;
        } else if (model == MotionModel::CT && state.size() >= 7) {
            omega = state[6];
            dimension = 7;
        } else {
            dimension = 6;
        }
    }
    
    /**
     * @brief Compute position covariance trace (uncertainty measure)
     */
    double getPositionUncertainty() const {
        return std::sqrt(sigmaX * sigmaX + sigmaY * sigmaY + sigmaZ * sigmaZ);
    }
    
    /**
     * @brief Extrapolate state forward in time
     */
    TrackState extrapolate(double dt) const {
        TrackState extrap = *this;
        
        switch (motionModel) {
            case MotionModel::CV:
                extrap.x += vx * dt;
                extrap.y += vy * dt;
                extrap.z += vz * dt;
                break;
                
            case MotionModel::CA:
                extrap.x += vx * dt + 0.5 * ax * dt * dt;
                extrap.y += vy * dt + 0.5 * ay * dt * dt;
                extrap.z += vz * dt + 0.5 * az * dt * dt;
                extrap.vx += ax * dt;
                extrap.vy += ay * dt;
                extrap.vz += az * dt;
                break;
                
            case MotionModel::CT: {
                if (std::abs(omega) < 1e-6) {
                    // Near-zero turn rate, use CV
                    extrap.x += vx * dt;
                    extrap.y += vy * dt;
                } else {
                    double sinOmegaT = std::sin(omega * dt);
                    double cosOmegaT = std::cos(omega * dt);
                    double oneMinusCos = 1.0 - cosOmegaT;
                    
                    extrap.x += (vx * sinOmegaT + vy * oneMinusCos) / omega;
                    extrap.y += (-vx * oneMinusCos + vy * sinOmegaT) / omega;
                    
                    double newVx = vx * cosOmegaT + vy * sinOmegaT;
                    double newVy = -vx * sinOmegaT + vy * cosOmegaT;
                    extrap.vx = newVx;
                    extrap.vy = newVy;
                }
                extrap.z += vz * dt;
                break;
            }
                
            default:
                extrap.x += vx * dt;
                extrap.y += vy * dt;
                extrap.z += vz * dt;
                break;
        }
        
        return extrap;
    }
};

} // namespace zoppler

#endif // ZOPPLER_CORE_TRACKSTATE_HPP
