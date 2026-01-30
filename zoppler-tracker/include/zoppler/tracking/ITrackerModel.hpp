/**
 * @file ITrackerModel.hpp
 * @brief Tracker model interface for filtering
 * @copyright Defense-grade radar tracking system
 */

#ifndef ZOPPLER_TRACKING_ITRACKERMODEL_HPP
#define ZOPPLER_TRACKING_ITRACKERMODEL_HPP

#include "../core/TrackState.hpp"
#include "../core/Measurement.hpp"
#include "../core/TrackerConfig.hpp"
#include "../utils/Matrix.hpp"
#include <memory>

namespace zoppler {

/**
 * @brief Abstract interface for tracking/filtering models
 * 
 * Defines the contract for all tracker implementations.
 * Each tracker implements predict-update cycle for state estimation.
 */
class ITrackerModel {
public:
    /**
     * @brief Virtual destructor
     */
    virtual ~ITrackerModel() = default;
    
    /**
     * @brief Predict state forward by time dt
     * 
     * @param dt Time step in seconds
     */
    virtual void predict(double dt) = 0;
    
    /**
     * @brief Update state with measurement
     * 
     * @param meas Measurement from detection
     */
    virtual void update(const Measurement& meas) = 0;
    
    /**
     * @brief Get current state estimate
     */
    virtual TrackState getState() const = 0;
    
    /**
     * @brief Get algorithm name
     */
    virtual std::string getName() const = 0;
    
    /**
     * @brief Clone the tracker model
     */
    virtual std::unique_ptr<ITrackerModel> clone() const = 0;
    
    /**
     * @brief Initialize state from first measurement
     */
    virtual void initialize(const Measurement& meas,
                           double initialVelStd = 50.0,
                           double initialAccStd = 10.0);
    
    /**
     * @brief Get state vector
     */
    virtual Matrix getStateVector() const = 0;
    
    /**
     * @brief Get state covariance matrix
     */
    virtual Matrix getCovariance() const = 0;
    
    /**
     * @brief Set state vector
     */
    virtual void setStateVector(const Matrix& state) = 0;
    
    /**
     * @brief Set state covariance
     */
    virtual void setCovariance(const Matrix& covariance) = 0;
    
    /**
     * @brief Get innovation (measurement residual)
     */
    virtual Matrix getInnovation() const { return innovation_; }
    
    /**
     * @brief Get innovation covariance
     */
    virtual Matrix getInnovationCovariance() const { return S_; }
    
    /**
     * @brief Get motion model type
     */
    virtual MotionModel getMotionModel() const = 0;
    
    /**
     * @brief Set process noise parameters
     */
    virtual void setProcessNoise(double qPos, double qVel, double qAcc = 0.0);
    
    /**
     * @brief Get state dimension
     */
    int getStateDimension() const { return stateDim_; }
    
    /**
     * @brief Get measurement dimension
     */
    int getMeasurementDimension() const { return measDim_; }
    
protected:
    ITrackerModel() = default;
    
    int stateDim_ = 6;      // State dimension
    int measDim_ = 3;       // Measurement dimension
    
    // Process noise parameters
    double qPos_ = 1.0;     // Position process noise
    double qVel_ = 10.0;    // Velocity process noise
    double qAcc_ = 50.0;    // Acceleration process noise
    
    // Innovation (for gating)
    Matrix innovation_;
    Matrix S_;              // Innovation covariance
    
    // Timestamp
    Timestamp lastUpdateTime_ = 0;
};

} // namespace zoppler

#endif // ZOPPLER_TRACKING_ITRACKERMODEL_HPP
