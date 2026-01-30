/**
 * @file EKFTracker.hpp
 * @brief Extended Kalman Filter tracker
 * @copyright Defense-grade radar tracking system
 */

#ifndef ZOPPLER_TRACKING_EKF_HPP
#define ZOPPLER_TRACKING_EKF_HPP

#include "ITrackerModel.hpp"

namespace zoppler {

/**
 * @brief Extended Kalman Filter Tracker
 * 
 * Implements EKF for nonlinear state estimation.
 * Supports CV, CA, and CT motion models.
 */
class EKFTracker : public ITrackerModel {
public:
    /**
     * @brief Construct EKF with motion model
     * 
     * @param motionModel Motion model (CV, CA, CT)
     * @param is3D Whether to track in 3D
     */
    EKFTracker(MotionModel motionModel = MotionModel::CV, bool is3D = true);
    
    /**
     * @brief Construct from config
     */
    explicit EKFTracker(const TrackerConfig& config);
    
    /**
     * @brief Destructor
     */
    ~EKFTracker() override = default;
    
    /**
     * @brief Predict state forward
     */
    void predict(double dt) override;
    
    /**
     * @brief Update with measurement
     */
    void update(const Measurement& meas) override;
    
    /**
     * @brief Get current state
     */
    TrackState getState() const override;
    
    /**
     * @brief Get algorithm name
     */
    std::string getName() const override;
    
    /**
     * @brief Clone the tracker
     */
    std::unique_ptr<ITrackerModel> clone() const override;
    
    /**
     * @brief Initialize from measurement
     */
    void initialize(const Measurement& meas,
                   double initialVelStd = 50.0,
                   double initialAccStd = 10.0) override;
    
    /**
     * @brief Get state vector
     */
    Matrix getStateVector() const override { return x_; }
    
    /**
     * @brief Get covariance
     */
    Matrix getCovariance() const override { return P_; }
    
    /**
     * @brief Set state vector
     */
    void setStateVector(const Matrix& state) override { x_ = state; }
    
    /**
     * @brief Set covariance
     */
    void setCovariance(const Matrix& cov) override { P_ = cov; }
    
    /**
     * @brief Get motion model
     */
    MotionModel getMotionModel() const override { return motionModel_; }
    
    /**
     * @brief Set turn rate (for CT model)
     */
    void setTurnRate(double omega) { 
        if (motionModel_ == MotionModel::CT && x_.rows() > 6) {
            x_(6, 0) = omega;
        }
    }
    
protected:
    /**
     * @brief Compute state transition matrix F
     */
    Matrix computeF(double dt) const;
    
    /**
     * @brief Compute process noise Q
     */
    Matrix computeQ(double dt) const;
    
    /**
     * @brief Compute measurement matrix H
     */
    Matrix computeH() const;
    
    /**
     * @brief Compute measurement noise R
     */
    Matrix computeR(const Measurement& meas) const;
    
    MotionModel motionModel_;
    bool is3D_;
    
    Matrix x_;  // State vector
    Matrix P_;  // State covariance
    
    // Measurement noise defaults
    double sigmaX_ = 5.0;
    double sigmaY_ = 5.0;
    double sigmaZ_ = 10.0;
};

/**
 * @brief CV (Constant Velocity) Tracker - Convenience wrapper
 */
class CVTracker : public EKFTracker {
public:
    CVTracker(bool is3D = true) : EKFTracker(MotionModel::CV, is3D) {}
    explicit CVTracker(const TrackerConfig& config);
    std::unique_ptr<ITrackerModel> clone() const override;
};

/**
 * @brief CA (Constant Acceleration) Tracker - Convenience wrapper
 */
class CATracker : public EKFTracker {
public:
    CATracker(bool is3D = true) : EKFTracker(MotionModel::CA, is3D) {}
    explicit CATracker(const TrackerConfig& config);
    std::unique_ptr<ITrackerModel> clone() const override;
};

/**
 * @brief CT (Coordinated Turn) Tracker - Convenience wrapper
 */
class CTTracker : public EKFTracker {
public:
    CTTracker(bool is3D = true) : EKFTracker(MotionModel::CT, is3D) {}
    explicit CTTracker(const TrackerConfig& config);
    std::unique_ptr<ITrackerModel> clone() const override;
};

} // namespace zoppler

#endif // ZOPPLER_TRACKING_EKF_HPP
