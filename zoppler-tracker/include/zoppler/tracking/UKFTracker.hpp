/**
 * @file UKFTracker.hpp
 * @brief Unscented Kalman Filter tracker
 * @copyright Defense-grade radar tracking system
 */

#ifndef ZOPPLER_TRACKING_UKF_HPP
#define ZOPPLER_TRACKING_UKF_HPP

#include "ITrackerModel.hpp"

namespace zoppler {

/**
 * @brief Unscented Kalman Filter Tracker
 * 
 * Uses sigma points for nonlinear state estimation.
 * Better handles highly nonlinear systems than EKF.
 */
class UKFTracker : public ITrackerModel {
public:
    /**
     * @brief Construct UKF with motion model
     * 
     * @param motionModel Motion model (CV, CA, CT)
     * @param is3D Whether to track in 3D
     */
    UKFTracker(MotionModel motionModel = MotionModel::CV, bool is3D = true);
    
    /**
     * @brief Construct from config
     */
    explicit UKFTracker(const TrackerConfig& config);
    
    /**
     * @brief Destructor
     */
    ~UKFTracker() override = default;
    
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
    std::string getName() const override { return "UKF"; }
    
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
    
    // Accessors
    Matrix getStateVector() const override { return x_; }
    Matrix getCovariance() const override { return P_; }
    void setStateVector(const Matrix& state) override { x_ = state; }
    void setCovariance(const Matrix& cov) override { P_ = cov; }
    MotionModel getMotionModel() const override { return motionModel_; }
    
    // UKF parameters
    void setAlpha(double alpha) { alpha_ = alpha; computeWeights(); }
    void setBeta(double beta) { beta_ = beta; computeWeights(); }
    void setKappa(double kappa) { kappa_ = kappa; computeWeights(); }
    
private:
    /**
     * @brief Compute sigma points
     */
    std::vector<Matrix> computeSigmaPoints() const;
    
    /**
     * @brief Process model (state transition)
     */
    Matrix processModel(const Matrix& state, double dt) const;
    
    /**
     * @brief Measurement model
     */
    Matrix measurementModel(const Matrix& state) const;
    
    /**
     * @brief Compute UKF weights
     */
    void computeWeights();
    
    /**
     * @brief Compute process noise Q
     */
    Matrix computeQ(double dt) const;
    
    MotionModel motionModel_;
    bool is3D_;
    
    Matrix x_;  // State vector
    Matrix P_;  // State covariance
    
    // UKF parameters
    double alpha_ = 0.001;  // Sigma point spread
    double beta_ = 2.0;     // Prior knowledge (2 for Gaussian)
    double kappa_ = 0.0;    // Secondary scaling parameter
    double lambda_;         // Composite scaling parameter
    
    // Weights
    std::vector<double> Wm_;  // Mean weights
    std::vector<double> Wc_;  // Covariance weights
    
    // Measurement noise defaults
    double sigmaX_ = 5.0;
    double sigmaY_ = 5.0;
    double sigmaZ_ = 10.0;
};

} // namespace zoppler

#endif // ZOPPLER_TRACKING_UKF_HPP
