/**
 * @file ParticleFilterTracker.hpp
 * @brief Particle Filter (Sequential Monte Carlo) tracker
 * @copyright Defense-grade radar tracking system
 */

#ifndef ZOPPLER_TRACKING_PF_HPP
#define ZOPPLER_TRACKING_PF_HPP

#include "ITrackerModel.hpp"
#include <random>

namespace zoppler {

/**
 * @brief Particle representation
 */
struct Particle {
    Matrix state;
    double weight;
    
    Particle() : weight(1.0) {}
    Particle(const Matrix& s, double w) : state(s), weight(w) {}
};

/**
 * @brief Particle Filter Tracker
 * 
 * Non-parametric Bayesian filter using Sequential Monte Carlo.
 * Can handle multi-modal distributions and highly nonlinear dynamics.
 * 
 * Best for: Clutter-heavy environments, non-Gaussian noise, complex maneuvers
 */
class ParticleFilterTracker : public ITrackerModel {
public:
    /**
     * @brief Construct particle filter
     * 
     * @param numParticles Number of particles
     * @param motionModel Motion model
     * @param is3D Whether to track in 3D
     */
    ParticleFilterTracker(int numParticles = 500,
                          MotionModel motionModel = MotionModel::CV,
                          bool is3D = true);
    
    /**
     * @brief Construct from config
     */
    explicit ParticleFilterTracker(const TrackerConfig& config);
    
    /**
     * @brief Destructor
     */
    ~ParticleFilterTracker() override = default;
    
    /**
     * @brief Predict state forward
     */
    void predict(double dt) override;
    
    /**
     * @brief Update with measurement
     */
    void update(const Measurement& meas) override;
    
    /**
     * @brief Get state estimate (weighted mean)
     */
    TrackState getState() const override;
    
    /**
     * @brief Get algorithm name
     */
    std::string getName() const override { return "PARTICLE_FILTER"; }
    
    /**
     * @brief Clone the tracker
     */
    std::unique_ptr<ITrackerModel> clone() const override;
    
    /**
     * @brief Initialize particles from measurement
     */
    void initialize(const Measurement& meas,
                   double initialVelStd = 50.0,
                   double initialAccStd = 10.0) override;
    
    // Accessors
    Matrix getStateVector() const override;
    Matrix getCovariance() const override;
    void setStateVector(const Matrix& state) override;
    void setCovariance(const Matrix& cov) override;
    MotionModel getMotionModel() const override { return motionModel_; }
    
    /**
     * @brief Set number of particles
     */
    void setNumParticles(int n) { numParticles_ = n; }
    
    /**
     * @brief Set resampling threshold (effective sample size ratio)
     */
    void setResampleThreshold(double thresh) { resampleThreshold_ = thresh; }
    
    /**
     * @brief Get effective sample size
     */
    double getEffectiveSampleSize() const;
    
    /**
     * @brief Get particles (for visualization/debugging)
     */
    const std::vector<Particle>& getParticles() const { return particles_; }
    
private:
    /**
     * @brief Propagate particle through dynamics
     */
    Matrix propagateParticle(const Matrix& state, double dt);
    
    /**
     * @brief Compute likelihood for particle given measurement
     */
    double computeLikelihood(const Matrix& state, const Measurement& meas) const;
    
    /**
     * @brief Systematic resampling
     */
    void resample();
    
    /**
     * @brief Normalize particle weights
     */
    void normalizeWeights();
    
    /**
     * @brief Compute weighted mean state
     */
    Matrix computeMean() const;
    
    /**
     * @brief Compute weighted covariance
     */
    Matrix computeCovariance(const Matrix& mean) const;
    
    int numParticles_;
    MotionModel motionModel_;
    bool is3D_;
    
    std::vector<Particle> particles_;
    
    // Resampling threshold (ratio of N_eff / N)
    double resampleThreshold_ = 0.5;
    
    // Random number generator
    mutable std::mt19937 rng_;
    
    // Process and measurement noise
    double processNoise_ = 10.0;
    double measurementNoise_ = 5.0;
};

} // namespace zoppler

#endif // ZOPPLER_TRACKING_PF_HPP
