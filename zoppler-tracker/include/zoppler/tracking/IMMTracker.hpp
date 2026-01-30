/**
 * @file IMMTracker.hpp
 * @brief Interacting Multiple Model tracker
 * @copyright Defense-grade radar tracking system
 */

#ifndef ZOPPLER_TRACKING_IMM_HPP
#define ZOPPLER_TRACKING_IMM_HPP

#include "ITrackerModel.hpp"
#include <vector>

namespace zoppler {

/**
 * @brief Interacting Multiple Model (IMM) Tracker
 * 
 * Maintains multiple motion model filters in parallel and
 * combines their estimates based on model probabilities.
 * Adapts to changing target dynamics automatically.
 */
class IMMTracker : public ITrackerModel {
public:
    /**
     * @brief Construct IMM with specified models
     * 
     * @param models Motion models to use
     * @param is3D Whether to track in 3D
     */
    IMMTracker(const std::vector<MotionModel>& models = {MotionModel::CV, MotionModel::CA},
               bool is3D = true);
    
    /**
     * @brief Construct from config
     */
    explicit IMMTracker(const TrackerConfig& config);
    
    /**
     * @brief Destructor
     */
    ~IMMTracker() override = default;
    
    /**
     * @brief Predict state forward
     */
    void predict(double dt) override;
    
    /**
     * @brief Update with measurement
     */
    void update(const Measurement& meas) override;
    
    /**
     * @brief Get combined state estimate
     */
    TrackState getState() const override;
    
    /**
     * @brief Get algorithm name
     */
    std::string getName() const override { return "IMM"; }
    
    /**
     * @brief Clone the tracker
     */
    std::unique_ptr<ITrackerModel> clone() const override;
    
    /**
     * @brief Initialize all models from measurement
     */
    void initialize(const Measurement& meas,
                   double initialVelStd = 50.0,
                   double initialAccStd = 10.0) override;
    
    // Accessors - return combined state
    Matrix getStateVector() const override;
    Matrix getCovariance() const override;
    void setStateVector(const Matrix& state) override;
    void setCovariance(const Matrix& cov) override;
    MotionModel getMotionModel() const override { return MotionModel::IMM; }
    
    /**
     * @brief Get model probabilities
     */
    std::vector<double> getModelProbabilities() const { return mu_; }
    
    /**
     * @brief Set Markov transition matrix
     * 
     * @param transitionMatrix Row-major [nModels x nModels]
     */
    void setTransitionMatrix(const std::vector<double>& transitionMatrix);
    
    /**
     * @brief Set initial model probabilities
     */
    void setInitialProbabilities(const std::vector<double>& probs);
    
    /**
     * @brief Get number of models
     */
    size_t getNumModels() const { return filters_.size(); }
    
    /**
     * @brief Get individual model state
     */
    TrackState getModelState(size_t modelIdx) const;
    
private:
    /**
     * @brief IMM mixing step
     */
    void mixing();
    
    /**
     * @brief IMM mode probability update
     */
    void updateProbabilities(const Measurement& meas);
    
    /**
     * @brief Combine estimates from all models
     */
    void combineEstimates();
    
    /**
     * @brief Compute model likelihood
     */
    double computeLikelihood(size_t modelIdx, const Measurement& meas) const;
    
    std::vector<std::unique_ptr<ITrackerModel>> filters_;
    std::vector<MotionModel> models_;
    
    // Model probabilities
    std::vector<double> mu_;
    
    // Markov transition matrix (row-major)
    std::vector<std::vector<double>> transitionMatrix_;
    
    // Mixing probabilities
    std::vector<std::vector<double>> mixingProbs_;
    
    // Combined state and covariance
    Matrix x_combined_;
    Matrix P_combined_;
    
    bool is3D_;
    int commonStateDim_;  // Common state dimension for mixing
};

} // namespace zoppler

#endif // ZOPPLER_TRACKING_IMM_HPP
