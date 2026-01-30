/**
 * @file IMMTracker.cpp
 * @brief IMM tracker implementation
 * @copyright Defense-grade radar tracking system
 */

#include "zoppler/tracking/IMMTracker.hpp"
#include "zoppler/tracking/EKFTracker.hpp"
#include "zoppler/utils/MathUtils.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>

namespace zoppler {

IMMTracker::IMMTracker(const std::vector<MotionModel>& models, bool is3D)
    : models_(models)
    , is3D_(is3D) {
    
    if (models_.empty()) {
        models_ = {MotionModel::CV, MotionModel::CA};
    }
    
    size_t n = models_.size();
    
    // Create filters for each model
    for (const auto& model : models_) {
        filters_.push_back(std::make_unique<EKFTracker>(model, is3D));
    }
    
    // Initialize equal probabilities
    mu_.resize(n, 1.0 / n);
    
    // Initialize transition matrix (default: slight preference to stay)
    transitionMatrix_.resize(n, std::vector<double>(n, 0.02));
    for (size_t i = 0; i < n; ++i) {
        transitionMatrix_[i][i] = 1.0 - 0.02 * (n - 1);
    }
    
    // Initialize mixing probabilities
    mixingProbs_.resize(n, std::vector<double>(n, 0.0));
    
    // Common state dimension (position + velocity)
    commonStateDim_ = is3D ? 6 : 4;
    stateDim_ = commonStateDim_;
    measDim_ = is3D ? 3 : 2;
    
    x_combined_ = Matrix(stateDim_, 1, 0.0);
    P_combined_ = Matrix::identity(stateDim_) * 100.0;
}

IMMTracker::IMMTracker(const TrackerConfig& config)
    : is3D_(config.radar.is3D) {
    
    // Parse model names from config
    for (const auto& modelName : config.trackingParams.immModels) {
        models_.push_back(stringToMotionModel(modelName));
    }
    
    if (models_.empty()) {
        models_ = {MotionModel::CV, MotionModel::CA};
    }
    
    size_t n = models_.size();
    
    // Create filters
    TrackerConfig filterConfig = config;
    for (const auto& model : models_) {
        filterConfig.motionModel = motionModelToString(model);
        filters_.push_back(std::make_unique<EKFTracker>(filterConfig));
    }
    
    // Set initial probabilities
    if (config.trackingParams.immInitialProbs.size() == n) {
        mu_ = config.trackingParams.immInitialProbs;
    } else {
        mu_.resize(n, 1.0 / n);
    }
    
    // Set transition matrix
    if (config.trackingParams.immTransitionMatrix.size() == n * n) {
        transitionMatrix_.resize(n);
        for (size_t i = 0; i < n; ++i) {
            transitionMatrix_[i].resize(n);
            for (size_t j = 0; j < n; ++j) {
                transitionMatrix_[i][j] = config.trackingParams.immTransitionMatrix[i * n + j];
            }
        }
    } else {
        transitionMatrix_.resize(n, std::vector<double>(n, 0.02));
        for (size_t i = 0; i < n; ++i) {
            transitionMatrix_[i][i] = 1.0 - 0.02 * (n - 1);
        }
    }
    
    mixingProbs_.resize(n, std::vector<double>(n, 0.0));
    
    commonStateDim_ = is3D_ ? 6 : 4;
    stateDim_ = commonStateDim_;
    measDim_ = is3D_ ? 3 : 2;
    
    x_combined_ = Matrix(stateDim_, 1, 0.0);
    P_combined_ = Matrix::identity(stateDim_) * 100.0;
}

std::unique_ptr<ITrackerModel> IMMTracker::clone() const {
    auto copy = std::make_unique<IMMTracker>(models_, is3D_);
    
    for (size_t i = 0; i < filters_.size(); ++i) {
        copy->filters_[i] = filters_[i]->clone();
    }
    
    copy->mu_ = mu_;
    copy->transitionMatrix_ = transitionMatrix_;
    copy->x_combined_ = x_combined_;
    copy->P_combined_ = P_combined_;
    
    return copy;
}

void IMMTracker::setTransitionMatrix(const std::vector<double>& transitionMatrix) {
    size_t n = models_.size();
    if (transitionMatrix.size() != n * n) return;
    
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
            transitionMatrix_[i][j] = transitionMatrix[i * n + j];
        }
    }
}

void IMMTracker::setInitialProbabilities(const std::vector<double>& probs) {
    if (probs.size() == models_.size()) {
        mu_ = probs;
        
        // Normalize
        double sum = std::accumulate(mu_.begin(), mu_.end(), 0.0);
        if (sum > 0) {
            for (auto& p : mu_) p /= sum;
        }
    }
}

void IMMTracker::initialize(const Measurement& meas,
                            double initialVelStd,
                            double initialAccStd) {
    for (auto& filter : filters_) {
        filter->initialize(meas, initialVelStd, initialAccStd);
    }
    
    // Reset probabilities
    size_t n = filters_.size();
    std::fill(mu_.begin(), mu_.end(), 1.0 / n);
    
    // Initialize combined state
    combineEstimates();
    
    lastUpdateTime_ = meas.timestamp;
}

void IMMTracker::mixing() {
    size_t n = filters_.size();
    
    // Compute predicted mode probabilities
    std::vector<double> cBar(n, 0.0);
    for (size_t j = 0; j < n; ++j) {
        for (size_t i = 0; i < n; ++i) {
            cBar[j] += transitionMatrix_[i][j] * mu_[i];
        }
    }
    
    // Compute mixing probabilities
    for (size_t j = 0; j < n; ++j) {
        for (size_t i = 0; i < n; ++i) {
            if (cBar[j] > 1e-10) {
                mixingProbs_[i][j] = transitionMatrix_[i][j] * mu_[i] / cBar[j];
            } else {
                mixingProbs_[i][j] = 1.0 / n;
            }
        }
    }
    
    // Compute mixed initial states for each filter
    std::vector<Matrix> mixedStates(n);
    std::vector<Matrix> mixedCovs(n);
    
    for (size_t j = 0; j < n; ++j) {
        // Get state dimension for this filter
        int dimJ = filters_[j]->getStateDimension();
        mixedStates[j] = Matrix(dimJ, 1, 0.0);
        
        // Compute mixed mean (only common states)
        for (size_t i = 0; i < n; ++i) {
            Matrix xi = filters_[i]->getStateVector();
            
            // Only mix common state elements
            for (int k = 0; k < std::min(commonStateDim_, static_cast<int>(xi.rows())); ++k) {
                mixedStates[j](k, 0) += mixingProbs_[i][j] * xi(k, 0);
            }
        }
        
        // Copy extra states from filter j itself
        Matrix xj = filters_[j]->getStateVector();
        for (int k = commonStateDim_; k < dimJ; ++k) {
            mixedStates[j](k, 0) = xj(k, 0);
        }
        
        // Compute mixed covariance
        mixedCovs[j] = Matrix(dimJ, dimJ, 0.0);
        
        for (size_t i = 0; i < n; ++i) {
            Matrix xi = filters_[i]->getStateVector();
            Matrix Pi = filters_[i]->getCovariance();
            
            // Compute difference in common states
            Matrix diff(commonStateDim_, 1, 0.0);
            for (int k = 0; k < commonStateDim_; ++k) {
                if (k < static_cast<int>(xi.rows())) {
                    diff(k, 0) = xi(k, 0) - mixedStates[j](k, 0);
                }
            }
            
            // Add to covariance (common states only)
            for (int r = 0; r < commonStateDim_; ++r) {
                for (int c = 0; c < commonStateDim_; ++c) {
                    double pVal = (r < static_cast<int>(Pi.rows()) && c < static_cast<int>(Pi.cols())) 
                                  ? Pi(r, c) : 0.0;
                    mixedCovs[j](r, c) += mixingProbs_[i][j] * (pVal + diff(r, 0) * diff(c, 0));
                }
            }
        }
        
        // Copy extra covariance elements from filter j
        Matrix Pj = filters_[j]->getCovariance();
        for (int r = commonStateDim_; r < dimJ; ++r) {
            for (int c = 0; c < dimJ; ++c) {
                mixedCovs[j](r, c) = Pj(r, c);
            }
        }
        for (int r = 0; r < dimJ; ++r) {
            for (int c = commonStateDim_; c < dimJ; ++c) {
                mixedCovs[j](r, c) = Pj(r, c);
            }
        }
    }
    
    // Set mixed states to filters
    for (size_t j = 0; j < n; ++j) {
        filters_[j]->setStateVector(mixedStates[j]);
        filters_[j]->setCovariance(mixedCovs[j]);
    }
}

double IMMTracker::computeLikelihood(size_t modelIdx, const Measurement& meas) const {
    if (modelIdx >= filters_.size()) return 1e-10;
    
    Matrix innovation = filters_[modelIdx]->getInnovation();
    Matrix S = filters_[modelIdx]->getInnovationCovariance();
    
    if (innovation.empty() || S.empty()) {
        return 1.0 / filters_.size();
    }
    
    // Multivariate Gaussian likelihood
    int dim = static_cast<int>(innovation.rows());
    double det = S.determinant();
    if (det <= 0) det = 1e-10;
    
    double mahalDist = mahalanobisDistSq(innovation, S.safeInverse());
    double normConst = std::pow(2.0 * PI, -dim / 2.0) * std::pow(det, -0.5);
    double likelihood = normConst * std::exp(-0.5 * mahalDist);
    
    return std::max(likelihood, 1e-20);
}

void IMMTracker::updateProbabilities(const Measurement& meas) {
    size_t n = filters_.size();
    
    // Compute predicted probabilities
    std::vector<double> cBar(n, 0.0);
    for (size_t j = 0; j < n; ++j) {
        for (size_t i = 0; i < n; ++i) {
            cBar[j] += transitionMatrix_[i][j] * mu_[i];
        }
    }
    
    // Compute likelihoods
    std::vector<double> likelihoods(n);
    for (size_t j = 0; j < n; ++j) {
        likelihoods[j] = computeLikelihood(j, meas);
    }
    
    // Update mode probabilities
    double c = 0.0;
    for (size_t j = 0; j < n; ++j) {
        mu_[j] = likelihoods[j] * cBar[j];
        c += mu_[j];
    }
    
    // Normalize
    if (c > 1e-20) {
        for (auto& p : mu_) {
            p /= c;
        }
    } else {
        std::fill(mu_.begin(), mu_.end(), 1.0 / n);
    }
}

void IMMTracker::combineEstimates() {
    size_t n = filters_.size();
    
    // Compute combined state (common states only)
    x_combined_ = Matrix(commonStateDim_, 1, 0.0);
    
    for (size_t i = 0; i < n; ++i) {
        Matrix xi = filters_[i]->getStateVector();
        for (int k = 0; k < commonStateDim_ && k < static_cast<int>(xi.rows()); ++k) {
            x_combined_(k, 0) += mu_[i] * xi(k, 0);
        }
    }
    
    // Compute combined covariance
    P_combined_ = Matrix(commonStateDim_, commonStateDim_, 0.0);
    
    for (size_t i = 0; i < n; ++i) {
        Matrix xi = filters_[i]->getStateVector();
        Matrix Pi = filters_[i]->getCovariance();
        
        // Difference from combined state
        Matrix diff(commonStateDim_, 1, 0.0);
        for (int k = 0; k < commonStateDim_ && k < static_cast<int>(xi.rows()); ++k) {
            diff(k, 0) = xi(k, 0) - x_combined_(k, 0);
        }
        
        // Add to covariance
        for (int r = 0; r < commonStateDim_; ++r) {
            for (int c = 0; c < commonStateDim_; ++c) {
                double pVal = (r < static_cast<int>(Pi.rows()) && c < static_cast<int>(Pi.cols())) 
                              ? Pi(r, c) : 0.0;
                P_combined_(r, c) += mu_[i] * (pVal + diff(r, 0) * diff(c, 0));
            }
        }
    }
}

void IMMTracker::predict(double dt) {
    if (dt <= 0) return;
    
    // Mixing step
    mixing();
    
    // Predict each filter
    for (auto& filter : filters_) {
        filter->predict(dt);
    }
    
    // Combine estimates
    combineEstimates();
}

void IMMTracker::update(const Measurement& meas) {
    if (!meas.isValid()) return;
    
    // Update each filter
    for (auto& filter : filters_) {
        filter->update(meas);
    }
    
    // Update mode probabilities
    updateProbabilities(meas);
    
    // Combine estimates
    combineEstimates();
    
    lastUpdateTime_ = meas.timestamp;
}

TrackState IMMTracker::getState() const {
    TrackState state;
    
    state.x = x_combined_(0, 0);
    state.y = x_combined_(1, 0);
    state.z = is3D_ ? x_combined_(2, 0) : 0.0;
    
    int velOffset = is3D_ ? 3 : 2;
    state.vx = x_combined_(velOffset, 0);
    state.vy = x_combined_(velOffset + 1, 0);
    state.vz = is3D_ ? x_combined_(velOffset + 2, 0) : 0.0;
    
    state.sigmaX = std::sqrt(P_combined_(0, 0));
    state.sigmaY = std::sqrt(P_combined_(1, 1));
    state.sigmaZ = is3D_ ? std::sqrt(P_combined_(2, 2)) : 0.0;
    state.sigmaVx = std::sqrt(P_combined_(velOffset, velOffset));
    state.sigmaVy = std::sqrt(P_combined_(velOffset + 1, velOffset + 1));
    state.sigmaVz = is3D_ ? std::sqrt(P_combined_(velOffset + 2, velOffset + 2)) : 0.0;
    
    state.covariance = P_combined_.data();
    state.dimension = commonStateDim_;
    state.motionModel = MotionModel::IMM;
    state.modelProbabilities = mu_;
    state.timestamp = lastUpdateTime_;
    
    return state;
}

TrackState IMMTracker::getModelState(size_t modelIdx) const {
    if (modelIdx < filters_.size()) {
        return filters_[modelIdx]->getState();
    }
    return TrackState();
}

Matrix IMMTracker::getStateVector() const {
    return x_combined_;
}

Matrix IMMTracker::getCovariance() const {
    return P_combined_;
}

void IMMTracker::setStateVector(const Matrix& state) {
    x_combined_ = state;
    
    // Also set to all filters (common states)
    for (auto& filter : filters_) {
        Matrix xi = filter->getStateVector();
        for (int k = 0; k < commonStateDim_ && k < static_cast<int>(xi.rows()); ++k) {
            xi(k, 0) = state(k, 0);
        }
        filter->setStateVector(xi);
    }
}

void IMMTracker::setCovariance(const Matrix& cov) {
    P_combined_ = cov;
    
    // Also set to all filters (common states)
    for (auto& filter : filters_) {
        Matrix Pi = filter->getCovariance();
        for (int r = 0; r < commonStateDim_ && r < static_cast<int>(Pi.rows()); ++r) {
            for (int c = 0; c < commonStateDim_ && c < static_cast<int>(Pi.cols()); ++c) {
                Pi(r, c) = cov(r, c);
            }
        }
        filter->setCovariance(Pi);
    }
}

} // namespace zoppler
