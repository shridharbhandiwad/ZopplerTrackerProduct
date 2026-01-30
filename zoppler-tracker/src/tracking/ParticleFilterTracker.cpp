/**
 * @file ParticleFilterTracker.cpp
 * @brief Particle Filter implementation
 * @copyright Defense-grade radar tracking system
 */

#include "zoppler/tracking/ParticleFilterTracker.hpp"
#include "zoppler/utils/MathUtils.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>

namespace zoppler {

ParticleFilterTracker::ParticleFilterTracker(int numParticles,
                                             MotionModel motionModel,
                                             bool is3D)
    : numParticles_(numParticles)
    , motionModel_(motionModel)
    , is3D_(is3D)
    , rng_(std::random_device{}()) {
    
    // Set state dimension
    switch (motionModel) {
        case MotionModel::CV:
            stateDim_ = is3D ? 6 : 4;
            break;
        case MotionModel::CA:
            stateDim_ = is3D ? 9 : 6;
            break;
        case MotionModel::CT:
            stateDim_ = is3D ? 7 : 5;
            break;
        default:
            stateDim_ = is3D ? 6 : 4;
            break;
    }
    
    measDim_ = is3D ? 3 : 2;
    
    // Initialize particles with zero state
    particles_.resize(numParticles);
    for (auto& p : particles_) {
        p.state = Matrix(stateDim_, 1, 0.0);
        p.weight = 1.0 / numParticles;
    }
}

ParticleFilterTracker::ParticleFilterTracker(const TrackerConfig& config)
    : ParticleFilterTracker(config.trackingParams.pfNumParticles,
                            config.getMotionModel(),
                            config.radar.is3D) {
    
    resampleThreshold_ = config.trackingParams.pfResampleThreshold;
    processNoise_ = config.trackerProfile.qVel;
    
    // Measurement noise from radar config
    measurementNoise_ = std::sqrt(config.radar.xStd * config.radar.xStd +
                                   config.radar.yStd * config.radar.yStd);
}

std::unique_ptr<ITrackerModel> ParticleFilterTracker::clone() const {
    auto copy = std::make_unique<ParticleFilterTracker>(numParticles_, motionModel_, is3D_);
    copy->particles_ = particles_;
    copy->resampleThreshold_ = resampleThreshold_;
    copy->processNoise_ = processNoise_;
    copy->measurementNoise_ = measurementNoise_;
    return copy;
}

void ParticleFilterTracker::initialize(const Measurement& meas,
                                       double initialVelStd,
                                       double initialAccStd) {
    std::normal_distribution<double> normalDist(0.0, 1.0);
    
    // Measurement position uncertainty
    double posSigma = (meas.R.size() >= 2) 
                      ? std::sqrt((meas.R[0] + meas.R[1]) / 2.0) 
                      : measurementNoise_;
    
    // Initialize particles around measurement
    for (auto& p : particles_) {
        p.state = Matrix(stateDim_, 1, 0.0);
        
        // Position from measurement with noise
        if (meas.z.size() >= 2) {
            p.state(0, 0) = meas.z[0] + normalDist(rng_) * posSigma;
            p.state(1, 0) = meas.z[1] + normalDist(rng_) * posSigma;
            if (is3D_ && meas.z.size() >= 3) {
                p.state(2, 0) = meas.z[2] + normalDist(rng_) * posSigma;
            }
        }
        
        // Random velocity
        int velOffset = is3D_ ? 3 : 2;
        p.state(velOffset, 0) = normalDist(rng_) * initialVelStd;
        p.state(velOffset + 1, 0) = normalDist(rng_) * initialVelStd;
        if (is3D_) {
            p.state(velOffset + 2, 0) = normalDist(rng_) * initialVelStd * 0.5;
        }
        
        // Acceleration for CA model
        if (motionModel_ == MotionModel::CA) {
            int accOffset = velOffset + (is3D_ ? 3 : 2);
            p.state(accOffset, 0) = normalDist(rng_) * initialAccStd;
            p.state(accOffset + 1, 0) = normalDist(rng_) * initialAccStd;
            if (is3D_) {
                p.state(accOffset + 2, 0) = normalDist(rng_) * initialAccStd * 0.5;
            }
        }
        
        // Turn rate for CT model
        if (motionModel_ == MotionModel::CT) {
            p.state(stateDim_ - 1, 0) = normalDist(rng_) * 0.1;
        }
        
        p.weight = 1.0 / numParticles_;
    }
    
    lastUpdateTime_ = meas.timestamp;
}

Matrix ParticleFilterTracker::propagateParticle(const Matrix& state, double dt) {
    Matrix newState = state;
    
    std::normal_distribution<double> normalDist(0.0, 1.0);
    
    int posOffset = 0;
    int velOffset = is3D_ ? 3 : 2;
    
    // Process noise
    double posNoise = processNoise_ * dt * dt * 0.5;
    double velNoise = processNoise_ * dt;
    
    switch (motionModel_) {
        case MotionModel::CV: {
            for (int i = 0; i < (is3D_ ? 3 : 2); ++i) {
                double noise = normalDist(rng_) * posNoise;
                newState(posOffset + i, 0) = state(posOffset + i, 0) 
                                           + state(velOffset + i, 0) * dt + noise;
                newState(velOffset + i, 0) = state(velOffset + i, 0) 
                                           + normalDist(rng_) * velNoise;
            }
            break;
        }
        
        case MotionModel::CA: {
            int accOffset = velOffset + (is3D_ ? 3 : 2);
            double dt2 = 0.5 * dt * dt;
            
            for (int i = 0; i < (is3D_ ? 3 : 2); ++i) {
                newState(posOffset + i, 0) = state(posOffset + i, 0) 
                                           + state(velOffset + i, 0) * dt
                                           + state(accOffset + i, 0) * dt2
                                           + normalDist(rng_) * posNoise;
                newState(velOffset + i, 0) = state(velOffset + i, 0) 
                                           + state(accOffset + i, 0) * dt
                                           + normalDist(rng_) * velNoise;
                newState(accOffset + i, 0) = state(accOffset + i, 0) 
                                           + normalDist(rng_) * processNoise_ * 0.1 * dt;
            }
            break;
        }
        
        case MotionModel::CT: {
            double omega = state(stateDim_ - 1, 0);
            double vx = state(velOffset, 0);
            double vy = state(velOffset + 1, 0);
            
            if (std::abs(omega) < 1e-6) {
                newState(0, 0) = state(0, 0) + vx * dt + normalDist(rng_) * posNoise;
                newState(1, 0) = state(1, 0) + vy * dt + normalDist(rng_) * posNoise;
                newState(velOffset, 0) = vx + normalDist(rng_) * velNoise;
                newState(velOffset + 1, 0) = vy + normalDist(rng_) * velNoise;
            } else {
                double sinOmegaT = std::sin(omega * dt);
                double cosOmegaT = std::cos(omega * dt);
                
                newState(0, 0) = state(0, 0) + (vx * sinOmegaT + vy * (1.0 - cosOmegaT)) / omega
                               + normalDist(rng_) * posNoise;
                newState(1, 0) = state(1, 0) + (-vx * (1.0 - cosOmegaT) + vy * sinOmegaT) / omega
                               + normalDist(rng_) * posNoise;
                newState(velOffset, 0) = vx * cosOmegaT + vy * sinOmegaT 
                                       + normalDist(rng_) * velNoise;
                newState(velOffset + 1, 0) = -vx * sinOmegaT + vy * cosOmegaT 
                                           + normalDist(rng_) * velNoise;
            }
            
            // Z component
            if (is3D_) {
                newState(2, 0) = state(2, 0) + state(velOffset + 2, 0) * dt 
                               + normalDist(rng_) * posNoise;
                newState(velOffset + 2, 0) = state(velOffset + 2, 0) 
                                           + normalDist(rng_) * velNoise;
            }
            
            // Turn rate
            newState(stateDim_ - 1, 0) = omega + normalDist(rng_) * 0.01;
            break;
        }
        
        default:
            for (int i = 0; i < (is3D_ ? 3 : 2); ++i) {
                newState(posOffset + i, 0) = state(posOffset + i, 0) 
                                           + state(velOffset + i, 0) * dt
                                           + normalDist(rng_) * posNoise;
                newState(velOffset + i, 0) = state(velOffset + i, 0) 
                                           + normalDist(rng_) * velNoise;
            }
            break;
    }
    
    return newState;
}

double ParticleFilterTracker::computeLikelihood(const Matrix& state, 
                                                 const Measurement& meas) const {
    // Compute distance between particle position and measurement
    double dx = state(0, 0) - meas.z[0];
    double dy = state(1, 0) - meas.z[1];
    double dz = (is3D_ && meas.z.size() >= 3) 
                ? (state(2, 0) - meas.z[2]) : 0.0;
    
    // Measurement noise
    double sigmaX = (meas.R.size() >= 1) ? std::sqrt(meas.R[0]) : measurementNoise_;
    double sigmaY = (meas.R.size() >= 2) ? std::sqrt(meas.R[1]) : measurementNoise_;
    double sigmaZ = (meas.R.size() >= 3) ? std::sqrt(meas.R[2]) : measurementNoise_ * 2;
    
    // Gaussian likelihood
    double mahalDist = (dx * dx) / (sigmaX * sigmaX)
                     + (dy * dy) / (sigmaY * sigmaY)
                     + (dz * dz) / (sigmaZ * sigmaZ);
    
    return std::exp(-0.5 * mahalDist);
}

void ParticleFilterTracker::normalizeWeights() {
    double totalWeight = 0.0;
    for (const auto& p : particles_) {
        totalWeight += p.weight;
    }
    
    if (totalWeight > 1e-20) {
        for (auto& p : particles_) {
            p.weight /= totalWeight;
        }
    } else {
        // All weights near zero - reset to uniform
        double uniformWeight = 1.0 / numParticles_;
        for (auto& p : particles_) {
            p.weight = uniformWeight;
        }
    }
}

double ParticleFilterTracker::getEffectiveSampleSize() const {
    double sumSq = 0.0;
    for (const auto& p : particles_) {
        sumSq += p.weight * p.weight;
    }
    
    return (sumSq > 1e-20) ? 1.0 / sumSq : numParticles_;
}

void ParticleFilterTracker::resample() {
    std::vector<Particle> newParticles(numParticles_);
    
    // Compute cumulative sum
    std::vector<double> cumSum(numParticles_);
    cumSum[0] = particles_[0].weight;
    for (int i = 1; i < numParticles_; ++i) {
        cumSum[i] = cumSum[i - 1] + particles_[i].weight;
    }
    
    // Systematic resampling
    std::uniform_real_distribution<double> uniformDist(0.0, 1.0 / numParticles_);
    double u = uniformDist(rng_);
    
    int idx = 0;
    for (int i = 0; i < numParticles_; ++i) {
        double target = u + static_cast<double>(i) / numParticles_;
        
        while (idx < numParticles_ - 1 && cumSum[idx] < target) {
            ++idx;
        }
        
        newParticles[i].state = particles_[idx].state;
        newParticles[i].weight = 1.0 / numParticles_;
    }
    
    particles_ = std::move(newParticles);
}

Matrix ParticleFilterTracker::computeMean() const {
    Matrix mean(stateDim_, 1, 0.0);
    
    for (const auto& p : particles_) {
        mean = mean + p.state * p.weight;
    }
    
    return mean;
}

Matrix ParticleFilterTracker::computeCovariance(const Matrix& mean) const {
    Matrix cov(stateDim_, stateDim_, 0.0);
    
    for (const auto& p : particles_) {
        Matrix diff = p.state - mean;
        cov = cov + outerProduct(diff, diff) * p.weight;
    }
    
    return cov;
}

void ParticleFilterTracker::predict(double dt) {
    if (dt <= 0) return;
    
    // Propagate each particle
    for (auto& p : particles_) {
        p.state = propagateParticle(p.state, dt);
    }
}

void ParticleFilterTracker::update(const Measurement& meas) {
    if (!meas.isValid()) return;
    
    // Update weights based on likelihood
    for (auto& p : particles_) {
        double likelihood = computeLikelihood(p.state, meas);
        p.weight *= likelihood;
    }
    
    // Normalize weights
    normalizeWeights();
    
    // Compute innovation for interface compatibility
    Matrix mean = computeMean();
    innovation_ = Matrix(measDim_, 1);
    innovation_(0, 0) = meas.z[0] - mean(0, 0);
    innovation_(1, 0) = meas.z[1] - mean(1, 0);
    if (measDim_ > 2 && meas.z.size() >= 3) {
        innovation_(2, 0) = meas.z[2] - mean(2, 0);
    }
    
    // Resample if effective sample size is too low
    double nEff = getEffectiveSampleSize();
    if (nEff < resampleThreshold_ * numParticles_) {
        resample();
    }
    
    lastUpdateTime_ = meas.timestamp;
}

TrackState ParticleFilterTracker::getState() const {
    Matrix mean = computeMean();
    Matrix cov = computeCovariance(mean);
    
    TrackState state;
    
    state.x = mean(0, 0);
    state.y = mean(1, 0);
    state.z = is3D_ ? mean(2, 0) : 0.0;
    
    int velOffset = is3D_ ? 3 : 2;
    state.vx = mean(velOffset, 0);
    state.vy = mean(velOffset + 1, 0);
    state.vz = is3D_ ? mean(velOffset + 2, 0) : 0.0;
    
    if (motionModel_ == MotionModel::CA) {
        int accOffset = velOffset + (is3D_ ? 3 : 2);
        state.ax = mean(accOffset, 0);
        state.ay = mean(accOffset + 1, 0);
        state.az = is3D_ ? mean(accOffset + 2, 0) : 0.0;
    }
    
    if (motionModel_ == MotionModel::CT) {
        state.omega = mean(stateDim_ - 1, 0);
    }
    
    state.sigmaX = std::sqrt(cov(0, 0));
    state.sigmaY = std::sqrt(cov(1, 1));
    state.sigmaZ = is3D_ ? std::sqrt(cov(2, 2)) : 0.0;
    state.sigmaVx = std::sqrt(cov(velOffset, velOffset));
    state.sigmaVy = std::sqrt(cov(velOffset + 1, velOffset + 1));
    state.sigmaVz = is3D_ ? std::sqrt(cov(velOffset + 2, velOffset + 2)) : 0.0;
    
    state.covariance = cov.data();
    state.dimension = stateDim_;
    state.motionModel = motionModel_;
    state.timestamp = lastUpdateTime_;
    
    return state;
}

Matrix ParticleFilterTracker::getStateVector() const {
    return computeMean();
}

Matrix ParticleFilterTracker::getCovariance() const {
    return computeCovariance(computeMean());
}

void ParticleFilterTracker::setStateVector(const Matrix& state) {
    // Set all particles to this state (with small perturbation)
    std::normal_distribution<double> normalDist(0.0, 0.1);
    
    for (auto& p : particles_) {
        p.state = state;
        for (size_t i = 0; i < state.rows(); ++i) {
            p.state(i, 0) += normalDist(rng_);
        }
        p.weight = 1.0 / numParticles_;
    }
}

void ParticleFilterTracker::setCovariance(const Matrix& cov) {
    // Resample particles from current mean with new covariance
    Matrix mean = computeMean();
    
    try {
        auto samples = MathUtils::sampleMvnormal(mean, cov, numParticles_, rng_);
        
        for (int i = 0; i < numParticles_; ++i) {
            particles_[i].state = samples[i];
            particles_[i].weight = 1.0 / numParticles_;
        }
    } catch (const std::runtime_error&) {
        // If Cholesky fails, just keep current particles
    }
}

} // namespace zoppler
