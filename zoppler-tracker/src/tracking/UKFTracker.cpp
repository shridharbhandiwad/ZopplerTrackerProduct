/**
 * @file UKFTracker.cpp
 * @brief UKF tracker implementation
 * @copyright Defense-grade radar tracking system
 */

#include "zoppler/tracking/UKFTracker.hpp"
#include "zoppler/utils/MathUtils.hpp"
#include <cmath>

namespace zoppler {

UKFTracker::UKFTracker(MotionModel motionModel, bool is3D)
    : motionModel_(motionModel)
    , is3D_(is3D) {
    
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
    
    x_ = Matrix(stateDim_, 1, 0.0);
    P_ = Matrix::identity(stateDim_) * 100.0;
    
    computeWeights();
}

UKFTracker::UKFTracker(const TrackerConfig& config)
    : UKFTracker(config.getMotionModel(), config.radar.is3D) {
    
    qPos_ = config.trackerProfile.qPos;
    qVel_ = config.trackerProfile.qVel;
    qAcc_ = config.trackerProfile.qAcc;
    
    alpha_ = config.trackingParams.ukfAlpha;
    beta_ = config.trackingParams.ukfBeta;
    kappa_ = config.trackingParams.ukfKappa;
    
    sigmaX_ = config.radar.xStd;
    sigmaY_ = config.radar.yStd;
    sigmaZ_ = config.radar.zStd;
    
    computeWeights();
}

std::unique_ptr<ITrackerModel> UKFTracker::clone() const {
    auto copy = std::make_unique<UKFTracker>(motionModel_, is3D_);
    copy->x_ = x_;
    copy->P_ = P_;
    copy->qPos_ = qPos_;
    copy->qVel_ = qVel_;
    copy->qAcc_ = qAcc_;
    copy->alpha_ = alpha_;
    copy->beta_ = beta_;
    copy->kappa_ = kappa_;
    copy->computeWeights();
    return copy;
}

void UKFTracker::computeWeights() {
    int n = stateDim_;
    lambda_ = alpha_ * alpha_ * (n + kappa_) - n;
    
    int numSigmaPoints = 2 * n + 1;
    Wm_.resize(numSigmaPoints);
    Wc_.resize(numSigmaPoints);
    
    // Weight for mean sigma point
    Wm_[0] = lambda_ / (n + lambda_);
    Wc_[0] = Wm_[0] + (1 - alpha_ * alpha_ + beta_);
    
    // Weights for other sigma points
    double weight = 0.5 / (n + lambda_);
    for (int i = 1; i < numSigmaPoints; ++i) {
        Wm_[i] = weight;
        Wc_[i] = weight;
    }
}

std::vector<Matrix> UKFTracker::computeSigmaPoints() const {
    int n = stateDim_;
    int numSigmaPoints = 2 * n + 1;
    
    std::vector<Matrix> sigmaPoints(numSigmaPoints, Matrix(n, 1));
    
    // Compute square root of scaled covariance
    double scale = std::sqrt(n + lambda_);
    Matrix sqrtP;
    
    try {
        sqrtP = P_.cholesky();
    } catch (const std::runtime_error&) {
        // If Cholesky fails, use diagonal approximation
        sqrtP = Matrix(n, n, 0.0);
        for (int i = 0; i < n; ++i) {
            sqrtP(i, i) = std::sqrt(std::max(P_(i, i), 1e-10));
        }
    }
    
    // First sigma point is the mean
    sigmaPoints[0] = x_;
    
    // Generate sigma points
    for (int i = 0; i < n; ++i) {
        Matrix col = sqrtP.col(i) * scale;
        sigmaPoints[1 + i] = x_ + col;
        sigmaPoints[1 + n + i] = x_ - col;
    }
    
    return sigmaPoints;
}

Matrix UKFTracker::processModel(const Matrix& state, double dt) const {
    Matrix x_new = state;
    
    int posOffset = 0;
    int velOffset = is3D_ ? 3 : 2;
    
    switch (motionModel_) {
        case MotionModel::CV: {
            // x' = x + v*dt
            for (int i = 0; i < (is3D_ ? 3 : 2); ++i) {
                x_new(posOffset + i, 0) = state(posOffset + i, 0) 
                                        + state(velOffset + i, 0) * dt;
            }
            break;
        }
        
        case MotionModel::CA: {
            int accOffset = velOffset + (is3D_ ? 3 : 2);
            double dt2 = 0.5 * dt * dt;
            
            for (int i = 0; i < (is3D_ ? 3 : 2); ++i) {
                x_new(posOffset + i, 0) = state(posOffset + i, 0) 
                                        + state(velOffset + i, 0) * dt
                                        + state(accOffset + i, 0) * dt2;
                x_new(velOffset + i, 0) = state(velOffset + i, 0) 
                                        + state(accOffset + i, 0) * dt;
            }
            break;
        }
        
        case MotionModel::CT: {
            double omega = state(stateDim_ - 1, 0);
            double vx = state(velOffset, 0);
            double vy = state(velOffset + 1, 0);
            
            if (std::abs(omega) < 1e-6) {
                // Near-zero turn rate
                x_new(0, 0) = state(0, 0) + vx * dt;
                x_new(1, 0) = state(1, 0) + vy * dt;
            } else {
                double sinOmegaT = std::sin(omega * dt);
                double cosOmegaT = std::cos(omega * dt);
                
                x_new(0, 0) = state(0, 0) + (vx * sinOmegaT + vy * (1.0 - cosOmegaT)) / omega;
                x_new(1, 0) = state(1, 0) + (-vx * (1.0 - cosOmegaT) + vy * sinOmegaT) / omega;
                x_new(velOffset, 0) = vx * cosOmegaT + vy * sinOmegaT;
                x_new(velOffset + 1, 0) = -vx * sinOmegaT + vy * cosOmegaT;
            }
            
            // Z component (if 3D) - simple CV
            if (is3D_) {
                x_new(2, 0) = state(2, 0) + state(velOffset + 2, 0) * dt;
            }
            break;
        }
        
        default:
            for (int i = 0; i < (is3D_ ? 3 : 2); ++i) {
                x_new(posOffset + i, 0) = state(posOffset + i, 0) 
                                        + state(velOffset + i, 0) * dt;
            }
            break;
    }
    
    return x_new;
}

Matrix UKFTracker::measurementModel(const Matrix& state) const {
    Matrix z(measDim_, 1);
    
    // Position measurement
    for (int i = 0; i < measDim_; ++i) {
        z(i, 0) = state(i, 0);
    }
    
    return z;
}

Matrix UKFTracker::computeQ(double dt) const {
    Matrix Q(stateDim_, stateDim_, 0.0);
    
    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    double dt4 = dt3 * dt;
    
    int posOffset = 0;
    int velOffset = is3D_ ? 3 : 2;
    
    // Similar to EKF process noise
    for (int i = 0; i < (is3D_ ? 3 : 2); ++i) {
        Q(posOffset + i, posOffset + i) = qVel_ * dt4 / 4.0;
        Q(posOffset + i, velOffset + i) = qVel_ * dt3 / 2.0;
        Q(velOffset + i, posOffset + i) = qVel_ * dt3 / 2.0;
        Q(velOffset + i, velOffset + i) = qVel_ * dt2;
    }
    
    if (motionModel_ == MotionModel::CA) {
        int accOffset = velOffset + (is3D_ ? 3 : 2);
        for (int i = 0; i < (is3D_ ? 3 : 2); ++i) {
            Q(accOffset + i, accOffset + i) = qAcc_ * dt;
        }
    }
    
    if (motionModel_ == MotionModel::CT) {
        Q(stateDim_ - 1, stateDim_ - 1) = qAcc_ * 0.01 * dt;
    }
    
    return Q;
}

void UKFTracker::initialize(const Measurement& meas,
                            double initialVelStd,
                            double initialAccStd) {
    if (meas.z.size() >= 2) {
        x_(0, 0) = meas.z[0];
        x_(1, 0) = meas.z[1];
        if (is3D_ && meas.z.size() >= 3) {
            x_(2, 0) = meas.z[2];
        }
    }
    
    P_ = Matrix::identity(stateDim_);
    
    if (meas.R.size() >= 2) {
        P_(0, 0) = meas.R[0];
        P_(1, 1) = meas.R[1];
        if (is3D_ && meas.R.size() >= 3) {
            P_(2, 2) = meas.R[2];
        }
    }
    
    int velOffset = is3D_ ? 3 : 2;
    double velVar = initialVelStd * initialVelStd;
    for (int i = 0; i < (is3D_ ? 3 : 2); ++i) {
        P_(velOffset + i, velOffset + i) = velVar;
    }
    
    if (motionModel_ == MotionModel::CA) {
        int accOffset = velOffset + (is3D_ ? 3 : 2);
        double accVar = initialAccStd * initialAccStd;
        for (int i = 0; i < (is3D_ ? 3 : 2); ++i) {
            P_(accOffset + i, accOffset + i) = accVar;
        }
    }
    
    if (motionModel_ == MotionModel::CT) {
        P_(stateDim_ - 1, stateDim_ - 1) = 0.1;
    }
    
    lastUpdateTime_ = meas.timestamp;
}

void UKFTracker::predict(double dt) {
    if (dt <= 0) return;
    
    int numSigmaPoints = 2 * stateDim_ + 1;
    
    // Generate sigma points
    auto sigmaPoints = computeSigmaPoints();
    
    // Propagate sigma points through process model
    std::vector<Matrix> propagatedPoints(numSigmaPoints);
    for (int i = 0; i < numSigmaPoints; ++i) {
        propagatedPoints[i] = processModel(sigmaPoints[i], dt);
    }
    
    // Compute predicted mean
    x_ = MathUtils::weightedMean(propagatedPoints, Wm_);
    
    // Compute predicted covariance
    P_ = MathUtils::weightedCovariance(propagatedPoints, x_, Wc_);
    
    // Add process noise
    P_ = P_ + computeQ(dt);
}

void UKFTracker::update(const Measurement& meas) {
    if (!meas.isValid()) return;
    
    int numSigmaPoints = 2 * stateDim_ + 1;
    
    // Generate sigma points
    auto sigmaPoints = computeSigmaPoints();
    
    // Transform sigma points through measurement model
    std::vector<Matrix> measPoints(numSigmaPoints);
    for (int i = 0; i < numSigmaPoints; ++i) {
        measPoints[i] = measurementModel(sigmaPoints[i]);
    }
    
    // Predicted measurement mean
    Matrix z_pred = MathUtils::weightedMean(measPoints, Wm_);
    
    // Innovation covariance
    S_ = MathUtils::weightedCovariance(measPoints, z_pred, Wc_);
    
    // Add measurement noise
    Matrix R(measDim_, measDim_, 0.0);
    for (int i = 0; i < measDim_ && i < static_cast<int>(meas.R.size()); ++i) {
        R(i, i) = meas.R[i];
    }
    if (meas.R.empty()) {
        R(0, 0) = sigmaX_ * sigmaX_;
        R(1, 1) = sigmaY_ * sigmaY_;
        if (measDim_ > 2) R(2, 2) = sigmaZ_ * sigmaZ_;
    }
    S_ = S_ + R;
    
    // Cross covariance
    Matrix Pxz(stateDim_, measDim_, 0.0);
    for (int i = 0; i < numSigmaPoints; ++i) {
        Matrix xDiff = sigmaPoints[i] - x_;
        Matrix zDiff = measPoints[i] - z_pred;
        Pxz = Pxz + outerProduct(xDiff, zDiff) * Wc_[i];
    }
    
    // Kalman gain
    Matrix K = Pxz * S_.safeInverse();
    
    // Measurement vector
    Matrix z(measDim_, 1);
    for (int i = 0; i < measDim_ && i < static_cast<int>(meas.z.size()); ++i) {
        z(i, 0) = meas.z[i];
    }
    
    // Innovation
    innovation_ = z - z_pred;
    
    // Update state
    x_ = x_ + K * innovation_;
    
    // Update covariance
    P_ = P_ - K * S_ * K.transpose();
    
    lastUpdateTime_ = meas.timestamp;
}

TrackState UKFTracker::getState() const {
    TrackState state;
    
    int posOffset = 0;
    int velOffset = is3D_ ? 3 : 2;
    
    state.x = x_(0, 0);
    state.y = x_(1, 0);
    state.z = is3D_ ? x_(2, 0) : 0.0;
    
    state.vx = x_(velOffset, 0);
    state.vy = x_(velOffset + 1, 0);
    state.vz = is3D_ ? x_(velOffset + 2, 0) : 0.0;
    
    if (motionModel_ == MotionModel::CA) {
        int accOffset = velOffset + (is3D_ ? 3 : 2);
        state.ax = x_(accOffset, 0);
        state.ay = x_(accOffset + 1, 0);
        state.az = is3D_ ? x_(accOffset + 2, 0) : 0.0;
    }
    
    if (motionModel_ == MotionModel::CT) {
        state.omega = x_(stateDim_ - 1, 0);
    }
    
    state.sigmaX = std::sqrt(P_(0, 0));
    state.sigmaY = std::sqrt(P_(1, 1));
    state.sigmaZ = is3D_ ? std::sqrt(P_(2, 2)) : 0.0;
    state.sigmaVx = std::sqrt(P_(velOffset, velOffset));
    state.sigmaVy = std::sqrt(P_(velOffset + 1, velOffset + 1));
    state.sigmaVz = is3D_ ? std::sqrt(P_(velOffset + 2, velOffset + 2)) : 0.0;
    
    state.covariance = P_.data();
    state.dimension = stateDim_;
    state.motionModel = motionModel_;
    state.timestamp = lastUpdateTime_;
    
    return state;
}

} // namespace zoppler
