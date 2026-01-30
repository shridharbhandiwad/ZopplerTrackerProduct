/**
 * @file EKFTracker.cpp
 * @brief EKF tracker implementation
 * @copyright Defense-grade radar tracking system
 */

#include "zoppler/tracking/EKFTracker.hpp"
#include "zoppler/utils/MathUtils.hpp"
#include <cmath>

namespace zoppler {

// ITrackerModel base implementations
void ITrackerModel::initialize(const Measurement& meas,
                               double initialVelStd,
                               double initialAccStd) {
    // Default implementation - subclasses override
    (void)meas;
    (void)initialVelStd;
    (void)initialAccStd;
}

void ITrackerModel::setProcessNoise(double qPos, double qVel, double qAcc) {
    qPos_ = qPos;
    qVel_ = qVel;
    qAcc_ = qAcc;
}

// EKFTracker implementation
EKFTracker::EKFTracker(MotionModel motionModel, bool is3D)
    : motionModel_(motionModel)
    , is3D_(is3D) {
    
    // Set state dimension based on motion model
    switch (motionModel) {
        case MotionModel::CV:
            stateDim_ = is3D ? 6 : 4;  // [x, y, (z), vx, vy, (vz)]
            break;
        case MotionModel::CA:
            stateDim_ = is3D ? 9 : 6;  // [x, y, (z), vx, vy, (vz), ax, ay, (az)]
            break;
        case MotionModel::CT:
            stateDim_ = is3D ? 7 : 5;  // [x, y, (z), vx, vy, (vz), omega]
            break;
        default:
            stateDim_ = is3D ? 6 : 4;
            break;
    }
    
    measDim_ = is3D ? 3 : 2;
    
    // Initialize state and covariance
    x_ = Matrix(stateDim_, 1, 0.0);
    P_ = Matrix::identity(stateDim_) * 100.0;
}

EKFTracker::EKFTracker(const TrackerConfig& config)
    : EKFTracker(config.getMotionModel(), config.radar.is3D) {
    
    qPos_ = config.trackerProfile.qPos;
    qVel_ = config.trackerProfile.qVel;
    qAcc_ = config.trackerProfile.qAcc;
    
    sigmaX_ = config.radar.xStd;
    sigmaY_ = config.radar.yStd;
    sigmaZ_ = config.radar.zStd;
}

std::string EKFTracker::getName() const {
    switch (motionModel_) {
        case MotionModel::CV: return "EKF-CV";
        case MotionModel::CA: return "EKF-CA";
        case MotionModel::CT: return "EKF-CT";
        default: return "EKF";
    }
}

std::unique_ptr<ITrackerModel> EKFTracker::clone() const {
    auto copy = std::make_unique<EKFTracker>(motionModel_, is3D_);
    copy->x_ = x_;
    copy->P_ = P_;
    copy->qPos_ = qPos_;
    copy->qVel_ = qVel_;
    copy->qAcc_ = qAcc_;
    copy->sigmaX_ = sigmaX_;
    copy->sigmaY_ = sigmaY_;
    copy->sigmaZ_ = sigmaZ_;
    return copy;
}

void EKFTracker::initialize(const Measurement& meas,
                            double initialVelStd,
                            double initialAccStd) {
    // Initialize position from measurement
    if (meas.z.size() >= 2) {
        x_(0, 0) = meas.z[0];  // x
        x_(1, 0) = meas.z[1];  // y
        
        if (is3D_ && meas.z.size() >= 3) {
            x_(2, 0) = meas.z[2];  // z
        }
    }
    
    // Initialize covariance
    P_ = Matrix::identity(stateDim_);
    
    // Position covariance from measurement
    int posOffset = 0;
    if (meas.R.size() >= 2) {
        P_(0, 0) = meas.R[0];
        P_(1, 1) = meas.R[1];
        if (is3D_ && meas.R.size() >= 3) {
            P_(2, 2) = meas.R[2];
        }
        posOffset = is3D_ ? 3 : 2;
    }
    
    // Velocity covariance
    double velVar = initialVelStd * initialVelStd;
    for (int i = 0; i < (is3D_ ? 3 : 2); ++i) {
        P_(posOffset + i, posOffset + i) = velVar;
    }
    
    // Acceleration covariance (for CA model)
    if (motionModel_ == MotionModel::CA) {
        int accOffset = posOffset + (is3D_ ? 3 : 2);
        double accVar = initialAccStd * initialAccStd;
        for (int i = 0; i < (is3D_ ? 3 : 2); ++i) {
            P_(accOffset + i, accOffset + i) = accVar;
        }
    }
    
    // Turn rate covariance (for CT model)
    if (motionModel_ == MotionModel::CT) {
        int omegaIdx = stateDim_ - 1;
        P_(omegaIdx, omegaIdx) = 0.1;  // Initial turn rate variance
    }
    
    lastUpdateTime_ = meas.timestamp;
}

Matrix EKFTracker::computeF(double dt) const {
    Matrix F = Matrix::identity(stateDim_);
    
    int posOffset = 0;
    int velOffset = is3D_ ? 3 : 2;
    
    switch (motionModel_) {
        case MotionModel::CV: {
            // Position depends on velocity
            for (int i = 0; i < (is3D_ ? 3 : 2); ++i) {
                F(posOffset + i, velOffset + i) = dt;
            }
            break;
        }
        
        case MotionModel::CA: {
            int accOffset = velOffset + (is3D_ ? 3 : 2);
            double dt2 = 0.5 * dt * dt;
            
            for (int i = 0; i < (is3D_ ? 3 : 2); ++i) {
                F(posOffset + i, velOffset + i) = dt;
                F(posOffset + i, accOffset + i) = dt2;
                F(velOffset + i, accOffset + i) = dt;
            }
            break;
        }
        
        case MotionModel::CT: {
            double omega = x_(stateDim_ - 1, 0);
            
            if (std::abs(omega) < 1e-6) {
                // Near-zero turn rate, use CV model
                for (int i = 0; i < 2; ++i) {
                    F(posOffset + i, velOffset + i) = dt;
                }
            } else {
                double sinOmegaT = std::sin(omega * dt);
                double cosOmegaT = std::cos(omega * dt);
                double sinOverOmega = sinOmegaT / omega;
                double oneMinusCosOverOmega = (1.0 - cosOmegaT) / omega;
                
                // x' = x + sin(wt)/w * vx + (1-cos(wt))/w * vy
                F(0, velOffset) = sinOverOmega;
                F(0, velOffset + 1) = oneMinusCosOverOmega;
                
                // y' = y - (1-cos(wt))/w * vx + sin(wt)/w * vy
                F(1, velOffset) = -oneMinusCosOverOmega;
                F(1, velOffset + 1) = sinOverOmega;
                
                // vx' = cos(wt) * vx + sin(wt) * vy
                F(velOffset, velOffset) = cosOmegaT;
                F(velOffset, velOffset + 1) = sinOmegaT;
                
                // vy' = -sin(wt) * vx + cos(wt) * vy
                F(velOffset + 1, velOffset) = -sinOmegaT;
                F(velOffset + 1, velOffset + 1) = cosOmegaT;
            }
            
            // z component (if 3D) - simple CV
            if (is3D_) {
                F(2, velOffset + 2) = dt;
            }
            break;
        }
        
        default:
            for (int i = 0; i < (is3D_ ? 3 : 2); ++i) {
                F(posOffset + i, velOffset + i) = dt;
            }
            break;
    }
    
    return F;
}

Matrix EKFTracker::computeQ(double dt) const {
    Matrix Q(stateDim_, stateDim_, 0.0);
    
    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    double dt4 = dt3 * dt;
    
    int posOffset = 0;
    int velOffset = is3D_ ? 3 : 2;
    
    switch (motionModel_) {
        case MotionModel::CV: {
            // Discrete white noise acceleration model
            for (int i = 0; i < (is3D_ ? 3 : 2); ++i) {
                Q(posOffset + i, posOffset + i) = qVel_ * dt4 / 4.0;
                Q(posOffset + i, velOffset + i) = qVel_ * dt3 / 2.0;
                Q(velOffset + i, posOffset + i) = qVel_ * dt3 / 2.0;
                Q(velOffset + i, velOffset + i) = qVel_ * dt2;
            }
            break;
        }
        
        case MotionModel::CA: {
            int accOffset = velOffset + (is3D_ ? 3 : 2);
            double dt5 = dt4 * dt;
            
            // Discrete white noise jerk model
            for (int i = 0; i < (is3D_ ? 3 : 2); ++i) {
                Q(posOffset + i, posOffset + i) = qAcc_ * dt5 / 20.0;
                Q(posOffset + i, velOffset + i) = qAcc_ * dt4 / 8.0;
                Q(posOffset + i, accOffset + i) = qAcc_ * dt3 / 6.0;
                
                Q(velOffset + i, posOffset + i) = qAcc_ * dt4 / 8.0;
                Q(velOffset + i, velOffset + i) = qAcc_ * dt3 / 3.0;
                Q(velOffset + i, accOffset + i) = qAcc_ * dt2 / 2.0;
                
                Q(accOffset + i, posOffset + i) = qAcc_ * dt3 / 6.0;
                Q(accOffset + i, velOffset + i) = qAcc_ * dt2 / 2.0;
                Q(accOffset + i, accOffset + i) = qAcc_ * dt;
            }
            break;
        }
        
        case MotionModel::CT: {
            // CV-like process noise for position/velocity
            for (int i = 0; i < (is3D_ ? 3 : 2); ++i) {
                Q(posOffset + i, posOffset + i) = qVel_ * dt4 / 4.0;
                Q(posOffset + i, velOffset + i) = qVel_ * dt3 / 2.0;
                Q(velOffset + i, posOffset + i) = qVel_ * dt3 / 2.0;
                Q(velOffset + i, velOffset + i) = qVel_ * dt2;
            }
            
            // Turn rate process noise
            Q(stateDim_ - 1, stateDim_ - 1) = qAcc_ * 0.01 * dt;
            break;
        }
        
        default:
            // Default CV process noise
            for (int i = 0; i < (is3D_ ? 3 : 2); ++i) {
                Q(posOffset + i, posOffset + i) = qVel_ * dt4 / 4.0;
                Q(velOffset + i, velOffset + i) = qVel_ * dt2;
            }
            break;
    }
    
    return Q;
}

Matrix EKFTracker::computeH() const {
    Matrix H(measDim_, stateDim_, 0.0);
    
    // Measurement is position only
    for (int i = 0; i < measDim_; ++i) {
        H(i, i) = 1.0;
    }
    
    return H;
}

Matrix EKFTracker::computeR(const Measurement& meas) const {
    Matrix R(measDim_, measDim_, 0.0);
    
    if (meas.R.size() >= static_cast<size_t>(measDim_)) {
        for (int i = 0; i < measDim_; ++i) {
            R(i, i) = meas.R[i];
        }
    } else {
        // Default measurement noise
        R(0, 0) = sigmaX_ * sigmaX_;
        R(1, 1) = sigmaY_ * sigmaY_;
        if (measDim_ > 2) {
            R(2, 2) = sigmaZ_ * sigmaZ_;
        }
    }
    
    return R;
}

void EKFTracker::predict(double dt) {
    if (dt <= 0) return;
    
    // State transition
    Matrix F = computeF(dt);
    Matrix Q = computeQ(dt);
    
    // Predict state
    x_ = F * x_;
    
    // Predict covariance
    P_ = F * P_ * F.transpose() + Q;
}

void EKFTracker::update(const Measurement& meas) {
    if (!meas.isValid()) return;
    
    Matrix H = computeH();
    Matrix R = computeR(meas);
    
    // Measurement vector
    Matrix z(measDim_, 1);
    for (int i = 0; i < measDim_ && i < static_cast<int>(meas.z.size()); ++i) {
        z(i, 0) = meas.z[i];
    }
    
    // Predicted measurement
    Matrix z_pred = H * x_;
    
    // Innovation
    innovation_ = z - z_pred;
    
    // Innovation covariance
    S_ = H * P_ * H.transpose() + R;
    
    // Kalman gain
    Matrix K = P_ * H.transpose() * S_.safeInverse();
    
    // Update state
    x_ = x_ + K * innovation_;
    
    // Update covariance (Joseph form for numerical stability)
    Matrix I = Matrix::identity(stateDim_);
    Matrix IKH = I - K * H;
    P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
    
    lastUpdateTime_ = meas.timestamp;
}

TrackState EKFTracker::getState() const {
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
    
    // Extract covariance diagonal
    state.sigmaX = std::sqrt(P_(0, 0));
    state.sigmaY = std::sqrt(P_(1, 1));
    state.sigmaZ = is3D_ ? std::sqrt(P_(2, 2)) : 0.0;
    state.sigmaVx = std::sqrt(P_(velOffset, velOffset));
    state.sigmaVy = std::sqrt(P_(velOffset + 1, velOffset + 1));
    state.sigmaVz = is3D_ ? std::sqrt(P_(velOffset + 2, velOffset + 2)) : 0.0;
    
    // Store full covariance
    state.covariance = P_.data();
    state.dimension = stateDim_;
    state.motionModel = motionModel_;
    state.timestamp = lastUpdateTime_;
    
    return state;
}

// CVTracker implementation
CVTracker::CVTracker(const TrackerConfig& config) 
    : EKFTracker(MotionModel::CV, config.radar.is3D) {
    qPos_ = config.trackerProfile.qPos;
    qVel_ = config.trackerProfile.qVel;
    sigmaX_ = config.radar.xStd;
    sigmaY_ = config.radar.yStd;
    sigmaZ_ = config.radar.zStd;
}

std::unique_ptr<ITrackerModel> CVTracker::clone() const {
    auto copy = std::make_unique<CVTracker>(is3D_);
    copy->x_ = x_;
    copy->P_ = P_;
    copy->qPos_ = qPos_;
    copy->qVel_ = qVel_;
    return copy;
}

// CATracker implementation
CATracker::CATracker(const TrackerConfig& config) 
    : EKFTracker(MotionModel::CA, config.radar.is3D) {
    qPos_ = config.trackerProfile.qPos;
    qVel_ = config.trackerProfile.qVel;
    qAcc_ = config.trackerProfile.qAcc;
    sigmaX_ = config.radar.xStd;
    sigmaY_ = config.radar.yStd;
    sigmaZ_ = config.radar.zStd;
}

std::unique_ptr<ITrackerModel> CATracker::clone() const {
    auto copy = std::make_unique<CATracker>(is3D_);
    copy->x_ = x_;
    copy->P_ = P_;
    copy->qPos_ = qPos_;
    copy->qVel_ = qVel_;
    copy->qAcc_ = qAcc_;
    return copy;
}

// CTTracker implementation
CTTracker::CTTracker(const TrackerConfig& config) 
    : EKFTracker(MotionModel::CT, config.radar.is3D) {
    qPos_ = config.trackerProfile.qPos;
    qVel_ = config.trackerProfile.qVel;
    qAcc_ = config.trackerProfile.qOmega;
    sigmaX_ = config.radar.xStd;
    sigmaY_ = config.radar.yStd;
    sigmaZ_ = config.radar.zStd;
}

std::unique_ptr<ITrackerModel> CTTracker::clone() const {
    auto copy = std::make_unique<CTTracker>(is3D_);
    copy->x_ = x_;
    copy->P_ = P_;
    copy->qPos_ = qPos_;
    copy->qVel_ = qVel_;
    copy->qAcc_ = qAcc_;
    return copy;
}

} // namespace zoppler
