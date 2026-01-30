/**
 * @file MathUtils.hpp
 * @brief Mathematical utility functions for tracking
 * @copyright Defense-grade radar tracking system
 */

#ifndef ZOPPLER_UTILS_MATHUTILS_HPP
#define ZOPPLER_UTILS_MATHUTILS_HPP

#include "../core/Types.hpp"
#include "Matrix.hpp"
#include <cmath>
#include <random>
#include <algorithm>

namespace zoppler {

/**
 * @brief Utility class for mathematical operations
 */
class MathUtils {
public:
    /**
     * @brief Normalize angle to [-pi, pi]
     */
    static double normalizeAngle(double angle) {
        while (angle > PI) angle -= 2.0 * PI;
        while (angle < -PI) angle += 2.0 * PI;
        return angle;
    }
    
    /**
     * @brief Normalize angle to [0, 2*pi]
     */
    static double normalizeAnglePositive(double angle) {
        while (angle < 0) angle += 2.0 * PI;
        while (angle >= 2.0 * PI) angle -= 2.0 * PI;
        return angle;
    }
    
    /**
     * @brief Compute angular difference (accounting for wraparound)
     */
    static double angleDifference(double a1, double a2) {
        return normalizeAngle(a1 - a2);
    }
    
    /**
     * @brief Convert degrees to radians
     */
    static double degToRad(double deg) {
        return deg * DEG_TO_RAD;
    }
    
    /**
     * @brief Convert radians to degrees
     */
    static double radToDeg(double rad) {
        return rad * RAD_TO_DEG;
    }
    
    /**
     * @brief Clamp value to range
     */
    static double clamp(double value, double minVal, double maxVal) {
        return std::max(minVal, std::min(maxVal, value));
    }
    
    /**
     * @brief Sign function
     */
    static double sign(double x) {
        return (x > 0) ? 1.0 : ((x < 0) ? -1.0 : 0.0);
    }
    
    /**
     * @brief Square function
     */
    static double sq(double x) {
        return x * x;
    }
    
    /**
     * @brief Gaussian PDF evaluation
     */
    static double gaussianPdf(double x, double mean, double stdDev) {
        double z = (x - mean) / stdDev;
        return std::exp(-0.5 * z * z) / (stdDev * std::sqrt(2.0 * PI));
    }
    
    /**
     * @brief Multivariate Gaussian PDF evaluation
     */
    static double mvnPdf(const Matrix& x, const Matrix& mean, const Matrix& covariance) {
        size_t n = x.rows();
        Matrix diff = x - mean;
        Matrix Sinv = covariance.safeInverse();
        double det = covariance.determinant();
        
        if (det <= 0) det = 1e-10;
        
        double exponent = -0.5 * mahalanobisDistSq(diff, Sinv);
        double normConst = std::pow(2.0 * PI, -static_cast<double>(n) / 2.0) 
                         * std::pow(det, -0.5);
        
        return normConst * std::exp(exponent);
    }
    
    /**
     * @brief Chi-squared CDF (approximate for small DOF)
     */
    static double chiSquaredCdf(double x, int dof) {
        if (x < 0) return 0.0;
        if (dof <= 0) return 0.0;
        
        // Use incomplete gamma function approximation
        double k = dof / 2.0;
        double theta = 2.0;
        
        // Simple approximation using series expansion
        double sum = 0.0;
        double term = std::exp(-x / theta) * std::pow(x / theta, k) / tgamma(k + 1);
        
        for (int i = 0; i < 100 && term > 1e-10; ++i) {
            sum += term;
            term *= (x / theta) / (k + i + 1);
        }
        
        return 1.0 - sum;
    }
    
    /**
     * @brief Chi-squared quantile (inverse CDF)
     * 
     * Common values for gating:
     * - DOF=2: 95%->5.99, 99%->9.21
     * - DOF=3: 95%->7.81, 99%->11.34
     */
    static double chiSquaredQuantile(double p, int dof) {
        // Look-up table for common values
        if (dof == 2) {
            if (std::abs(p - 0.90) < 0.01) return 4.61;
            if (std::abs(p - 0.95) < 0.01) return 5.99;
            if (std::abs(p - 0.99) < 0.01) return 9.21;
        } else if (dof == 3) {
            if (std::abs(p - 0.90) < 0.01) return 6.25;
            if (std::abs(p - 0.95) < 0.01) return 7.81;
            if (std::abs(p - 0.99) < 0.01) return 11.34;
        } else if (dof == 4) {
            if (std::abs(p - 0.90) < 0.01) return 7.78;
            if (std::abs(p - 0.95) < 0.01) return 9.49;
            if (std::abs(p - 0.99) < 0.01) return 13.28;
        }
        
        // Wilson-Hilferty approximation for other values
        double h = 2.0 / (9.0 * dof);
        double z = normalQuantile(p);
        double chi = dof * std::pow(1.0 - h + z * std::sqrt(h), 3);
        return std::max(0.0, chi);
    }
    
    /**
     * @brief Standard normal quantile (inverse CDF)
     */
    static double normalQuantile(double p) {
        // Rational approximation
        if (p <= 0) return -std::numeric_limits<double>::infinity();
        if (p >= 1) return std::numeric_limits<double>::infinity();
        if (std::abs(p - 0.5) < 1e-10) return 0.0;
        
        double t = (p < 0.5) ? std::sqrt(-2.0 * std::log(p))
                            : std::sqrt(-2.0 * std::log(1.0 - p));
        
        // Coefficients for rational approximation
        double c0 = 2.515517;
        double c1 = 0.802853;
        double c2 = 0.010328;
        double d1 = 1.432788;
        double d2 = 0.189269;
        double d3 = 0.001308;
        
        double x = t - (c0 + c1 * t + c2 * t * t) 
                     / (1.0 + d1 * t + d2 * t * t + d3 * t * t * t);
        
        return (p < 0.5) ? -x : x;
    }
    
    /**
     * @brief Hungarian algorithm for assignment problem
     * 
     * Finds optimal assignment minimizing total cost.
     * Returns vector where result[i] is the column assigned to row i.
     */
    static std::vector<int> hungarianAssignment(const std::vector<std::vector<double>>& costMatrix) {
        if (costMatrix.empty()) return {};
        
        int n = static_cast<int>(costMatrix.size());
        int m = static_cast<int>(costMatrix[0].size());
        
        // Make square matrix by padding
        int dim = std::max(n, m);
        std::vector<std::vector<double>> cost(dim, std::vector<double>(dim, 0.0));
        
        double maxCost = 0.0;
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < m; ++j) {
                cost[i][j] = costMatrix[i][j];
                maxCost = std::max(maxCost, cost[i][j]);
            }
        }
        
        // Fill padding with large value
        double padding = maxCost * 10 + 1e6;
        for (int i = n; i < dim; ++i) {
            for (int j = 0; j < dim; ++j) cost[i][j] = padding;
        }
        for (int i = 0; i < dim; ++i) {
            for (int j = m; j < dim; ++j) cost[i][j] = padding;
        }
        
        // Hungarian algorithm implementation
        std::vector<double> u(dim + 1), v(dim + 1);
        std::vector<int> p(dim + 1), way(dim + 1);
        
        for (int i = 1; i <= dim; ++i) {
            p[0] = i;
            int j0 = 0;
            std::vector<double> minv(dim + 1, std::numeric_limits<double>::infinity());
            std::vector<bool> used(dim + 1, false);
            
            do {
                used[j0] = true;
                int i0 = p[j0];
                double delta = std::numeric_limits<double>::infinity();
                int j1 = 0;
                
                for (int j = 1; j <= dim; ++j) {
                    if (!used[j]) {
                        double cur = cost[i0 - 1][j - 1] - u[i0] - v[j];
                        if (cur < minv[j]) {
                            minv[j] = cur;
                            way[j] = j0;
                        }
                        if (minv[j] < delta) {
                            delta = minv[j];
                            j1 = j;
                        }
                    }
                }
                
                for (int j = 0; j <= dim; ++j) {
                    if (used[j]) {
                        u[p[j]] += delta;
                        v[j] -= delta;
                    } else {
                        minv[j] -= delta;
                    }
                }
                
                j0 = j1;
            } while (p[j0] != 0);
            
            do {
                int j1 = way[j0];
                p[j0] = p[j1];
                j0 = j1;
            } while (j0);
        }
        
        // Extract assignment
        std::vector<int> assignment(n, -1);
        for (int j = 1; j <= dim; ++j) {
            if (p[j] > 0 && p[j] <= n && j <= m) {
                assignment[p[j] - 1] = j - 1;
            }
        }
        
        return assignment;
    }
    
    /**
     * @brief Coordinate conversion: Cartesian to Spherical
     */
    static void cartesianToSpherical(double x, double y, double z,
                                      double& range, double& azimuth, double& elevation) {
        range = std::sqrt(x * x + y * y + z * z);
        azimuth = std::atan2(y, x);
        elevation = (range > 0) ? std::asin(z / range) : 0.0;
    }
    
    /**
     * @brief Coordinate conversion: Spherical to Cartesian
     */
    static void sphericalToCartesian(double range, double azimuth, double elevation,
                                      double& x, double& y, double& z) {
        double cosEl = std::cos(elevation);
        x = range * std::cos(azimuth) * cosEl;
        y = range * std::sin(azimuth) * cosEl;
        z = range * std::sin(elevation);
    }
    
    /**
     * @brief Generate random samples from multivariate Gaussian
     */
    static std::vector<Matrix> sampleMvnormal(const Matrix& mean, const Matrix& covariance,
                                               int numSamples, std::mt19937& rng) {
        std::normal_distribution<double> normalDist(0.0, 1.0);
        
        Matrix L = covariance.cholesky();
        size_t n = mean.rows();
        
        std::vector<Matrix> samples;
        samples.reserve(numSamples);
        
        for (int s = 0; s < numSamples; ++s) {
            Matrix z(n, 1);
            for (size_t i = 0; i < n; ++i) {
                z(i, 0) = normalDist(rng);
            }
            
            Matrix sample = mean + L * z;
            samples.push_back(sample);
        }
        
        return samples;
    }
    
    /**
     * @brief Weighted mean of matrices
     */
    static Matrix weightedMean(const std::vector<Matrix>& matrices,
                                const std::vector<double>& weights) {
        if (matrices.empty()) return Matrix();
        
        Matrix result(matrices[0].rows(), matrices[0].cols(), 0.0);
        double totalWeight = 0.0;
        
        for (size_t i = 0; i < matrices.size(); ++i) {
            double w = (i < weights.size()) ? weights[i] : 1.0;
            result += matrices[i] * w;
            totalWeight += w;
        }
        
        if (totalWeight > 0) {
            result *= (1.0 / totalWeight);
        }
        
        return result;
    }
    
    /**
     * @brief Weighted covariance
     */
    static Matrix weightedCovariance(const std::vector<Matrix>& samples,
                                      const Matrix& mean,
                                      const std::vector<double>& weights) {
        if (samples.empty()) return Matrix();
        
        size_t n = mean.rows();
        Matrix cov(n, n, 0.0);
        double totalWeight = 0.0;
        
        for (size_t i = 0; i < samples.size(); ++i) {
            double w = (i < weights.size()) ? weights[i] : 1.0;
            Matrix diff = samples[i] - mean;
            cov += outerProduct(diff, diff) * w;
            totalWeight += w;
        }
        
        if (totalWeight > 0) {
            cov *= (1.0 / totalWeight);
        }
        
        return cov;
    }
};

} // namespace zoppler

#endif // ZOPPLER_UTILS_MATHUTILS_HPP
