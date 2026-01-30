/**
 * @file Matrix.hpp
 * @brief STL-only matrix operations for Kalman filtering
 * @copyright Defense-grade radar tracking system
 */

#ifndef ZOPPLER_UTILS_MATRIX_HPP
#define ZOPPLER_UTILS_MATRIX_HPP

#include <vector>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <numeric>

namespace zoppler {

/**
 * @brief Simple matrix class using STL vectors
 * 
 * Row-major storage for basic linear algebra operations.
 * Suitable for small matrices typical in tracking applications.
 */
class Matrix {
public:
    /**
     * @brief Default constructor (empty matrix)
     */
    Matrix() : rows_(0), cols_(0) {}
    
    /**
     * @brief Construct matrix with dimensions
     */
    Matrix(size_t rows, size_t cols, double value = 0.0)
        : rows_(rows), cols_(cols), data_(rows * cols, value) {}
    
    /**
     * @brief Construct from initializer list (row-major)
     */
    Matrix(size_t rows, size_t cols, std::initializer_list<double> values)
        : rows_(rows), cols_(cols), data_(values) {
        if (data_.size() != rows * cols) {
            data_.resize(rows * cols, 0.0);
        }
    }
    
    /**
     * @brief Construct from vector (column vector)
     */
    explicit Matrix(const std::vector<double>& vec)
        : rows_(vec.size()), cols_(1), data_(vec) {}
    
    /**
     * @brief Create identity matrix
     */
    static Matrix identity(size_t n) {
        Matrix I(n, n, 0.0);
        for (size_t i = 0; i < n; ++i) {
            I(i, i) = 1.0;
        }
        return I;
    }
    
    /**
     * @brief Create zero matrix
     */
    static Matrix zeros(size_t rows, size_t cols) {
        return Matrix(rows, cols, 0.0);
    }
    
    /**
     * @brief Create diagonal matrix from vector
     */
    static Matrix diagonal(const std::vector<double>& diag) {
        size_t n = diag.size();
        Matrix D(n, n, 0.0);
        for (size_t i = 0; i < n; ++i) {
            D(i, i) = diag[i];
        }
        return D;
    }
    
    // Accessors
    size_t rows() const { return rows_; }
    size_t cols() const { return cols_; }
    size_t size() const { return data_.size(); }
    bool empty() const { return data_.empty(); }
    
    /**
     * @brief Element access (row, col)
     */
    double& operator()(size_t row, size_t col) {
        return data_[row * cols_ + col];
    }
    
    const double& operator()(size_t row, size_t col) const {
        return data_[row * cols_ + col];
    }
    
    /**
     * @brief Linear access
     */
    double& operator[](size_t idx) { return data_[idx]; }
    const double& operator[](size_t idx) const { return data_[idx]; }
    
    /**
     * @brief Get raw data
     */
    std::vector<double>& data() { return data_; }
    const std::vector<double>& data() const { return data_; }
    
    /**
     * @brief Matrix addition
     */
    Matrix operator+(const Matrix& other) const {
        if (rows_ != other.rows_ || cols_ != other.cols_) {
            throw std::runtime_error("Matrix dimension mismatch in addition");
        }
        Matrix result(rows_, cols_);
        for (size_t i = 0; i < data_.size(); ++i) {
            result.data_[i] = data_[i] + other.data_[i];
        }
        return result;
    }
    
    Matrix& operator+=(const Matrix& other) {
        if (rows_ != other.rows_ || cols_ != other.cols_) {
            throw std::runtime_error("Matrix dimension mismatch in addition");
        }
        for (size_t i = 0; i < data_.size(); ++i) {
            data_[i] += other.data_[i];
        }
        return *this;
    }
    
    /**
     * @brief Matrix subtraction
     */
    Matrix operator-(const Matrix& other) const {
        if (rows_ != other.rows_ || cols_ != other.cols_) {
            throw std::runtime_error("Matrix dimension mismatch in subtraction");
        }
        Matrix result(rows_, cols_);
        for (size_t i = 0; i < data_.size(); ++i) {
            result.data_[i] = data_[i] - other.data_[i];
        }
        return result;
    }
    
    Matrix& operator-=(const Matrix& other) {
        if (rows_ != other.rows_ || cols_ != other.cols_) {
            throw std::runtime_error("Matrix dimension mismatch in subtraction");
        }
        for (size_t i = 0; i < data_.size(); ++i) {
            data_[i] -= other.data_[i];
        }
        return *this;
    }
    
    /**
     * @brief Scalar multiplication
     */
    Matrix operator*(double scalar) const {
        Matrix result(rows_, cols_);
        for (size_t i = 0; i < data_.size(); ++i) {
            result.data_[i] = data_[i] * scalar;
        }
        return result;
    }
    
    Matrix& operator*=(double scalar) {
        for (auto& v : data_) v *= scalar;
        return *this;
    }
    
    friend Matrix operator*(double scalar, const Matrix& m) {
        return m * scalar;
    }
    
    /**
     * @brief Matrix multiplication
     */
    Matrix operator*(const Matrix& other) const {
        if (cols_ != other.rows_) {
            throw std::runtime_error("Matrix dimension mismatch in multiplication");
        }
        Matrix result(rows_, other.cols_, 0.0);
        for (size_t i = 0; i < rows_; ++i) {
            for (size_t j = 0; j < other.cols_; ++j) {
                double sum = 0.0;
                for (size_t k = 0; k < cols_; ++k) {
                    sum += (*this)(i, k) * other(k, j);
                }
                result(i, j) = sum;
            }
        }
        return result;
    }
    
    /**
     * @brief Transpose
     */
    Matrix transpose() const {
        Matrix result(cols_, rows_);
        for (size_t i = 0; i < rows_; ++i) {
            for (size_t j = 0; j < cols_; ++j) {
                result(j, i) = (*this)(i, j);
            }
        }
        return result;
    }
    
    /**
     * @brief Matrix inverse using Gauss-Jordan elimination
     * 
     * For small matrices typical in tracking (up to ~10x10)
     */
    Matrix inverse() const {
        if (rows_ != cols_) {
            throw std::runtime_error("Cannot invert non-square matrix");
        }
        
        size_t n = rows_;
        Matrix aug(n, 2 * n, 0.0);
        
        // Create augmented matrix [A | I]
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < n; ++j) {
                aug(i, j) = (*this)(i, j);
            }
            aug(i, n + i) = 1.0;
        }
        
        // Gauss-Jordan elimination with partial pivoting
        for (size_t col = 0; col < n; ++col) {
            // Find pivot
            size_t maxRow = col;
            double maxVal = std::abs(aug(col, col));
            for (size_t row = col + 1; row < n; ++row) {
                if (std::abs(aug(row, col)) > maxVal) {
                    maxVal = std::abs(aug(row, col));
                    maxRow = row;
                }
            }
            
            // Swap rows
            if (maxRow != col) {
                for (size_t j = 0; j < 2 * n; ++j) {
                    std::swap(aug(col, j), aug(maxRow, j));
                }
            }
            
            // Check for singular matrix
            double pivot = aug(col, col);
            if (std::abs(pivot) < 1e-12) {
                throw std::runtime_error("Matrix is singular or nearly singular");
            }
            
            // Scale pivot row
            for (size_t j = 0; j < 2 * n; ++j) {
                aug(col, j) /= pivot;
            }
            
            // Eliminate column
            for (size_t row = 0; row < n; ++row) {
                if (row != col) {
                    double factor = aug(row, col);
                    for (size_t j = 0; j < 2 * n; ++j) {
                        aug(row, j) -= factor * aug(col, j);
                    }
                }
            }
        }
        
        // Extract inverse
        Matrix inv(n, n);
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < n; ++j) {
                inv(i, j) = aug(i, n + j);
            }
        }
        
        return inv;
    }
    
    /**
     * @brief Safe inverse with fallback to pseudo-inverse for ill-conditioned matrices
     */
    Matrix safeInverse() const {
        try {
            return inverse();
        } catch (const std::runtime_error&) {
            // Add small regularization to diagonal
            Matrix reg = *this;
            for (size_t i = 0; i < std::min(rows_, cols_); ++i) {
                reg(i, i) += 1e-6;
            }
            return reg.inverse();
        }
    }
    
    /**
     * @brief Cholesky decomposition (lower triangular)
     * 
     * Matrix must be symmetric positive definite
     */
    Matrix cholesky() const {
        if (rows_ != cols_) {
            throw std::runtime_error("Cholesky requires square matrix");
        }
        
        size_t n = rows_;
        Matrix L(n, n, 0.0);
        
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j <= i; ++j) {
                double sum = 0.0;
                
                if (i == j) {
                    for (size_t k = 0; k < j; ++k) {
                        sum += L(j, k) * L(j, k);
                    }
                    double diag = (*this)(j, j) - sum;
                    if (diag <= 0) {
                        // Not positive definite, add regularization
                        diag = 1e-10;
                    }
                    L(j, j) = std::sqrt(diag);
                } else {
                    for (size_t k = 0; k < j; ++k) {
                        sum += L(i, k) * L(j, k);
                    }
                    if (std::abs(L(j, j)) < 1e-12) {
                        L(i, j) = 0.0;
                    } else {
                        L(i, j) = ((*this)(i, j) - sum) / L(j, j);
                    }
                }
            }
        }
        
        return L;
    }
    
    /**
     * @brief Compute determinant (for small matrices)
     */
    double determinant() const {
        if (rows_ != cols_) {
            throw std::runtime_error("Determinant requires square matrix");
        }
        
        if (rows_ == 1) return data_[0];
        if (rows_ == 2) return data_[0] * data_[3] - data_[1] * data_[2];
        if (rows_ == 3) {
            return data_[0] * (data_[4] * data_[8] - data_[5] * data_[7])
                 - data_[1] * (data_[3] * data_[8] - data_[5] * data_[6])
                 + data_[2] * (data_[3] * data_[7] - data_[4] * data_[6]);
        }
        
        // LU decomposition for larger matrices
        size_t n = rows_;
        Matrix lu = *this;
        double det = 1.0;
        
        for (size_t i = 0; i < n; ++i) {
            // Partial pivoting
            size_t maxRow = i;
            for (size_t k = i + 1; k < n; ++k) {
                if (std::abs(lu(k, i)) > std::abs(lu(maxRow, i))) {
                    maxRow = k;
                }
            }
            
            if (maxRow != i) {
                for (size_t j = 0; j < n; ++j) {
                    std::swap(lu(i, j), lu(maxRow, j));
                }
                det *= -1;
            }
            
            if (std::abs(lu(i, i)) < 1e-12) {
                return 0.0;
            }
            
            det *= lu(i, i);
            
            for (size_t k = i + 1; k < n; ++k) {
                lu(k, i) /= lu(i, i);
                for (size_t j = i + 1; j < n; ++j) {
                    lu(k, j) -= lu(k, i) * lu(i, j);
                }
            }
        }
        
        return det;
    }
    
    /**
     * @brief Compute trace
     */
    double trace() const {
        double tr = 0.0;
        for (size_t i = 0; i < std::min(rows_, cols_); ++i) {
            tr += (*this)(i, i);
        }
        return tr;
    }
    
    /**
     * @brief Frobenius norm
     */
    double norm() const {
        double sum = 0.0;
        for (const auto& v : data_) {
            sum += v * v;
        }
        return std::sqrt(sum);
    }
    
    /**
     * @brief Get submatrix
     */
    Matrix block(size_t startRow, size_t startCol, size_t blockRows, size_t blockCols) const {
        Matrix result(blockRows, blockCols);
        for (size_t i = 0; i < blockRows; ++i) {
            for (size_t j = 0; j < blockCols; ++j) {
                result(i, j) = (*this)(startRow + i, startCol + j);
            }
        }
        return result;
    }
    
    /**
     * @brief Set submatrix
     */
    void setBlock(size_t startRow, size_t startCol, const Matrix& block) {
        for (size_t i = 0; i < block.rows(); ++i) {
            for (size_t j = 0; j < block.cols(); ++j) {
                (*this)(startRow + i, startCol + j) = block(i, j);
            }
        }
    }
    
    /**
     * @brief Get column vector
     */
    Matrix col(size_t c) const {
        Matrix result(rows_, 1);
        for (size_t i = 0; i < rows_; ++i) {
            result(i, 0) = (*this)(i, c);
        }
        return result;
    }
    
    /**
     * @brief Get row vector
     */
    Matrix row(size_t r) const {
        Matrix result(1, cols_);
        for (size_t j = 0; j < cols_; ++j) {
            result(0, j) = (*this)(r, j);
        }
        return result;
    }
    
    /**
     * @brief Set column
     */
    void setCol(size_t c, const Matrix& vec) {
        for (size_t i = 0; i < std::min(rows_, vec.rows()); ++i) {
            (*this)(i, c) = vec(i, 0);
        }
    }
    
    /**
     * @brief Set row
     */
    void setRow(size_t r, const Matrix& vec) {
        for (size_t j = 0; j < std::min(cols_, vec.cols()); ++j) {
            (*this)(r, j) = vec(0, j);
        }
    }
    
    /**
     * @brief Convert to vector (for column vectors)
     */
    std::vector<double> toVector() const {
        return data_;
    }
    
    /**
     * @brief Resize matrix (data preserved where possible)
     */
    void resize(size_t newRows, size_t newCols) {
        std::vector<double> newData(newRows * newCols, 0.0);
        for (size_t i = 0; i < std::min(rows_, newRows); ++i) {
            for (size_t j = 0; j < std::min(cols_, newCols); ++j) {
                newData[i * newCols + j] = (*this)(i, j);
            }
        }
        rows_ = newRows;
        cols_ = newCols;
        data_ = std::move(newData);
    }
    
    /**
     * @brief Fill with value
     */
    void fill(double value) {
        std::fill(data_.begin(), data_.end(), value);
    }
    
private:
    size_t rows_;
    size_t cols_;
    std::vector<double> data_;
};

// Utility functions

/**
 * @brief Outer product of two vectors
 */
inline Matrix outerProduct(const Matrix& a, const Matrix& b) {
    Matrix result(a.rows(), b.rows());
    for (size_t i = 0; i < a.rows(); ++i) {
        for (size_t j = 0; j < b.rows(); ++j) {
            result(i, j) = a(i, 0) * b(j, 0);
        }
    }
    return result;
}

/**
 * @brief Dot product of two vectors
 */
inline double dotProduct(const Matrix& a, const Matrix& b) {
    double sum = 0.0;
    for (size_t i = 0; i < std::min(a.size(), b.size()); ++i) {
        sum += a[i] * b[i];
    }
    return sum;
}

/**
 * @brief Compute Mahalanobis distance squared
 */
inline double mahalanobisDistSq(const Matrix& diff, const Matrix& Sinv) {
    Matrix temp = Sinv * diff;
    return dotProduct(diff, temp);
}

} // namespace zoppler

#endif // ZOPPLER_UTILS_MATRIX_HPP
