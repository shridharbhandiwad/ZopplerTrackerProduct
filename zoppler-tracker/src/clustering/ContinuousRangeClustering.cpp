/**
 * @file ContinuousRangeClustering.cpp
 * @brief Continuous range clustering implementation
 * @copyright Defense-grade radar tracking system
 */

#include "zoppler/clustering/ContinuousRangeClustering.hpp"
#include "zoppler/utils/MathUtils.hpp"
#include <algorithm>
#include <cmath>
#include <map>

namespace zoppler {

ContinuousRangeClustering::ContinuousRangeClustering(
    double rangeGate, double azimuthGate, double elevationGate, double dopplerGate)
    : rangeGate_(rangeGate)
    , azimuthGate_(azimuthGate)
    , elevationGate_(elevationGate)
    , dopplerGate_(dopplerGate)
    , use3D_(true)
    , useDoppler_(false) {
}

ContinuousRangeClustering::ContinuousRangeClustering(const TrackerConfig& config)
    : rangeGate_(config.clusteringParams.rangeGate)
    , azimuthGate_(config.clusteringParams.azimuthGate)
    , elevationGate_(config.clusteringParams.elevationGate)
    , dopplerGate_(config.clusteringParams.dopplerGate)
    , use3D_(config.clusteringParams.use3D && config.radar.is3D)
    , useDoppler_(config.clusteringParams.useDoppler && config.radar.hasDoppler) {
}

std::unique_ptr<IClustering> ContinuousRangeClustering::clone() const {
    auto copy = std::make_unique<ContinuousRangeClustering>(
        rangeGate_, azimuthGate_, elevationGate_, dopplerGate_);
    copy->use3D_ = use3D_;
    copy->useDoppler_ = useDoppler_;
    return copy;
}

bool ContinuousRangeClustering::inSameGate(const Detection& a, const Detection& b) const {
    // Check range gate
    if (!a.hasValidPolar() || !b.hasValidPolar()) {
        // Fall back to Cartesian distance if no polar coordinates
        double dist = a.distanceTo(b);
        return isValid(dist) && dist <= rangeGate_ * 2;
    }
    
    // Range gating
    if (std::abs(a.range - b.range) > rangeGate_) {
        return false;
    }
    
    // Azimuth gating (account for wraparound)
    double azDiff = std::abs(MathUtils::angleDifference(a.azimuth, b.azimuth));
    if (azDiff > azimuthGate_) {
        return false;
    }
    
    // Elevation gating (if 3D)
    if (use3D_ && isValid(a.elevation) && isValid(b.elevation)) {
        if (std::abs(a.elevation - b.elevation) > elevationGate_) {
            return false;
        }
    }
    
    // Doppler gating (if enabled)
    if (useDoppler_ && a.hasDoppler() && b.hasDoppler()) {
        if (std::abs(a.doppler - b.doppler) > dopplerGate_) {
            return false;
        }
    }
    
    return true;
}

int ContinuousRangeClustering::findRoot(std::vector<int>& parent, int idx) const {
    if (parent[idx] != idx) {
        parent[idx] = findRoot(parent, parent[idx]);  // Path compression
    }
    return parent[idx];
}

void ContinuousRangeClustering::unionClusters(std::vector<int>& parent,
                                               std::vector<int>& rank,
                                               int a, int b) const {
    int rootA = findRoot(parent, a);
    int rootB = findRoot(parent, b);
    
    if (rootA != rootB) {
        // Union by rank
        if (rank[rootA] < rank[rootB]) {
            parent[rootA] = rootB;
        } else if (rank[rootA] > rank[rootB]) {
            parent[rootB] = rootA;
        } else {
            parent[rootB] = rootA;
            rank[rootA]++;
        }
    }
}

std::vector<Cluster> ContinuousRangeClustering::cluster(
    const std::vector<Detection>& detections) {
    
    std::vector<Cluster> clusters;
    
    if (detections.empty()) {
        return clusters;
    }
    
    size_t n = detections.size();
    
    // Sort detections by range for efficient sequential processing
    std::vector<size_t> sortedIndices(n);
    for (size_t i = 0; i < n; ++i) {
        sortedIndices[i] = i;
    }
    
    std::sort(sortedIndices.begin(), sortedIndices.end(),
        [&detections](size_t a, size_t b) {
            return detections[a].range < detections[b].range;
        });
    
    // Union-find data structures
    std::vector<int> parent(n);
    std::vector<int> rank(n, 0);
    for (size_t i = 0; i < n; ++i) {
        parent[i] = static_cast<int>(i);
    }
    
    // Process detections in range order
    for (size_t i = 0; i < n; ++i) {
        size_t idx_i = sortedIndices[i];
        const Detection& det_i = detections[idx_i];
        
        // Only need to check nearby detections in range
        for (size_t j = i + 1; j < n; ++j) {
            size_t idx_j = sortedIndices[j];
            const Detection& det_j = detections[idx_j];
            
            // Early termination if range difference exceeds gate
            if (det_j.range - det_i.range > rangeGate_) {
                break;
            }
            
            // Check if detections should be in same cluster
            if (inSameGate(det_i, det_j)) {
                unionClusters(parent, rank, static_cast<int>(idx_i), static_cast<int>(idx_j));
            }
        }
    }
    
    // Build clusters from union-find result
    std::map<int, Cluster> clusterMap;
    int nextClusterId = 0;
    std::map<int, int> rootToClusterId;
    
    for (size_t i = 0; i < n; ++i) {
        int root = findRoot(parent, static_cast<int>(i));
        
        if (rootToClusterId.find(root) == rootToClusterId.end()) {
            rootToClusterId[root] = nextClusterId++;
            clusterMap[rootToClusterId[root]].id = rootToClusterId[root];
        }
        
        clusterMap[rootToClusterId[root]].detectionIndices.push_back(i);
    }
    
    // Convert map to vector and compute centroids
    clusters.reserve(clusterMap.size());
    for (auto& [id, cluster] : clusterMap) {
        cluster.computeCentroid(detections);
        cluster.computeExtent(detections);
        clusters.push_back(std::move(cluster));
    }
    
    return clusters;
}

} // namespace zoppler
