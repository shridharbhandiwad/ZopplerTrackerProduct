/**
 * @file DBSCANClustering.cpp
 * @brief DBSCAN clustering implementation
 * @copyright Defense-grade radar tracking system
 */

#include "zoppler/clustering/DBSCANClustering.hpp"
#include <cmath>
#include <algorithm>

namespace zoppler {

DBSCANClustering::DBSCANClustering(double epsilon, int minPoints,
                                   bool use3D, bool useDoppler)
    : epsilon_(epsilon)
    , minPoints_(minPoints)
    , use3D_(use3D)
    , useDoppler_(useDoppler)
    , dopplerWeight_(1.0) {
}

DBSCANClustering::DBSCANClustering(const TrackerConfig& config)
    : epsilon_(config.clusteringParams.epsilon)
    , minPoints_(config.clusteringParams.minPoints)
    , use3D_(config.clusteringParams.use3D && config.radar.is3D)
    , useDoppler_(config.clusteringParams.useDoppler && config.radar.hasDoppler)
    , dopplerWeight_(1.0) {
}

std::unique_ptr<IClustering> DBSCANClustering::clone() const {
    auto copy = std::make_unique<DBSCANClustering>(epsilon_, minPoints_, use3D_, useDoppler_);
    copy->dopplerWeight_ = dopplerWeight_;
    return copy;
}

double DBSCANClustering::computeDistance(const Detection& a, const Detection& b) const {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double distSq = dx * dx + dy * dy;
    
    if (use3D_ && isValid(a.z) && isValid(b.z)) {
        double dz = a.z - b.z;
        distSq += dz * dz;
    }
    
    double dist = std::sqrt(distSq);
    
    // Add weighted Doppler difference
    if (useDoppler_ && a.hasDoppler() && b.hasDoppler()) {
        double dDoppler = std::abs(a.doppler - b.doppler);
        dist += dopplerWeight_ * dDoppler;
    }
    
    return dist;
}

std::vector<size_t> DBSCANClustering::findNeighbors(
    const std::vector<Detection>& detections, size_t pointIdx) const {
    
    std::vector<size_t> neighbors;
    const Detection& point = detections[pointIdx];
    
    for (size_t i = 0; i < detections.size(); ++i) {
        if (i != pointIdx) {
            double dist;
            if (useDistanceMatrix_ && !distanceMatrix_.empty()) {
                dist = distanceMatrix_[pointIdx][i];
            } else {
                dist = computeDistance(point, detections[i]);
            }
            
            if (dist <= epsilon_) {
                neighbors.push_back(i);
            }
        }
    }
    
    return neighbors;
}

void DBSCANClustering::expandCluster(
    const std::vector<Detection>& detections,
    size_t pointIdx,
    std::vector<size_t>& neighbors,
    int clusterId,
    std::vector<int>& labels) const {
    
    labels[pointIdx] = clusterId;
    
    size_t i = 0;
    while (i < neighbors.size()) {
        size_t neighborIdx = neighbors[i];
        
        if (labels[neighborIdx] == -1) {
            // Was marked as noise, now belongs to cluster
            labels[neighborIdx] = clusterId;
        }
        
        if (labels[neighborIdx] == 0) {
            // Unprocessed point
            labels[neighborIdx] = clusterId;
            
            std::vector<size_t> neighborNeighbors = findNeighbors(detections, neighborIdx);
            
            if (static_cast<int>(neighborNeighbors.size()) >= minPoints_) {
                // Core point, add its neighbors
                for (size_t nn : neighborNeighbors) {
                    if (std::find(neighbors.begin(), neighbors.end(), nn) == neighbors.end()) {
                        neighbors.push_back(nn);
                    }
                }
            }
        }
        
        ++i;
    }
}

std::vector<Cluster> DBSCANClustering::cluster(const std::vector<Detection>& detections) {
    std::vector<Cluster> clusters;
    
    if (detections.empty()) {
        return clusters;
    }
    
    size_t n = detections.size();
    
    // Precompute distance matrix for small datasets
    if (n <= 500) {
        distanceMatrix_.resize(n, std::vector<double>(n, 0.0));
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = i + 1; j < n; ++j) {
                double dist = computeDistance(detections[i], detections[j]);
                distanceMatrix_[i][j] = dist;
                distanceMatrix_[j][i] = dist;
            }
        }
        useDistanceMatrix_ = true;
    } else {
        useDistanceMatrix_ = false;
    }
    
    // Labels: 0 = unprocessed, -1 = noise, >0 = cluster ID
    std::vector<int> labels(n, 0);
    int clusterId = 0;
    
    for (size_t i = 0; i < n; ++i) {
        if (labels[i] != 0) {
            continue;  // Already processed
        }
        
        std::vector<size_t> neighbors = findNeighbors(detections, i);
        
        if (static_cast<int>(neighbors.size()) < minPoints_) {
            // Mark as noise (might be claimed by another cluster later)
            labels[i] = -1;
        } else {
            // Start new cluster
            ++clusterId;
            expandCluster(detections, i, neighbors, clusterId, labels);
        }
    }
    
    // Create cluster objects
    clusters.reserve(clusterId);
    for (int c = 1; c <= clusterId; ++c) {
        Cluster cluster;
        cluster.id = c - 1;  // Zero-indexed
        
        for (size_t i = 0; i < n; ++i) {
            if (labels[i] == c) {
                cluster.detectionIndices.push_back(i);
            }
        }
        
        if (!cluster.empty()) {
            cluster.computeCentroid(detections);
            cluster.computeExtent(detections);
            clusters.push_back(std::move(cluster));
        }
    }
    
    // Clear distance matrix
    distanceMatrix_.clear();
    
    return clusters;
}

} // namespace zoppler
