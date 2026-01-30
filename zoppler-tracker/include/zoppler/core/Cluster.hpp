/**
 * @file Cluster.hpp
 * @brief Cluster structure for grouped detections
 * @copyright Defense-grade radar tracking system
 */

#ifndef ZOPPLER_CORE_CLUSTER_HPP
#define ZOPPLER_CORE_CLUSTER_HPP

#include "Types.hpp"
#include "Detection.hpp"
#include <vector>
#include <numeric>
#include <algorithm>

namespace zoppler {

/**
 * @brief Represents a cluster of radar detections
 * 
 * Groups related detections that likely originate from the same target.
 */
struct Cluster {
    // Unique cluster identifier
    int id = -1;
    
    // Detection indices belonging to this cluster
    std::vector<size_t> detectionIndices;
    
    // Centroid position (Cartesian)
    double centroidX = INVALID_VALUE;
    double centroidY = INVALID_VALUE;
    double centroidZ = INVALID_VALUE;
    
    // Centroid position (Polar)
    double centroidRange = INVALID_VALUE;
    double centroidAzimuth = INVALID_VALUE;
    double centroidElevation = INVALID_VALUE;
    
    // Average Doppler velocity
    double meanDoppler = INVALID_VALUE;
    
    // Cluster extent (bounding box)
    double extentX = 0.0;
    double extentY = 0.0;
    double extentZ = 0.0;
    
    // Cluster statistics
    double meanSNR = INVALID_VALUE;
    double maxSNR = INVALID_VALUE;
    
    // Timestamp (from most recent detection)
    Timestamp timestamp = 0;
    
    // Number of detections in cluster
    size_t size() const { return detectionIndices.size(); }
    
    // Check if cluster is empty
    bool empty() const { return detectionIndices.empty(); }
    
    /**
     * @brief Compute centroid from detections
     */
    void computeCentroid(const std::vector<Detection>& detections) {
        if (detectionIndices.empty()) return;
        
        double sumX = 0, sumY = 0, sumZ = 0;
        double sumR = 0, sumAz = 0, sumEl = 0;
        double sumDop = 0;
        double sumSNR = 0;
        int countZ = 0, countDop = 0, countSNR = 0;
        
        maxSNR = -std::numeric_limits<double>::infinity();
        
        for (size_t idx : detectionIndices) {
            if (idx >= detections.size()) continue;
            const auto& det = detections[idx];
            
            if (det.hasValidCartesian()) {
                sumX += det.x;
                sumY += det.y;
                if (isValid(det.z)) {
                    sumZ += det.z;
                    countZ++;
                }
            }
            
            if (det.hasValidPolar()) {
                sumR += det.range;
                sumAz += det.azimuth;
                if (isValid(det.elevation)) {
                    sumEl += det.elevation;
                }
            }
            
            if (det.hasDoppler()) {
                sumDop += det.doppler;
                countDop++;
            }
            
            if (isValid(det.snr)) {
                sumSNR += det.snr;
                maxSNR = std::max(maxSNR, det.snr);
                countSNR++;
            }
            
            timestamp = std::max(timestamp, det.timestamp);
        }
        
        size_t n = detectionIndices.size();
        centroidX = sumX / n;
        centroidY = sumY / n;
        centroidZ = (countZ > 0) ? sumZ / countZ : INVALID_VALUE;
        
        centroidRange = sumR / n;
        centroidAzimuth = sumAz / n;
        centroidElevation = (countZ > 0) ? sumEl / countZ : INVALID_VALUE;
        
        meanDoppler = (countDop > 0) ? sumDop / countDop : INVALID_VALUE;
        meanSNR = (countSNR > 0) ? sumSNR / countSNR : INVALID_VALUE;
        
        if (countSNR == 0) maxSNR = INVALID_VALUE;
    }
    
    /**
     * @brief Compute cluster extent (bounding box)
     */
    void computeExtent(const std::vector<Detection>& detections) {
        if (detectionIndices.empty()) return;
        
        double minX = std::numeric_limits<double>::max();
        double maxX = std::numeric_limits<double>::lowest();
        double minY = std::numeric_limits<double>::max();
        double maxY = std::numeric_limits<double>::lowest();
        double minZ = std::numeric_limits<double>::max();
        double maxZ = std::numeric_limits<double>::lowest();
        
        for (size_t idx : detectionIndices) {
            if (idx >= detections.size()) continue;
            const auto& det = detections[idx];
            
            if (det.hasValidCartesian()) {
                minX = std::min(minX, det.x);
                maxX = std::max(maxX, det.x);
                minY = std::min(minY, det.y);
                maxY = std::max(maxY, det.y);
                if (isValid(det.z)) {
                    minZ = std::min(minZ, det.z);
                    maxZ = std::max(maxZ, det.z);
                }
            }
        }
        
        extentX = maxX - minX;
        extentY = maxY - minY;
        extentZ = (minZ < maxZ) ? maxZ - minZ : 0.0;
    }
    
    /**
     * @brief Get centroid as Detection
     */
    Detection getCentroidDetection() const {
        Detection det;
        det.x = centroidX;
        det.y = centroidY;
        det.z = centroidZ;
        det.range = centroidRange;
        det.azimuth = centroidAzimuth;
        det.elevation = centroidElevation;
        det.doppler = meanDoppler;
        det.timestamp = timestamp;
        det.snr = meanSNR;
        det.clusterId = id;
        return det;
    }
    
    /**
     * @brief Euclidean distance to another cluster
     */
    double distanceTo(const Cluster& other) const {
        double dx = centroidX - other.centroidX;
        double dy = centroidY - other.centroidY;
        double dz = (isValid(centroidZ) && isValid(other.centroidZ)) 
                    ? (centroidZ - other.centroidZ) : 0.0;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }
};

} // namespace zoppler

#endif // ZOPPLER_CORE_CLUSTER_HPP
