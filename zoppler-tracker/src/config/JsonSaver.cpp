/**
 * @file JsonSaver.cpp
 * @brief JSON saver implementation
 * @copyright Defense-grade radar tracking system
 */

#include "zoppler/config/JsonSaver.hpp"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <stdexcept>

namespace zoppler {

std::string JsonSaver::escapeString(const std::string& str) {
    std::string result;
    result.reserve(str.size());
    
    for (char c : str) {
        switch (c) {
            case '"': result += "\\\""; break;
            case '\\': result += "\\\\"; break;
            case '\n': result += "\\n"; break;
            case '\r': result += "\\r"; break;
            case '\t': result += "\\t"; break;
            default: result += c; break;
        }
    }
    
    return result;
}

std::string JsonSaver::indent(int level) {
    return std::string(level * 2, ' ');
}

std::string JsonSaver::saveToString(const TrackerConfig& config, bool pretty) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(6);
    
    std::string nl = pretty ? "\n" : "";
    auto ind = [&](int level) { return pretty ? indent(level) : ""; };
    
    ss << "{" << nl;
    
    // Radar configuration
    ss << ind(1) << "\"radar\": {" << nl;
    ss << ind(2) << "\"is3D\": " << (config.radar.is3D ? "true" : "false") << "," << nl;
    ss << ind(2) << "\"hasDoppler\": " << (config.radar.hasDoppler ? "true" : "false") << "," << nl;
    ss << ind(2) << "\"updateTime\": " << config.radar.updateTime << "," << nl;
    ss << ind(2) << "\"accuracy\": {" << nl;
    ss << ind(3) << "\"range\": " << config.radar.rangeStd << "," << nl;
    ss << ind(3) << "\"azimuth\": " << config.radar.azimuthStd << "," << nl;
    ss << ind(3) << "\"elevation\": " << config.radar.elevationStd << "," << nl;
    ss << ind(3) << "\"doppler\": " << config.radar.dopplerStd << nl;
    ss << ind(2) << "}," << nl;
    ss << ind(2) << "\"coverage\": {" << nl;
    ss << ind(3) << "\"maxRange\": " << config.radar.maxRange << "," << nl;
    ss << ind(3) << "\"minRange\": " << config.radar.minRange << nl;
    ss << ind(2) << "}," << nl;
    ss << ind(2) << "\"pD\": " << config.radar.pD << "," << nl;
    ss << ind(2) << "\"radarType\": \"" << escapeString(config.radar.radarType) << "\"" << nl;
    ss << ind(1) << "}," << nl;
    
    // Target type
    ss << ind(1) << "\"targetType\": \"" << escapeString(config.targetType) << "\"," << nl;
    
    // Clustering
    ss << ind(1) << "\"clustering\": {" << nl;
    ss << ind(2) << "\"algorithm\": \"" << escapeString(config.clusteringAlgo) << "\"," << nl;
    ss << ind(2) << "\"epsilon\": " << config.clusteringParams.epsilon << "," << nl;
    ss << ind(2) << "\"minPoints\": " << config.clusteringParams.minPoints << nl;
    ss << ind(1) << "}," << nl;
    
    // Association
    ss << ind(1) << "\"association\": {" << nl;
    ss << ind(2) << "\"algorithm\": \"" << escapeString(config.associationAlgo) << "\"," << nl;
    ss << ind(2) << "\"maxDistance\": " << config.associationParams.maxAssociationDistance << nl;
    ss << ind(1) << "}," << nl;
    
    // Tracking
    ss << ind(1) << "\"tracking\": {" << nl;
    ss << ind(2) << "\"algorithm\": \"" << escapeString(config.trackingAlgo) << "\"," << nl;
    ss << ind(2) << "\"motionModel\": \"" << escapeString(config.motionModel) << "\"," << nl;
    ss << ind(2) << "\"profile\": \"" << escapeString(config.profile) << "\"," << nl;
    
    // IMM models if applicable
    if (!config.trackingParams.immModels.empty()) {
        ss << ind(2) << "\"immModels\": [";
        for (size_t i = 0; i < config.trackingParams.immModels.size(); ++i) {
            if (i > 0) ss << ", ";
            ss << "\"" << escapeString(config.trackingParams.immModels[i]) << "\"";
        }
        ss << "]," << nl;
    }
    
    ss << ind(2) << "\"ukfAlpha\": " << config.trackingParams.ukfAlpha << "," << nl;
    ss << ind(2) << "\"ukfBeta\": " << config.trackingParams.ukfBeta << "," << nl;
    ss << ind(2) << "\"numParticles\": " << config.trackingParams.pfNumParticles << nl;
    ss << ind(1) << "}," << nl;
    
    // Track management
    ss << ind(1) << "\"trackManagement\": {" << nl;
    ss << ind(2) << "\"minHits\": " << config.trackManagement.minHitsToConfirm << "," << nl;
    ss << ind(2) << "\"confirmWindow\": " << config.trackManagement.confirmWindow << "," << nl;
    ss << ind(2) << "\"maxMisses\": " << config.trackManagement.maxConsecutiveMisses << "," << nl;
    ss << ind(2) << "\"maxTracks\": " << config.trackManagement.maxTracks << nl;
    ss << ind(1) << "}," << nl;
    
    // Profile parameters
    ss << ind(1) << "\"profileParams\": {" << nl;
    ss << ind(2) << "\"qPos\": " << config.trackerProfile.qPos << "," << nl;
    ss << ind(2) << "\"qVel\": " << config.trackerProfile.qVel << "," << nl;
    ss << ind(2) << "\"qAcc\": " << config.trackerProfile.qAcc << "," << nl;
    ss << ind(2) << "\"gateChiSq\": " << config.trackerProfile.gateChiSq << "," << nl;
    ss << ind(2) << "\"confirmHits\": " << config.trackerProfile.confirmHits << "," << nl;
    ss << ind(2) << "\"maxMisses\": " << config.trackerProfile.maxMisses << nl;
    ss << ind(1) << "}" << nl;
    
    ss << "}";
    
    return ss.str();
}

void JsonSaver::saveToFile(const TrackerConfig& config, const std::string& filepath) {
    std::ofstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file for writing: " + filepath);
    }
    
    file << saveToString(config, true);
    
    if (!file.good()) {
        throw std::runtime_error("Error writing to file: " + filepath);
    }
}

} // namespace zoppler
