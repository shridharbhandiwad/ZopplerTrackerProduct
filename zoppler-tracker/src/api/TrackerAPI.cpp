/**
 * @file TrackerAPI.cpp
 * @brief Qt-friendly API implementation
 * @copyright Defense-grade radar tracking system
 */

#include "zoppler/api/TrackerAPI.hpp"
#include "zoppler/config/JsonLoader.hpp"
#include "zoppler/config/JsonSaver.hpp"
#include <mutex>

namespace zoppler {

/**
 * @brief Private implementation (PIMPL idiom for ABI stability)
 */
class TrackerAPI::Impl {
public:
    std::unique_ptr<TrackerEngine> engine;
    TrackerConfig config;
    mutable std::mutex mutex;
    bool initialized = false;
    
    void ensureInitialized() {
        if (!initialized && engine) {
            initialized = true;
        }
    }
};

TrackerAPI::TrackerAPI()
    : pImpl_(std::make_unique<Impl>()) {
}

TrackerAPI::TrackerAPI(const TrackerConfig& config)
    : pImpl_(std::make_unique<Impl>()) {
    initialize(config);
}

TrackerAPI::~TrackerAPI() = default;

TrackerAPI::TrackerAPI(TrackerAPI&&) noexcept = default;
TrackerAPI& TrackerAPI::operator=(TrackerAPI&&) noexcept = default;

bool TrackerAPI::initialize(const TrackerConfig& config) {
    std::lock_guard<std::mutex> lock(pImpl_->mutex);
    
    try {
        pImpl_->config = config;
        pImpl_->config.resolveProfile();
        pImpl_->engine = std::make_unique<TrackerEngine>(pImpl_->config);
        pImpl_->initialized = true;
        return true;
    } catch (const std::exception&) {
        pImpl_->initialized = false;
        return false;
    }
}

bool TrackerAPI::isInitialized() const {
    std::lock_guard<std::mutex> lock(pImpl_->mutex);
    return pImpl_->initialized && pImpl_->engine != nullptr;
}

void TrackerAPI::processDetections(const std::vector<Detection>& detections, int64_t timestamp) {
    std::lock_guard<std::mutex> lock(pImpl_->mutex);
    
    if (!pImpl_->engine) return;
    
    pImpl_->engine->processDetections(detections, static_cast<Timestamp>(timestamp));
}

std::vector<TrackState> TrackerAPI::getTracks() const {
    std::lock_guard<std::mutex> lock(pImpl_->mutex);
    
    if (!pImpl_->engine) return {};
    
    return pImpl_->engine->getTrackStates();
}

std::vector<TrackState> TrackerAPI::getConfirmedTracks() const {
    std::lock_guard<std::mutex> lock(pImpl_->mutex);
    
    if (!pImpl_->engine) return {};
    
    std::vector<TrackState> states;
    for (const Track* track : pImpl_->engine->getConfirmedTracks()) {
        if (track && track->model) {
            states.push_back(track->model->getState());
        }
    }
    return states;
}

std::vector<int> TrackerAPI::getTrackIds() const {
    std::lock_guard<std::mutex> lock(pImpl_->mutex);
    
    if (!pImpl_->engine) return {};
    
    std::vector<int> ids;
    for (const auto& track : pImpl_->engine->getTracks()) {
        if (track.isActive()) {
            ids.push_back(track.id);
        }
    }
    return ids;
}

bool TrackerAPI::getTrackById(int trackId, TrackState& state) const {
    std::lock_guard<std::mutex> lock(pImpl_->mutex);
    
    if (!pImpl_->engine) return false;
    
    const Track* track = pImpl_->engine->getTrackById(trackId);
    if (track && track->model) {
        state = track->model->getState();
        return true;
    }
    return false;
}

int TrackerAPI::getNumTracks() const {
    std::lock_guard<std::mutex> lock(pImpl_->mutex);
    
    if (!pImpl_->engine) return 0;
    
    return static_cast<int>(pImpl_->engine->getNumTracks());
}

int TrackerAPI::getNumConfirmedTracks() const {
    std::lock_guard<std::mutex> lock(pImpl_->mutex);
    
    if (!pImpl_->engine) return 0;
    
    return static_cast<int>(pImpl_->engine->getNumConfirmedTracks());
}

void TrackerAPI::reset() {
    std::lock_guard<std::mutex> lock(pImpl_->mutex);
    
    if (pImpl_->engine) {
        pImpl_->engine->reset();
    }
}

TrackerConfig TrackerAPI::getConfig() const {
    std::lock_guard<std::mutex> lock(pImpl_->mutex);
    return pImpl_->config;
}

void TrackerAPI::setConfig(const TrackerConfig& config) {
    initialize(config);
}

void TrackerAPI::setClusteringAlgorithm(const std::string& algorithm) {
    std::lock_guard<std::mutex> lock(pImpl_->mutex);
    
    pImpl_->config.clusteringAlgo = algorithm;
    if (pImpl_->engine) {
        pImpl_->engine->reconfigure(pImpl_->config);
    }
}

void TrackerAPI::setAssociationAlgorithm(const std::string& algorithm) {
    std::lock_guard<std::mutex> lock(pImpl_->mutex);
    
    pImpl_->config.associationAlgo = algorithm;
    if (pImpl_->engine) {
        pImpl_->engine->reconfigure(pImpl_->config);
    }
}

void TrackerAPI::setTrackingAlgorithm(const std::string& algorithm) {
    std::lock_guard<std::mutex> lock(pImpl_->mutex);
    
    pImpl_->config.trackingAlgo = algorithm;
    if (pImpl_->engine) {
        pImpl_->engine->reconfigure(pImpl_->config);
    }
}

void TrackerAPI::setMotionModel(const std::string& model) {
    std::lock_guard<std::mutex> lock(pImpl_->mutex);
    
    pImpl_->config.motionModel = model;
    if (pImpl_->engine) {
        pImpl_->engine->reconfigure(pImpl_->config);
    }
}

void TrackerAPI::setProfile(const std::string& profile) {
    std::lock_guard<std::mutex> lock(pImpl_->mutex);
    
    pImpl_->config.profile = profile;
    pImpl_->config.resolveProfile();
    if (pImpl_->engine) {
        pImpl_->engine->reconfigure(pImpl_->config);
    }
}

TrackerAPI::Statistics TrackerAPI::getStatistics() const {
    std::lock_guard<std::mutex> lock(pImpl_->mutex);
    
    Statistics stats = {};
    
    if (pImpl_->engine) {
        const auto& engineStats = pImpl_->engine->getStatistics();
        stats.totalScans = engineStats.totalScans;
        stats.totalDetections = engineStats.totalDetections;
        stats.tracksInitiated = engineStats.tracksInitiated;
        stats.tracksDeleted = engineStats.tracksDeleted;
        stats.lastProcessingTimeMs = engineStats.lastProcessingTimeMs;
        stats.lastTimestamp = static_cast<int64_t>(engineStats.lastTimestamp);
    }
    
    return stats;
}

bool TrackerAPI::loadConfigFromFile(const std::string& filepath) {
    try {
        TrackerConfig config = JsonLoader::loadFromFile(filepath);
        return initialize(config);
    } catch (const std::exception&) {
        return false;
    }
}

bool TrackerAPI::saveConfigToFile(const std::string& filepath) const {
    std::lock_guard<std::mutex> lock(pImpl_->mutex);
    
    try {
        JsonSaver::saveToFile(pImpl_->config, filepath);
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

bool TrackerAPI::loadConfigFromString(const std::string& jsonString) {
    try {
        TrackerConfig config = JsonLoader::loadFromString(jsonString);
        return initialize(config);
    } catch (const std::exception&) {
        return false;
    }
}

std::string TrackerAPI::getConfigAsJson() const {
    std::lock_guard<std::mutex> lock(pImpl_->mutex);
    return JsonSaver::saveToString(pImpl_->config);
}

} // namespace zoppler
