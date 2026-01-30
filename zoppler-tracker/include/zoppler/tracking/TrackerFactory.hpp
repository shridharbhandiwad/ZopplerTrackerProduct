/**
 * @file TrackerFactory.hpp
 * @brief Factory for creating tracker models
 * @copyright Defense-grade radar tracking system
 */

#ifndef ZOPPLER_TRACKING_FACTORY_HPP
#define ZOPPLER_TRACKING_FACTORY_HPP

#include "ITrackerModel.hpp"
#include "../core/TrackerConfig.hpp"
#include <memory>
#include <string>

namespace zoppler {

/**
 * @brief Factory for creating tracker model instances
 */
class TrackerFactory {
public:
    /**
     * @brief Create tracker from configuration
     * 
     * @param config Tracker configuration
     * @return Unique pointer to tracker implementation
     */
    static std::unique_ptr<ITrackerModel> create(const TrackerConfig& config);
    
    /**
     * @brief Create tracker by name
     * 
     * @param algorithmName Algorithm name
     * @param motionModel Motion model (for EKF-based trackers)
     * @return Unique pointer to tracker implementation with defaults
     */
    static std::unique_ptr<ITrackerModel> create(const std::string& algorithmName,
                                                  const std::string& motionModel = "CV");
    
    /**
     * @brief Check if algorithm name is valid
     */
    static bool isValidAlgorithm(const std::string& algorithmName);
    
    /**
     * @brief Get list of available algorithms
     */
    static std::vector<std::string> getAvailableAlgorithms();
    
    /**
     * @brief Get list of available motion models
     */
    static std::vector<std::string> getAvailableMotionModels();
    
private:
    TrackerFactory() = delete;  // Static factory, no instantiation
};

} // namespace zoppler

#endif // ZOPPLER_TRACKING_FACTORY_HPP
