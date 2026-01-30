/**
 * @file JsonLoader.hpp
 * @brief JSON configuration loader
 * @copyright Defense-grade radar tracking system
 */

#ifndef ZOPPLER_CONFIG_JSONLOADER_HPP
#define ZOPPLER_CONFIG_JSONLOADER_HPP

#include "../core/TrackerConfig.hpp"
#include <string>

namespace zoppler {

/**
 * @brief JSON configuration file loader
 * 
 * Loads TrackerConfig from JSON files or strings.
 * Uses a simple built-in JSON parser (no external dependencies).
 */
class JsonLoader {
public:
    /**
     * @brief Load configuration from file
     * 
     * @param filepath Path to JSON file
     * @return Parsed TrackerConfig
     * @throws std::runtime_error on parse error
     */
    static TrackerConfig loadFromFile(const std::string& filepath);
    
    /**
     * @brief Load configuration from JSON string
     * 
     * @param jsonString JSON configuration string
     * @return Parsed TrackerConfig
     * @throws std::runtime_error on parse error
     */
    static TrackerConfig loadFromString(const std::string& jsonString);
    
private:
    JsonLoader() = delete;
};

} // namespace zoppler

#endif // ZOPPLER_CONFIG_JSONLOADER_HPP
