/**
 * @file JsonSaver.hpp
 * @brief JSON configuration saver
 * @copyright Defense-grade radar tracking system
 */

#ifndef ZOPPLER_CONFIG_JSONSAVER_HPP
#define ZOPPLER_CONFIG_JSONSAVER_HPP

#include "../core/TrackerConfig.hpp"
#include <string>

namespace zoppler {

/**
 * @brief JSON configuration file saver
 * 
 * Saves TrackerConfig to JSON files or strings.
 */
class JsonSaver {
public:
    /**
     * @brief Save configuration to file
     * 
     * @param config Configuration to save
     * @param filepath Output file path
     * @throws std::runtime_error on write error
     */
    static void saveToFile(const TrackerConfig& config, const std::string& filepath);
    
    /**
     * @brief Save configuration to JSON string
     * 
     * @param config Configuration to save
     * @param pretty If true, format with indentation
     * @return JSON string
     */
    static std::string saveToString(const TrackerConfig& config, bool pretty = true);
    
private:
    JsonSaver() = delete;
    
    static std::string escapeString(const std::string& str);
    static std::string indent(int level);
};

} // namespace zoppler

#endif // ZOPPLER_CONFIG_JSONSAVER_HPP
