/**
 * @file AssociationFactory.hpp
 * @brief Factory for creating data association algorithms
 * @copyright Defense-grade radar tracking system
 */

#ifndef ZOPPLER_ASSOCIATION_FACTORY_HPP
#define ZOPPLER_ASSOCIATION_FACTORY_HPP

#include "IDataAssociation.hpp"
#include "../core/TrackerConfig.hpp"
#include <memory>
#include <string>

namespace zoppler {

/**
 * @brief Factory for creating data association algorithm instances
 */
class AssociationFactory {
public:
    /**
     * @brief Create association algorithm from configuration
     * 
     * @param config Tracker configuration
     * @return Unique pointer to association implementation
     */
    static std::unique_ptr<IDataAssociation> create(const TrackerConfig& config);
    
    /**
     * @brief Create association algorithm by name
     * 
     * @param algorithmName Algorithm name (GNN, JPDA, MHT)
     * @return Unique pointer to association implementation with defaults
     */
    static std::unique_ptr<IDataAssociation> create(const std::string& algorithmName);
    
    /**
     * @brief Check if algorithm name is valid
     */
    static bool isValidAlgorithm(const std::string& algorithmName);
    
    /**
     * @brief Get list of available algorithms
     */
    static std::vector<std::string> getAvailableAlgorithms();
    
private:
    AssociationFactory() = delete;  // Static factory, no instantiation
};

} // namespace zoppler

#endif // ZOPPLER_ASSOCIATION_FACTORY_HPP
