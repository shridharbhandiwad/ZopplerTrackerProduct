/**
 * @file AssociationFactory.cpp
 * @brief Association factory implementation
 * @copyright Defense-grade radar tracking system
 */

#include "zoppler/association/AssociationFactory.hpp"
#include "zoppler/association/GNNAssociation.hpp"
#include "zoppler/association/JPDAAssociation.hpp"
#include "zoppler/association/MHTAssociation.hpp"
#include <algorithm>
#include <stdexcept>

namespace zoppler {

std::unique_ptr<IDataAssociation> AssociationFactory::create(const TrackerConfig& config) {
    AssociationAlgorithm algo = config.getAssociationAlgorithm();
    
    switch (algo) {
        case AssociationAlgorithm::GNN:
            return std::make_unique<GNNAssociation>(config);
            
        case AssociationAlgorithm::JPDA:
            return std::make_unique<JPDAAssociation>(config);
            
        case AssociationAlgorithm::MHT:
            return std::make_unique<MHTAssociation>(config);
            
        default:
            return std::make_unique<GNNAssociation>(config);
    }
}

std::unique_ptr<IDataAssociation> AssociationFactory::create(const std::string& algorithmName) {
    std::string name = algorithmName;
    std::transform(name.begin(), name.end(), name.begin(), ::toupper);
    
    if (name == "GNN" || name == "GLOBAL_NEAREST_NEIGHBOR") {
        return std::make_unique<GNNAssociation>();
    } else if (name == "JPDA" || name == "JOINT_PROBABILISTIC") {
        return std::make_unique<JPDAAssociation>();
    } else if (name == "MHT" || name == "MULTIPLE_HYPOTHESIS") {
        return std::make_unique<MHTAssociation>();
    }
    
    throw std::invalid_argument("Unknown association algorithm: " + algorithmName);
}

bool AssociationFactory::isValidAlgorithm(const std::string& algorithmName) {
    std::string name = algorithmName;
    std::transform(name.begin(), name.end(), name.begin(), ::toupper);
    
    return name == "GNN" || name == "GLOBAL_NEAREST_NEIGHBOR" ||
           name == "JPDA" || name == "JOINT_PROBABILISTIC" ||
           name == "MHT" || name == "MULTIPLE_HYPOTHESIS";
}

std::vector<std::string> AssociationFactory::getAvailableAlgorithms() {
    return {"GNN", "JPDA", "MHT"};
}

} // namespace zoppler
