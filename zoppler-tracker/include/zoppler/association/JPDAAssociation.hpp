/**
 * @file JPDAAssociation.hpp
 * @brief Joint Probabilistic Data Association
 * @copyright Defense-grade radar tracking system
 */

#ifndef ZOPPLER_ASSOCIATION_JPDA_HPP
#define ZOPPLER_ASSOCIATION_JPDA_HPP

#include "IDataAssociation.hpp"

namespace zoppler {

/**
 * @brief Joint Probabilistic Data Association (JPDA)
 * 
 * Computes association probabilities for all track-detection pairs.
 * Handles measurement origin uncertainty probabilistically.
 * Allows soft data association for closely-spaced targets.
 * 
 * Best for: Moderate clutter, crossing targets, closely-spaced targets
 */
class JPDAAssociation : public IDataAssociation {
public:
    /**
     * @brief Construct JPDA with parameters
     * 
     * @param pD Probability of detection
     * @param clutterRate Spatial clutter density
     * @param gateThreshold Chi-squared gate threshold
     */
    JPDAAssociation(double pD = 0.9, double clutterRate = 1e-6,
                    double gateThreshold = 9.21);
    
    /**
     * @brief Construct from config
     */
    explicit JPDAAssociation(const TrackerConfig& config);
    
    /**
     * @brief Destructor
     */
    ~JPDAAssociation() override = default;
    
    /**
     * @brief Associate using JPDA algorithm
     */
    AssociationResult associate(
        const std::vector<Track>& tracks,
        const std::vector<Detection>& detections) override;
    
    /**
     * @brief Get algorithm name
     */
    std::string getName() const override { return "JPDA"; }
    
    /**
     * @brief Clone the algorithm
     */
    std::unique_ptr<IDataAssociation> clone() const override;
    
    // Configuration
    void setPD(double pD) { pD_ = pD; }
    void setClutterRate(double rate) { clutterRate_ = rate; }
    void setMaxHypotheses(int max) { maxHypotheses_ = max; }
    
    /**
     * @brief Get association probabilities (JPDA-specific)
     * 
     * Returns matrix where [i][j] is probability that track i
     * originated from detection j. [i][0] is probability of
     * no detection (missed).
     */
    const std::vector<std::vector<double>>& getAssociationProbabilities() const {
        return associationProbabilities_;
    }
    
private:
    /**
     * @brief Compute likelihood that detection came from track
     */
    double computeLikelihood(const Track& track, const Detection& detection) const;
    
    /**
     * @brief Enumerate valid joint events
     */
    void enumerateJointEvents(
        const std::vector<std::vector<bool>>& validAssignments,
        std::vector<std::vector<int>>& jointEvents) const;
    
    /**
     * @brief Compute joint event probability
     */
    double computeJointEventProbability(
        const std::vector<Track>& tracks,
        const std::vector<Detection>& detections,
        const std::vector<int>& event,
        const std::vector<std::vector<double>>& likelihoods) const;
    
    double pD_;             // Probability of detection
    double clutterRate_;    // Clutter spatial density (per unit volume)
    int maxHypotheses_;     // Maximum hypotheses to consider
    
    // Cached results
    mutable std::vector<std::vector<double>> associationProbabilities_;
};

} // namespace zoppler

#endif // ZOPPLER_ASSOCIATION_JPDA_HPP
