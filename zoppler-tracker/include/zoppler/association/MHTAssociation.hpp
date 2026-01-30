/**
 * @file MHTAssociation.hpp
 * @brief Multiple Hypothesis Tracking data association
 * @copyright Defense-grade radar tracking system
 */

#ifndef ZOPPLER_ASSOCIATION_MHT_HPP
#define ZOPPLER_ASSOCIATION_MHT_HPP

#include "IDataAssociation.hpp"
#include <list>
#include <map>

namespace zoppler {

/**
 * @brief Hypothesis for MHT
 */
struct MHTHypothesis {
    int id;
    double probability;
    std::vector<int> trackToDetection;  // Track index -> Detection index (-1 for missed)
    std::vector<int> newTrackDetections; // Detections that start new tracks
    
    bool operator<(const MHTHypothesis& other) const {
        return probability > other.probability;  // Higher probability first
    }
};

/**
 * @brief Multiple Hypothesis Tracking (MHT) Association
 * 
 * Maintains multiple hypotheses about track-detection associations.
 * Defers hard decisions and uses N-scan pruning for tractability.
 * 
 * Best for: High clutter, crossing targets, track initiation/termination
 */
class MHTAssociation : public IDataAssociation {
public:
    /**
     * @brief Construct MHT with parameters
     * 
     * @param nScan N-scan pruning depth
     * @param maxHypotheses Maximum number of hypotheses
     * @param pruneThreshold Probability threshold for pruning
     */
    MHTAssociation(int nScan = 3, int maxHypotheses = 1000,
                   double pruneThreshold = 0.01);
    
    /**
     * @brief Construct from config
     */
    explicit MHTAssociation(const TrackerConfig& config);
    
    /**
     * @brief Destructor
     */
    ~MHTAssociation() override = default;
    
    /**
     * @brief Associate using MHT algorithm
     */
    AssociationResult associate(
        const std::vector<Track>& tracks,
        const std::vector<Detection>& detections) override;
    
    /**
     * @brief Get algorithm name
     */
    std::string getName() const override { return "MHT"; }
    
    /**
     * @brief Clone the algorithm
     */
    std::unique_ptr<IDataAssociation> clone() const override;
    
    /**
     * @brief Reset hypothesis tree
     */
    void reset();
    
    // Configuration
    void setNScan(int n) { nScan_ = n; }
    void setMaxHypotheses(int max) { maxHypotheses_ = max; }
    void setPruneThreshold(double thresh) { pruneThreshold_ = thresh; }
    void setPD(double pD) { pD_ = pD; }
    void setClutterRate(double rate) { clutterRate_ = rate; }
    void setNewTrackProbability(double p) { newTrackProb_ = p; }
    
    /**
     * @brief Get current hypotheses (for debugging)
     */
    const std::list<MHTHypothesis>& getHypotheses() const { return hypotheses_; }
    
private:
    /**
     * @brief Generate child hypotheses from current hypotheses
     */
    void generateHypotheses(
        const std::vector<Track>& tracks,
        const std::vector<Detection>& detections);
    
    /**
     * @brief Prune low-probability hypotheses
     */
    void pruneHypotheses();
    
    /**
     * @brief Normalize hypothesis probabilities
     */
    void normalizeHypotheses();
    
    /**
     * @brief Apply N-scan pruning
     */
    void nScanPrune();
    
    /**
     * @brief Compute likelihood for hypothesis
     */
    double computeHypothesisLikelihood(
        const std::vector<Track>& tracks,
        const std::vector<Detection>& detections,
        const MHTHypothesis& hypothesis) const;
    
    /**
     * @brief Get best hypothesis
     */
    const MHTHypothesis& getBestHypothesis() const;
    
    int nScan_;                 // N-scan pruning depth
    int maxHypotheses_;         // Maximum hypotheses to maintain
    double pruneThreshold_;     // Minimum hypothesis probability
    double pD_;                 // Detection probability
    double clutterRate_;        // Clutter density
    double newTrackProb_;       // Prior probability for new track
    
    std::list<MHTHypothesis> hypotheses_;
    int nextHypothesisId_ = 0;
    int scanCount_ = 0;
    
    // History for N-scan pruning
    std::vector<std::list<MHTHypothesis>> hypothesisHistory_;
};

} // namespace zoppler

#endif // ZOPPLER_ASSOCIATION_MHT_HPP
