/**
 * @file Zoppler.hpp
 * @brief Main include file for Zoppler Tracker
 * @copyright Defense-grade radar tracking system
 * 
 * Include this single header to access all Zoppler Tracker functionality.
 */

#ifndef ZOPPLER_ZOPPLER_HPP
#define ZOPPLER_ZOPPLER_HPP

// Core types
#include "core/Types.hpp"
#include "core/Detection.hpp"
#include "core/Measurement.hpp"
#include "core/Cluster.hpp"
#include "core/TrackState.hpp"
#include "core/Track.hpp"
#include "core/RadarConfig.hpp"
#include "core/TrackerConfig.hpp"
#include "core/TrackerProfile.hpp"

// Utilities
#include "utils/Matrix.hpp"
#include "utils/MathUtils.hpp"

// Clustering
#include "clustering/IClustering.hpp"
#include "clustering/DBSCANClustering.hpp"
#include "clustering/ContinuousRangeClustering.hpp"
#include "clustering/ClusteringFactory.hpp"

// Data Association
#include "association/IDataAssociation.hpp"
#include "association/GNNAssociation.hpp"
#include "association/JPDAAssociation.hpp"
#include "association/MHTAssociation.hpp"
#include "association/AssociationFactory.hpp"

// Tracking / Filtering
#include "tracking/ITrackerModel.hpp"
#include "tracking/EKFTracker.hpp"
#include "tracking/UKFTracker.hpp"
#include "tracking/IMMTracker.hpp"
#include "tracking/ParticleFilterTracker.hpp"
#include "tracking/TrackerFactory.hpp"

// Engine
#include "engine/TrackManager.hpp"
#include "engine/TrackerEngine.hpp"

// Public API
#include "api/TrackerAPI.hpp"

// Configuration
#include "config/JsonLoader.hpp"
#include "config/JsonSaver.hpp"

/**
 * @namespace zoppler
 * @brief Zoppler Tracker - Defense-grade multi-target tracking system
 * 
 * The Zoppler namespace contains all components for the modular
 * multi-target tracker:
 * 
 * - **Core Types**: Detection, Track, Cluster, Measurement, Configuration
 * - **Clustering**: DBSCAN, Continuous Range
 * - **Association**: GNN, JPDA, MHT
 * - **Tracking**: EKF, UKF, IMM, Particle Filter with CV/CA/CT models
 * - **Engine**: TrackerEngine orchestrates the complete pipeline
 * - **API**: Thread-safe Qt-friendly TrackerAPI
 * 
 * @example
 * @code
 * #include <zoppler/Zoppler.hpp>
 * 
 * // Create tracker with preset
 * zoppler::TrackerConfig config = zoppler::TrackerConfig::shortRangeCUAS();
 * zoppler::TrackerAPI tracker(config);
 * 
 * // Process detections
 * std::vector<zoppler::Detection> detections = ...;
 * tracker.processDetections(detections, timestamp);
 * 
 * // Get track states
 * auto tracks = tracker.getTracks();
 * @endcode
 */

#endif // ZOPPLER_ZOPPLER_HPP
