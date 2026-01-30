# Zoppler Tracker

A defense-grade, modular multi-target radar tracking library written in C++17.

## Overview

Zoppler Tracker is a complete radar tracking pipeline that includes clustering, data association, and tracking/filtering algorithms. It is designed for real-time multi-target tracking applications such as Counter-UAS (CUAS), air surveillance, and general radar systems.

## Features

- **Modular Architecture**: Interchangeable algorithms for clustering, association, and tracking
- **Multiple Clustering Algorithms**:
  - DBSCAN (Density-Based Spatial Clustering)
  - Continuous Range Clustering
- **Multiple Data Association Algorithms**:
  - GNN (Global Nearest Neighbor)
  - JPDA (Joint Probabilistic Data Association)
  - MHT (Multiple Hypothesis Tracking)
- **Multiple Tracking/Filtering Algorithms**:
  - EKF (Extended Kalman Filter)
  - UKF (Unscented Kalman Filter)
  - IMM (Interacting Multiple Model)
  - Particle Filter
- **Motion Models**:
  - CV (Constant Velocity)
  - CA (Constant Acceleration)
  - CT (Coordinated Turn)
- **Thread-safe API**: Suitable for multi-threaded applications
- **JSON Configuration**: Load/save configurations in JSON format
- **Preset Configurations**: Pre-tuned profiles for common use cases

## Prerequisites

Before building Zoppler Tracker, ensure you have the following installed:

### Required

- **C++ Compiler**: GCC 7+, Clang 5+, or MSVC 2017+ with C++17 support
- **CMake**: Version 3.14 or higher
- **POSIX Threads**: pthread library (usually pre-installed on Linux/macOS)

### Installing Prerequisites

**Ubuntu/Debian:**

```bash
sudo apt update
sudo apt install build-essential cmake
```

**Fedora/RHEL:**

```bash
sudo dnf install gcc-c++ cmake make
```

**macOS (with Homebrew):**

```bash
brew install cmake
```

**Windows:**

- Install Visual Studio 2017 or later with C++ development tools
- Install CMake from https://cmake.org/download/

## Building from Source

### Step 1: Clone the Repository

```bash
git clone <repository-url>
cd zoppler-tracker
```

### Step 2: Create Build Directory

```bash
mkdir build
cd build
```

### Step 3: Configure with CMake

**Standard build (Release mode):**

```bash
cmake ..
```

**Debug build:**

```bash
cmake -DCMAKE_BUILD_TYPE=Debug ..
```

**Build shared library instead of static:**

```bash
cmake -DZOPPLER_BUILD_SHARED=ON ..
```

**All available CMake options:**

| Option | Default | Description |
|--------|---------|-------------|
| `CMAKE_BUILD_TYPE` | Release | Build type (Debug/Release) |
| `ZOPPLER_BUILD_EXAMPLES` | ON | Build demo application |
| `ZOPPLER_BUILD_TESTS` | OFF | Build unit tests |
| `ZOPPLER_BUILD_SHARED` | OFF | Build shared library (ON) or static (OFF) |

### Step 4: Build the Project

```bash
cmake --build . --config Release
```

Or using make directly (Linux/macOS):

```bash
make -j$(nproc)
```

### Step 5: Install (Optional)

```bash
sudo cmake --install .
```

Or with make:

```bash
sudo make install
```

This installs:
- Library to `/usr/local/lib/`
- Headers to `/usr/local/include/zoppler/`
- CMake config to `/usr/local/lib/cmake/ZopplerTracker/`

## Running the Demo

After building, run the demo application:

```bash
# From the build directory
./zoppler_demo
```

The demo demonstrates:
1. Basic tracking with simulated targets
2. Different algorithm combinations
3. JSON configuration save/load
4. Preset configurations

**Expected output:**

```
========================================
   Zoppler Tracker - Demo Application   
   Defense-Grade Multi-Target Tracker   
========================================

=== Demo 1: Basic Tracking ===
Tracker initialized with:
  Clustering: DBSCAN
  Association: GNN
  Tracking: EKF
  Motion Model: CA
  Profile: SHORT_RANGE
...
```

## Quick Start Guide

### Basic Usage

```cpp
#include <zoppler/Zoppler.hpp>

using namespace zoppler;

int main() {
    // Create tracker with preset configuration
    TrackerConfig config = TrackerConfig::shortRangeCUAS();
    TrackerAPI tracker(config);
    
    // Create detections
    std::vector<Detection> detections;
    Detection det;
    det.x = 100.0;  // meters
    det.y = 50.0;
    det.z = 25.0;
    det.timestamp = 1000000;  // microseconds
    det.computePolarFromCartesian();
    detections.push_back(det);
    
    // Process detections
    tracker.processDetections(detections, det.timestamp);
    
    // Get tracks
    auto tracks = tracker.getTracks();
    for (const auto& track : tracks) {
        std::cout << "Track at (" << track.x << ", " << track.y << ", " << track.z << ")\n";
    }
    
    return 0;
}
```

### Custom Configuration

```cpp
#include <zoppler/Zoppler.hpp>

using namespace zoppler;

int main() {
    TrackerConfig config;
    
    // Select algorithms
    config.clusteringAlgo = "DBSCAN";       // or "CONTINUOUS_RANGE"
    config.associationAlgo = "JPDA";        // or "GNN", "MHT"
    config.trackingAlgo = "UKF";            // or "EKF", "IMM", "PARTICLE_FILTER"
    config.motionModel = "CA";              // or "CV", "CT"
    config.profile = "MEDIUM_RANGE";        // or "SHORT_RANGE", "LONG_RANGE", "CLUTTER_HEAVY"
    
    // Customize clustering
    config.clusteringParams.epsilon = 15.0;
    config.clusteringParams.minPoints = 3;
    
    // Customize track management
    config.trackManagement.minHitsToConfirm = 4;
    config.trackManagement.maxConsecutiveMisses = 7;
    
    // Resolve profile and create tracker
    config.resolveProfile();
    TrackerAPI tracker(config);
    
    return 0;
}
```

### JSON Configuration

```cpp
#include <zoppler/Zoppler.hpp>

using namespace zoppler;

int main() {
    // Load configuration from file
    TrackerAPI tracker;
    tracker.loadConfigFromFile("config.json");
    
    // Or save current config
    TrackerConfig config = TrackerConfig::mediumRangeSurveillance();
    TrackerAPI tracker2(config);
    tracker2.saveConfigToFile("my_config.json");
    
    // Get config as JSON string
    std::string jsonStr = tracker2.getConfigAsJson();
    
    return 0;
}
```

## Preset Configurations

The library provides optimized presets for common scenarios:

| Preset | Use Case | Algorithms |
|--------|----------|------------|
| `shortRangeCUAS()` | Counter-UAS, FMCW radar | DBSCAN + GNN + EKF(CA) |
| `mediumRangeSurveillance()` | General surveillance | Range Clustering + JPDA + IMM |
| `longRangeCrossing()` | Long-range, crossing targets | Range Clustering + MHT + IMM |
| `clutterHeavyBirds()` | High clutter, birds | DBSCAN + JPDA + Particle Filter |

## Integrating into Your Project

### Using CMake (Recommended)

After installing, add to your `CMakeLists.txt`:

```cmake
find_package(ZopplerTracker REQUIRED)

add_executable(my_app main.cpp)
target_link_libraries(my_app PRIVATE Zoppler::zoppler_tracker)
```

### Using pkg-config

```bash
g++ -std=c++17 main.cpp $(pkg-config --cflags --libs zoppler-tracker)
```

### Manual Linking

```bash
g++ -std=c++17 -I/usr/local/include main.cpp -L/usr/local/lib -lzoppler_tracker -lpthread
```

## Algorithm Selection Guide

### Clustering Algorithms

| Algorithm | Best For | Performance |
|-----------|----------|-------------|
| **DBSCAN** | Irregular clusters, varying density | O(n²) worst case |
| **Continuous Range** | Uniform range bins, FMCW radar | O(n log n) |

### Data Association Algorithms

| Algorithm | Best For | Complexity |
|-----------|----------|------------|
| **GNN** | Simple scenarios, real-time | O(n³) |
| **JPDA** | Dense targets, clutter | O(n² × m²) |
| **MHT** | Crossing targets, ambiguous | O(exp) but pruned |

### Tracking Algorithms

| Algorithm | Best For | Notes |
|-----------|----------|-------|
| **EKF** | Mildly nonlinear, fast | Most common choice |
| **UKF** | Highly nonlinear | Better accuracy, slower |
| **IMM** | Maneuvering targets | Multiple motion models |
| **Particle Filter** | Non-Gaussian noise, clutter | Most flexible, slowest |

## API Reference

### TrackerAPI Class

Main interface for the tracker.

```cpp
class TrackerAPI {
    // Construction
    TrackerAPI();
    explicit TrackerAPI(const TrackerConfig& config);
    
    // Core operations
    void processDetections(const std::vector<Detection>& detections, int64_t timestamp);
    std::vector<TrackState> getTracks() const;
    std::vector<TrackState> getConfirmedTracks() const;
    
    // Configuration
    bool initialize(const TrackerConfig& config);
    TrackerConfig getConfig() const;
    void setConfig(const TrackerConfig& config);
    
    // Track queries
    int getNumTracks() const;
    int getNumConfirmedTracks() const;
    bool getTrackById(int trackId, TrackState& state) const;
    
    // Statistics
    Statistics getStatistics() const;
    
    // JSON configuration
    bool loadConfigFromFile(const std::string& filepath);
    bool saveConfigToFile(const std::string& filepath) const;
};
```

### Detection Structure

```cpp
struct Detection {
    double x, y, z;           // Cartesian position (meters)
    double range, azimuth, elevation;  // Polar coordinates
    double doppler;           // Radial velocity (m/s)
    double snr;               // Signal-to-noise ratio (dB)
    double quality;           // Detection quality [0, 1]
    Timestamp timestamp;      // Microseconds since epoch
    
    void computePolarFromCartesian();
    void computeCartesianFromPolar();
};
```

### TrackState Structure

```cpp
struct TrackState {
    int id;                   // Unique track ID
    double x, y, z;           // Position (meters)
    double vx, vy, vz;        // Velocity (m/s)
    double confidence;        // Track confidence [0, 1]
    bool confirmed;           // Track confirmation status
    Timestamp lastUpdate;     // Last update timestamp
    
    double getSpeed() const;
    double getHeading() const;
};
```

## Project Structure

```
zoppler-tracker/
├── CMakeLists.txt          # Build configuration
├── README.md               # This file
├── include/
│   └── zoppler/
│       ├── Zoppler.hpp     # Main include (all-in-one)
│       ├── api/            # Public API
│       ├── association/    # Data association algorithms
│       ├── clustering/     # Clustering algorithms
│       ├── config/         # JSON configuration
│       ├── core/           # Core types and structures
│       ├── engine/         # Tracker engine
│       ├── tracking/       # Tracking/filtering algorithms
│       └── utils/          # Utilities (math, matrix)
├── src/                    # Implementation files
│   ├── api/
│   ├── association/
│   ├── clustering/
│   ├── config/
│   ├── engine/
│   └── tracking/
└── examples/
    └── main.cpp            # Demo application
```

## Troubleshooting

### Build Errors

**"CMake version too old"**
```bash
# Ubuntu/Debian
sudo apt install cmake
# Or download from https://cmake.org/download/
```

**"Could not find Threads"**
```bash
# Ubuntu/Debian
sudo apt install libc6-dev
```

**"C++17 not supported"**
Update your compiler:
```bash
# Ubuntu
sudo apt install g++-9
cmake -DCMAKE_CXX_COMPILER=g++-9 ..
```

### Runtime Errors

**"Library not found"** after installation:
```bash
sudo ldconfig
# Or add to LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

## License

Proprietary - Defense-grade radar tracking system.

## Support

For issues and feature requests, please open an issue on the project repository.
