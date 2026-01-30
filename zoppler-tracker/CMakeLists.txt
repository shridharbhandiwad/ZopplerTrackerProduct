cmake_minimum_required(VERSION 3.14)
project(ZopplerTracker VERSION 1.0.0 LANGUAGES CXX)

# C++17 required
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Optimization flags for release
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")

# Options
option(ZOPPLER_BUILD_EXAMPLES "Build example applications" ON)
option(ZOPPLER_BUILD_TESTS "Build tests" OFF)
option(ZOPPLER_BUILD_SHARED "Build shared library" OFF)

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Collect source files
set(ZOPPLER_SOURCES
    # Core (header-only, but include for IDE)
    
    # Utils
    # (Matrix is header-only)
    
    # Clustering
    src/clustering/DBSCANClustering.cpp
    src/clustering/ContinuousRangeClustering.cpp
    src/clustering/ClusteringFactory.cpp
    
    # Association
    src/association/GNNAssociation.cpp
    src/association/JPDAAssociation.cpp
    src/association/MHTAssociation.cpp
    src/association/AssociationFactory.cpp
    
    # Tracking
    src/tracking/EKFTracker.cpp
    src/tracking/UKFTracker.cpp
    src/tracking/IMMTracker.cpp
    src/tracking/ParticleFilterTracker.cpp
    src/tracking/TrackerFactory.cpp
    
    # Engine
    src/engine/TrackManager.cpp
    src/engine/TrackerEngine.cpp
    
    # API
    src/api/TrackerAPI.cpp
    
    # Config
    src/config/JsonLoader.cpp
    src/config/JsonSaver.cpp
)

# Main library
if(ZOPPLER_BUILD_SHARED)
    add_library(zoppler_tracker SHARED ${ZOPPLER_SOURCES})
else()
    add_library(zoppler_tracker STATIC ${ZOPPLER_SOURCES})
endif()

target_include_directories(zoppler_tracker PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Thread support
find_package(Threads REQUIRED)
target_link_libraries(zoppler_tracker PUBLIC Threads::Threads)

# Example application
if(ZOPPLER_BUILD_EXAMPLES)
    add_executable(zoppler_demo examples/main.cpp)
    target_link_libraries(zoppler_demo PRIVATE zoppler_tracker)
endif()

# Installation
install(TARGETS zoppler_tracker
    EXPORT ZopplerTrackerTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/zoppler
    DESTINATION include
)

install(EXPORT ZopplerTrackerTargets
    FILE ZopplerTrackerConfig.cmake
    NAMESPACE Zoppler::
    DESTINATION lib/cmake/ZopplerTracker
)

# Package config
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/ZopplerTrackerConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/ZopplerTrackerConfigVersion.cmake"
    DESTINATION lib/cmake/ZopplerTracker
)

# Summary
message(STATUS "")
message(STATUS "Zoppler Tracker Configuration:")
message(STATUS "  Version:        ${PROJECT_VERSION}")
message(STATUS "  Build type:     ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ Standard:   ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build examples: ${ZOPPLER_BUILD_EXAMPLES}")
message(STATUS "  Build tests:    ${ZOPPLER_BUILD_TESTS}")
message(STATUS "  Shared library: ${ZOPPLER_BUILD_SHARED}")
message(STATUS "")
