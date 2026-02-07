#ifndef DISTRIBUTIONS_HPP
#define DISTRIBUTIONS_HPP

#include <random>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>

// Constants
const double LANE_WIDTH = 375.0; // cm (Updated to standard 3.75m)
const double VEHICLE_WIDTH = 200.0; // cm
const double MAX_OFFSET = 30.0; // cm (as per guide)

// Abstract base class for Trajectory Generator
class TrajectoryGenerator {
public:
    virtual double get_offset() = 0;
    virtual ~TrajectoryGenerator() = default;
};

// Group A: Minimal Deviation (Machine Precision)
// Normal Distribution with small sigma
// Literature: AV lateral error RMS ~ 8-20 cm (e.g. 0.08m @ 10m/s MPC)
// Let's use sigma = 10.0 cm to be conservative but distinct from human
class PrecisionTrajectory : public TrajectoryGenerator {
    std::mt19937 gen;
    std::normal_distribution<> d;
public:
    PrecisionTrajectory(double sigma = 10.0, uint32_t seed = 42) : gen(seed), d(0.0, sigma) {}
    double get_offset() override {
        return std::clamp(d(gen), -MAX_OFFSET, MAX_OFFSET);
    }
};

// Group B: Human Driving
// Normal Distribution with larger sigma
// Literature: Human lateral wander SD ~ 20-30 cm
class HumanTrajectory : public TrajectoryGenerator {
    std::mt19937 gen;
    std::normal_distribution<> d;
public:
    HumanTrajectory(double sigma = 25.0, uint32_t seed = 42) : gen(seed), d(0.0, sigma) {}
    double get_offset() override {
        return std::clamp(d(gen), -MAX_OFFSET, MAX_OFFSET);
    }
};

// Group C: Uniform Distribution (IATS Level 1)
class UniformTrajectory : public TrajectoryGenerator {
    std::mt19937 gen;
    std::uniform_real_distribution<> d;
public:
    UniformTrajectory(double limit = 30.0, uint32_t seed = 42) : gen(seed), d(-limit, limit) {}
    double get_offset() override {
        return d(gen);
    }
};

// Group D: Active Feedback (IATS Level 2)
// This needs state awareness, so we'll implement the logic in the main loop
// or pass the pavement state to it.
class FeedbackTrajectory : public TrajectoryGenerator {
    std::mt19937 gen;
    std::uniform_real_distribution<> jitter;
public:
    FeedbackTrajectory() : gen(42), jitter(-1.0, 1.0) {}
    
    // Simple placeholder, real logic requires pavement state input
    double get_offset() override {
        return 0.0; 
    }

    // Logic to find lowest damage spot
    double get_smart_offset(const std::vector<double>& damage_profile, double center_index) {
        // Simple heuristic: find local minimum in valid range
        // In a real implementation, this would look for "valleys" in the damage profile
        // For simulation, we scan the valid range indices.
        
        // This is a simplified "Monte Carlo" choice for the demo:
        // Sample a few points, pick the one that would result in lowest new peak damage.
        // Or sweep.
        return jitter(gen) * MAX_OFFSET; // Placeholder for compilation if used directly
    }
};

#endif
