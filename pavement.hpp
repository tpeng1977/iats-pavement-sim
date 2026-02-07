#ifndef PAVEMENT_HPP
#define PAVEMENT_HPP

#include <vector>
#include <cmath>
#include <string>
#include <fstream>
#include <algorithm>
#include <iostream>

// Theoretical Constants
const double FATIGUE_EXPONENT = 4.0;
// Heavy-vehicle tire width ~30 cm, contact patch slightly narrower. This model
// uses a Gaussian strain basin with sigma=15 cm for single-wheel lateral influence;
// the +/-45 cm computation range (3*sigma) is the stress diffusion zone in the pavement.
const double STRAIN_BASIN_SIGMA = 15.0; // cm
const double STRAIN_PEAK = 200.0; // microstrain

// Environmental Physics (Reviewer #1: Viscoelasticity & Temperature)
// Asphalt stiffness (E*) decreases as temperature increases, increasing strain.
// Simplified Arrhenius-like relationship for strain sensitivity to Temperature (T in Celsius).
// Strain(T) = Strain_ref * exp(k * (T - T_ref))
const double TEMP_REF = 20.0; // Reference temp Celsius
const double TEMP_SENSITIVITY = 0.04; // Strain increases ~4% per degree C (empirical proxy)

class Pavement {
private:
    int width_cm;
    std::vector<double> damage_grid; 
    std::vector<double> strain_buffer; 
    
    // Time tracking for daily cycles
    int current_pass_time_sec; // Seconds since simulation start (proxy)
    
public:
    Pavement(int width = 350) 
        : width_cm(width), damage_grid(width, 0.0), strain_buffer(width, 0.0), current_pass_time_sec(0) {}

    // Get current temperature based on daily cycle (Sinusoidal)
    // Assume 1 vehicle pass every ~3 seconds (heavy traffic)
    // 24 hour cycle = 86400 seconds
    double get_current_temperature() {
        // T_min = 15C (night), T_max = 45C (day pavement temp)
        const double T_MEAN = 30.0;
        const double T_AMP = 15.0;
        const double DAY_LENGTH = 86400.0; 
        
        // Simple sine wave
        double phase = (double)(current_pass_time_sec % (int)DAY_LENGTH) / DAY_LENGTH * 2.0 * M_PI;
        // Peak temp at 2PM (phase shift approx -2 hours from noon if 0 is midnight? Simplified: Peak at pi/2)
        // Let's just assume simple sine starting at mean.
        return T_MEAN + T_AMP * std::sin(phase);
    }

    void reset_buffer() {
        std::fill(strain_buffer.begin(), strain_buffer.end(), 0.0);
    }

    void apply_pass(double offset_cm) {
        reset_buffer();
        
        double center = width_cm / 2.0;
        double vehicle_center = center + offset_cm;
        double left_wheel_center = vehicle_center - 90.0;
        double right_wheel_center = vehicle_center + 90.0;

        // Calculate Temperature Scaling Factor
        double current_temp = get_current_temperature();
        double env_factor = std::exp(TEMP_SENSITIVITY * (current_temp - TEMP_REF));
        
        // Apply load with environmental scaling
        add_strain_basin(left_wheel_center, env_factor);
        add_strain_basin(right_wheel_center, env_factor);

        accumulate_damage();
        
        // Advance time (e.g. 5 seconds per car gap)
        current_pass_time_sec += 5;
    }

    // Single-wheel strain basin: Gaussian with sigma=15 cm, computed over +/-45 cm (stress diffusion zone).
    void add_strain_basin(double center_pos, double env_factor) {
        int range = (int)(STRAIN_BASIN_SIGMA * 3);
        double peak_strain = STRAIN_PEAK * env_factor; // Viscoelastic effect

        for (int i = -range; i <= range; ++i) {
            int idx = (int)(center_pos + i);
            if (idx >= 0 && idx < width_cm) {
                double dist = i;
                double strain = peak_strain * std::exp(-(dist * dist) / (2 * STRAIN_BASIN_SIGMA * STRAIN_BASIN_SIGMA));
                strain_buffer[idx] += strain;
            }
        }
    }

    void accumulate_damage() {
        for (int i = 0; i < width_cm; ++i) {
            if (strain_buffer[i] > 0) {
                // Miner's Rule: Damage is non-linear with strain
                double dmg = std::pow(strain_buffer[i], FATIGUE_EXPONENT);
                damage_grid[i] += dmg;
            }
        }
    }

    double get_max_damage() const {
        double max_d = 0.0;
        for (double d : damage_grid) {
            if (d > max_d) max_d = d;
        }
        return max_d;
    }
    
    const std::vector<double>& get_profile() const {
        return damage_grid;
    }

    void save_profile(const std::string& filename) {
        std::ofstream outfile(filename);
        if (outfile.is_open()) {
            for (int i = 0; i < width_cm; ++i) {
                outfile << i << "," << damage_grid[i] << "\n";
            }
            outfile.close();
        }
    }
};

#endif
