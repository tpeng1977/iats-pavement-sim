#include <cmath>
#include <fstream>
#include <functional>
// #include <iomanip> (removed unused)
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "distributions.hpp"
#include "pavement.hpp"

// #include <filesystem> (removed, handled by Makefile)
// namespace fs = std::filesystem;
const std::string DATA_DIR = "results/";

// Simulation Parameters
static constexpr int WIDTH_CM = 375;
static constexpr int NUM_PASSES = 100000;
static constexpr int NUM_TRIALS = 10; // Monte Carlo Trials
static constexpr double TRACK_HALF_CM = 90.0;

// Driver model constraints
static constexpr double OFFSET_LIMIT_CM = 30.0;   // comfort & lane-keeping (MAX_OFFSET)
static constexpr double CENTER_AVOID_CM = 15.0;    // mixed-traffic: AV avoids center band
static constexpr double DENSITY_PENALTY_SCALE = 1e14;  // complementary scheduling scale

struct TrialResult {
    double max_damage;
    std::vector<double> profile;
};

struct Stats {
    double mean;
    double stdev;
};

static Stats compute_stats_sample(const std::vector<double> &xs) {
    if (xs.empty())
        return {0.0, 0.0};
    if (xs.size() == 1)
        return {xs[0], 0.0};
    double sum = std::accumulate(xs.begin(), xs.end(), 0.0);
    double mean = sum / static_cast<double>(xs.size());
    double acc = 0.0;
    for (double x : xs) {
        double d = x - mean;
        acc += d * d;
    }
    double var = acc / static_cast<double>(xs.size() - 1);
    if (var < 0)
        var = 0;
    return {mean, std::sqrt(var)};
}

static double choose_feedback_offset(const Pavement &road, std::mt19937 &rng,
    double offset_limit_cm, double center_avoid_cm, double nonav_sigma_cm,
    double density_lambda) {
    const auto &profile = road.get_profile();
    const double center_idx = WIDTH_CM / 2.0;

    std::uniform_real_distribution<double> u(-offset_limit_cm, offset_limit_cm);

    double best_offset = 0.0;
    double best_obj = std::numeric_limits<double>::infinity();
    bool found = false;

    // Sample candidate offsets (stochastic search)
    for (int k = 0; k < 25; ++k) {
        double off = u(rng);
        if (center_avoid_cm > 0.0 && std::abs(off) < center_avoid_cm)
            continue;

        double left = center_idx + off - TRACK_HALF_CM;
        double right = center_idx + off + TRACK_HALF_CM;
        if (left < 0 || right >= WIDTH_CM)
            continue;

        double obj = profile[static_cast<int>(left)] + profile[static_cast<int>(right)];

        // Complementary scheduling: penalize overlap with non-AV lateral density
        // density(off) ~ exp(-off^2/(2*sigma^2)); lambda ~ O(1) is meaningful.
        if (density_lambda > 0.0 && nonav_sigma_cm > 0.0) {
            double density = std::exp(-(off * off) / (2.0 * nonav_sigma_cm * nonav_sigma_cm));
            obj += density_lambda * DENSITY_PENALTY_SCALE * density;
        }
        if (obj < best_obj) {
            best_obj = obj;
            best_offset = off;
            found = true;
        }
    }

    if (!found) {
        // Fallback (e.g., if center_avoid_cm is too large)
        best_offset = 0.0;
    }
    return best_offset;
}

// ---------- Single-trial runners ----------

static TrialResult run_trial_all_feedback(const std::string &name, int seed,
    bool log_history, double center_avoid_cm) {
    Pavement road(WIDTH_CM);
    std::mt19937 rng(static_cast<uint32_t>(seed));

    std::ofstream history_file;
    if (log_history) {
        history_file.open(DATA_DIR + "history_" + name + ".csv");
        if (history_file.is_open())
            history_file << "pass,max_damage\n";
    }

    for (int i = 0; i < NUM_PASSES; ++i) {
        double off = choose_feedback_offset(road, rng, OFFSET_LIMIT_CM,
            center_avoid_cm, 25.0, 0.0);
        road.apply_pass(off);
        if (log_history && history_file.is_open() && (i % 100 == 0)) {
            history_file << i << "," << road.get_max_damage() << "\n";
        }
    }

    if (log_history && history_file.is_open()) {
        history_file << NUM_PASSES << "," << road.get_max_damage() << "\n";
        history_file.close();
    }

    return {road.get_max_damage(), road.get_profile()};
}

static TrialResult run_trial_generator(const std::string &name, int seed,
    bool log_history,
    const std::function<std::unique_ptr<TrajectoryGenerator>(uint32_t)> &gen_factory) {
    Pavement road(WIDTH_CM);
    auto gen = gen_factory(static_cast<uint32_t>(seed));

    std::ofstream history_file;
    if (log_history) {
        history_file.open(DATA_DIR + "history_" + name + ".csv");
        if (history_file.is_open())
            history_file << "pass,max_damage\n";
    }

    for (int i = 0; i < NUM_PASSES; ++i) {
        road.apply_pass(gen->get_offset());
        if (log_history && history_file.is_open() && (i % 100 == 0)) {
            history_file << i << "," << road.get_max_damage() << "\n";
        }
    }

    if (log_history && history_file.is_open()) {
        history_file << NUM_PASSES << "," << road.get_max_damage() << "\n";
        history_file.close();
    }

    return {road.get_max_damage(), road.get_profile()};
}

enum class MixedAvMode {
    Precision,
    Feedback,
    FeedbackAvoidCenter,
    ComplementaryFeedback,
    AvoidCenterOnly
};

static double sample_avoid_center_offset(std::mt19937 &rng,
    double offset_limit_cm, double avoid_cm) {
    if (avoid_cm <= 0.0) {
        std::uniform_real_distribution<double> u(-offset_limit_cm, offset_limit_cm);
        return u(rng);
    }
    double hi = offset_limit_cm;
    double lo = avoid_cm;
    if (lo >= hi)
        return 0.0;
    std::bernoulli_distribution side(0.5);
    std::uniform_real_distribution<double> u(lo, hi);
    double mag = u(rng);
    return side(rng) ? mag : -mag;
}

static TrialResult run_trial_mixed(const std::string &name, int seed,
    bool log_history, double av_penetration, MixedAvMode av_mode,
    double center_avoid_cm, double nonav_sigma_cm = 25.0,
    double complementary_lambda0 = 0.0) {
    Pavement road(WIDTH_CM);
    std::mt19937 rng(static_cast<uint32_t>(seed));
    std::bernoulli_distribution is_av(av_penetration);

    // Non-AV vehicles keep their normal pattern (parameterized sigma for
    // channelization sensitivity)
    HumanTrajectory human(nonav_sigma_cm, static_cast<uint32_t>(seed + 1));
    // High-precision AV (future lane-centering / cooperative perception): sigma ~
    // 2 cm
    PrecisionTrajectory av_precise(2.0, static_cast<uint32_t>(seed + 2));

    std::ofstream history_file;
    if (log_history) {
        history_file.open(DATA_DIR + "history_" + name + ".csv");
        if (history_file.is_open())
            history_file << "pass,max_damage\n";
    }

    for (int i = 0; i < NUM_PASSES; ++i) {
        double off = 0.0;
        if (is_av(rng)) {
            if (av_mode == MixedAvMode::Precision) {
                off = av_precise.get_offset();
            } else if (av_mode == MixedAvMode::Feedback) {
                off = choose_feedback_offset(road, rng, OFFSET_LIMIT_CM, 0.0,
                    nonav_sigma_cm, 0.0);
            } else { // FeedbackAvoidCenter / ComplementaryFeedback / AvoidCenterOnly
                if (av_mode == MixedAvMode::FeedbackAvoidCenter) {
                    off = choose_feedback_offset(road, rng, OFFSET_LIMIT_CM,
                        center_avoid_cm, nonav_sigma_cm, 0.0);
                } else if (av_mode == MixedAvMode::AvoidCenterOnly) {
                    // Distribution-only center avoidance (no pavement sensing)
                    off = sample_avoid_center_offset(rng, OFFSET_LIMIT_CM, center_avoid_cm);
                } else {
                    // Complementary feedback: penalize overlap with non-AV density
                    // Adaptive weighting: as penetration rises, reduce penalty magnitude
                    double lambda = complementary_lambda0 * (1.0 - av_penetration);
                    off = choose_feedback_offset(road, rng, OFFSET_LIMIT_CM, 0.0,
                        nonav_sigma_cm, lambda);
                }
            }
        } else {
            off = human.get_offset();
        }

        road.apply_pass(off);

        if (log_history && history_file.is_open() && (i % 100 == 0)) {
            history_file << i << "," << road.get_max_damage() << "\n";
        }
    }

    if (log_history && history_file.is_open()) {
        history_file << NUM_PASSES << "," << road.get_max_damage() << "\n";
        history_file.close();
    }

    return {road.get_max_damage(), road.get_profile()};
}

// ---------- Monte Carlo wrappers ----------

static void save_stats_and_profile(const std::string &name,
    const std::vector<double> &max_damages,
    const std::vector<double> &avg_profile) {
    Stats s = compute_stats_sample(max_damages);

    std::cout << "  Mean Max Damage: " << s.mean << " (StdDev: " << s.stdev << ")"
              << std::endl;

    std::ofstream stat_file(DATA_DIR + "stats_" + name + ".csv");
    stat_file << "mean,std\n";
    stat_file << s.mean << "," << s.stdev << "\n";
    stat_file.close();

    std::ofstream profile_file(DATA_DIR + "profile_" + name + ".csv");
    for (size_t i = 0; i < avg_profile.size(); ++i) {
        profile_file << i << "," << avg_profile[i] << "\n";
    }
    profile_file.close();
}

static Stats save_stats_and_profile_return(const std::string &name,
    const std::vector<double> &max_damages,
    const std::vector<double> &avg_profile) {
    Stats s = compute_stats_sample(max_damages);
    std::cout << "  Mean Max Damage: " << s.mean << " (StdDev: " << s.stdev << ")"
              << std::endl;

    std::ofstream stat_file(DATA_DIR + "stats_" + name + ".csv");
    stat_file << "mean,std\n";
    stat_file << s.mean << "," << s.stdev << "\n";
    stat_file.close();

    std::ofstream profile_file(DATA_DIR + "profile_" + name + ".csv");
    for (size_t i = 0; i < avg_profile.size(); ++i) {
        profile_file << i << "," << avg_profile[i] << "\n";
    }
    profile_file.close();

    return s;
}

static void run_monte_carlo_group_generator(const std::string &name,
    const std::function<std::unique_ptr<TrajectoryGenerator>(uint32_t)> &gen_factory) {
    std::cout << "Running Monte Carlo (" << NUM_TRIALS << " trials): " << name
              << "..." << std::endl;

    std::vector<double> max_damages;
    std::vector<double> avg_profile(WIDTH_CM, 0.0);

    for (int t = 0; t < NUM_TRIALS; ++t) {
        int seed = 42 + t * 100; // Distinct seeds
        bool log = (t == 0);
        auto result = run_trial_generator(name, seed, log, gen_factory);
        max_damages.push_back(result.max_damage);
        for (size_t i = 0; i < result.profile.size(); ++i) {
            avg_profile[i] += result.profile[i] / static_cast<double>(NUM_TRIALS);
        }
    }

    save_stats_and_profile(name, max_damages, avg_profile);
}

static void run_monte_carlo_group_feedback(const std::string &name,
    double center_avoid_cm) {
    std::cout << "Running Monte Carlo (" << NUM_TRIALS << " trials): " << name
              << "..." << std::endl;

    std::vector<double> max_damages;
    std::vector<double> avg_profile(WIDTH_CM, 0.0);

    for (int t = 0; t < NUM_TRIALS; ++t) {
        int seed = 42 + t * 100;
        bool log = (t == 0);
        auto result = run_trial_all_feedback(name, seed, log, center_avoid_cm);
        max_damages.push_back(result.max_damage);
        for (size_t i = 0; i < result.profile.size(); ++i) {
            avg_profile[i] += result.profile[i] / static_cast<double>(NUM_TRIALS);
        }
    }

    save_stats_and_profile(name, max_damages, avg_profile);
}

static void append_mix_summary(std::ofstream &out, const std::string &tag,
    int p_pct, const std::vector<double> &max_damages) {
    Stats s = compute_stats_sample(max_damages);
    out << tag << "," << p_pct << "," << s.mean << "," << s.stdev << "\n";
}

static void run_monte_carlo_mixed_suite() {
    std::cout << "Running Mixed-Traffic Penetration Suite (Monte Carlo)..."
              << std::endl;

    std::vector<int> p_pcts = {0, 25, 50, 75, 100};

    std::ofstream summary(DATA_DIR + "mix_summary.csv");
    summary << "mode,pct,mean,std\n";

    // Grid sweep over center-avoid widths for deployment-phase optimization
    std::vector<double> avoid_grid_cm = {0.0, 5.0, 10.0, 15.0, 20.0};
    std::ofstream grid(DATA_DIR + "mix_grid_summary.csv");
    grid << "mode,pct,avoid_cm,mean,std\n";

    // Complementary scheduling grid (lambda0 sweep)
    std::vector<double> lambda0_grid = {0.0, 0.25, 0.5, 1.0, 2.0};
    std::ofstream comp(DATA_DIR + "mix_complementary_grid.csv");
    comp << "pct,lambda0,mean,std\n";

    // Distribution-only center avoidance grid (no sensing)
    std::ofstream avoidonly(DATA_DIR + "mix_avoidonly_grid.csv");
    avoidonly << "pct,avoid_cm,mean,std\n";

    for (int p_pct : p_pcts) {
        double p = static_cast<double>(p_pct) / 100.0;

        // Three configurations:
        // 1) AVs drive precise centerline (precision paradox in mixed traffic)
        // 2) AVs run feedback (IATS L2)
        // 3) AVs run feedback + avoid center band to complement human traffic
        struct Config {
            const char *tag;
            MixedAvMode mode;
            double avoid;
        };
        std::vector<Config> configs = {
            {"MixPrecision", MixedAvMode::Precision, 0.0},
            {"MixFeedback", MixedAvMode::Feedback, 0.0},
            {"MixFeedbackAvoidCenter", MixedAvMode::FeedbackAvoidCenter, CENTER_AVOID_CM},
        };

        for (const auto &cfg : configs) {
            std::string name = std::string("GroupE_") + cfg.tag + "_p" + std::to_string(p_pct);
            std::cout << "  " << name << "..." << std::endl;

            std::vector<double> max_damages;
            std::vector<double> avg_profile(WIDTH_CM, 0.0);

            for (int t = 0; t < NUM_TRIALS; ++t) {
                int seed = 4242 + p_pct * 17 + t * 131;
                bool log = (t == 0); // representative history per (mode,p)
                auto result = run_trial_mixed(name, seed, log, p, cfg.mode, cfg.avoid, 25.0);
                max_damages.push_back(result.max_damage);
                for (size_t i = 0; i < result.profile.size(); ++i) {
                    avg_profile[i] += result.profile[i] / static_cast<double>(NUM_TRIALS);
                }
            }

            save_stats_and_profile(name, max_damages, avg_profile);
            append_mix_summary(summary, cfg.tag, p_pct, max_damages);

            // Also record in grid file (only one avoid value per cfg)
            Stats s = compute_stats_sample(max_damages);
            grid << cfg.tag << "," << p_pct << "," << cfg.avoid << "," << s.mean
                 << "," << s.stdev << "\n";
        }

        // Sweep center-avoid widths for FeedbackAvoidCenter configuration to find
        // optimal deployment settings
        for (double avoid_cm : avoid_grid_cm) {
            std::string name = std::string("GroupE_MixFeedbackAvoidCenter") + "_p"
                + std::to_string(p_pct) + "_w" + std::to_string(static_cast<int>(avoid_cm));
            std::cout << "  " << name << "..." << std::endl;

            std::vector<double> max_damages;
            std::vector<double> avg_profile(WIDTH_CM, 0.0);

            for (int t = 0; t < NUM_TRIALS; ++t) {
                int seed = 9000 + p_pct * 19 + static_cast<int>(avoid_cm) * 7 + t * 131;
                bool log = (t == 0 && p_pct == 50
                    && static_cast<int>(avoid_cm) == static_cast<int>(CENTER_AVOID_CM));
                auto result = run_trial_mixed(name, seed, log, p,
                    MixedAvMode::FeedbackAvoidCenter, avoid_cm, 25.0);
                max_damages.push_back(result.max_damage);
                for (size_t i = 0; i < result.profile.size(); ++i) {
                    avg_profile[i] += result.profile[i] / static_cast<double>(NUM_TRIALS);
                }
            }

            Stats s = save_stats_and_profile_return(name, max_damages, avg_profile);
            grid << "MixFeedbackAvoidCenterGrid" << "," << p_pct << "," << avoid_cm
                 << "," << s.mean << "," << s.stdev << "\n";
        }

        // Sweep complementary penalty weights (lambda0). We only store summary
        // statistics (no per-config profiles).
        for (double lambda0 : lambda0_grid) {
            std::vector<double> max_damages;
            for (int t = 0; t < NUM_TRIALS; ++t) {
                int seed = 20000 + p_pct * 31 + static_cast<int>(lambda0 * 100) * 13 + t * 131;
                auto result = run_trial_mixed("GroupE_MixComplementary", seed, false, p,
                    MixedAvMode::ComplementaryFeedback, 0.0, 25.0, lambda0);
                max_damages.push_back(result.max_damage);
            }
            Stats s = compute_stats_sample(max_damages);
            comp << p_pct << "," << lambda0 << "," << s.mean << "," << s.stdev << "\n";
        }

        // Sweep distribution-only center avoidance (no sensing). Summary only.
        for (double avoid_cm : avoid_grid_cm) {
            std::vector<double> max_damages;
            for (int t = 0; t < NUM_TRIALS; ++t) {
                int seed = 26000 + p_pct * 37 + static_cast<int>(avoid_cm) * 17 + t * 131;
                auto result = run_trial_mixed("GroupE_MixAvoidOnly", seed, false, p,
                    MixedAvMode::AvoidCenterOnly, avoid_cm, 25.0, 0.0);
                max_damages.push_back(result.max_damage);
            }
            Stats s = compute_stats_sample(max_damages);
            avoidonly << p_pct << "," << avoid_cm << "," << s.mean << "," << s.stdev << "\n";
        }
    }

    summary.close();
    grid.close();
    comp.close();
    avoidonly.close();
}

static void run_monte_carlo_mixed_suite_channelized() {
    std::cout << "Running Mixed-Traffic Sensitivity Suite: channelized non-AV traffic..."
              << std::endl;
    std::vector<int> p_pcts = {0, 25, 50, 75, 100};
    std::vector<double> avoid_grid_cm = {0.0, 5.0, 10.0, 15.0, 20.0};

    std::ofstream grid(DATA_DIR + "mix_grid_summary_channelized.csv");
    grid << "mode,pct,avoid_cm,mean,std\n";

    for (int p_pct : p_pcts) {
        double p = static_cast<double>(p_pct) / 100.0;

        // Compare naive precision and feedback under channelized non-AV traffic
        // (sigma=10cm)
        for (const auto &cfg : std::vector<std::pair<std::string, MixedAvMode>>{
                 {"MixPrecision", MixedAvMode::Precision},
                 {"MixFeedback", MixedAvMode::Feedback},
             }) {
            std::string name = std::string("GroupF_") + cfg.first + "_p" + std::to_string(p_pct);
            std::cout << "  " << name << "..." << std::endl;

            std::vector<double> max_damages;
            std::vector<double> avg_profile(WIDTH_CM, 0.0);

            for (int t = 0; t < NUM_TRIALS; ++t) {
                int seed = 12000 + p_pct * 23 + t * 131;
                bool log = (t == 0 && p_pct == 50);
                auto result = run_trial_mixed(name, seed, log, p, cfg.second, 0.0, 10.0);
                max_damages.push_back(result.max_damage);
                for (size_t i = 0; i < result.profile.size(); ++i) {
                    avg_profile[i] += result.profile[i] / static_cast<double>(NUM_TRIALS);
                }
            }

            Stats s = save_stats_and_profile_return(name, max_damages, avg_profile);
            grid << cfg.first << "," << p_pct << "," << 0.0 << "," << s.mean << ","
                 << s.stdev << "\n";
        }

        // Sweep center-avoid widths for FeedbackAvoidCenter
        for (double avoid_cm : avoid_grid_cm) {
            std::string name = std::string("GroupF_MixFeedbackAvoidCenter") + "_p"
                + std::to_string(p_pct) + "_w" + std::to_string(static_cast<int>(avoid_cm));
            std::cout << "  " << name << "..." << std::endl;

            std::vector<double> max_damages;
            std::vector<double> avg_profile(WIDTH_CM, 0.0);

            for (int t = 0; t < NUM_TRIALS; ++t) {
                int seed = 15000 + p_pct * 29 + static_cast<int>(avoid_cm) * 11 + t * 131;
                bool log = false;
                auto result = run_trial_mixed(name, seed, log, p,
                    MixedAvMode::FeedbackAvoidCenter, avoid_cm, 10.0);
                max_damages.push_back(result.max_damage);
                for (size_t i = 0; i < result.profile.size(); ++i) {
                    avg_profile[i] += result.profile[i] / static_cast<double>(NUM_TRIALS);
                }
            }

            Stats s = save_stats_and_profile_return(name, max_damages, avg_profile);
            grid << "MixFeedbackAvoidCenterGrid" << "," << p_pct << "," << avoid_cm
                 << "," << s.mean << "," << s.stdev << "\n";
        }
    }

    grid.close();
}

int main() {
    // Ensure output directory exists (std::filesystem require C++17)
    // Output directory creation handled by Makefile (mkdir -p results)
    // if (!fs::exists(DATA_DIR)) {
    //   fs::create_directory(DATA_DIR);
    // }
    // Baseline groups (single-policy worlds)
    run_monte_carlo_group_generator("GroupA_Precision", [](uint32_t seed) {
        return std::make_unique<PrecisionTrajectory>(2.0, seed);
    });
    run_monte_carlo_group_generator("GroupB_Human", [](uint32_t seed) {
        return std::make_unique<HumanTrajectory>(25.0, seed);
    });
    run_monte_carlo_group_generator("GroupC_Uniform", [](uint32_t seed) {
        return std::make_unique<UniformTrajectory>(30.0, seed);
    });
    run_monte_carlo_group_feedback("GroupD_Feedback", 0.0);

    // New: mixed-traffic analysis with configurable AV feedback that avoids lane
    // center
    run_monte_carlo_mixed_suite();
    run_monte_carlo_mixed_suite_channelized();

    std::cout << "All simulations complete." << std::endl;
    return 0;
}
