# Pavement Damage Simulation — Infrastructure-Aware Trajectory Scheduling (IATS)

**Repository:** [iats-pavement-sim](https://github.com/tpeng1977/iats-pavement-sim) 

Mechanistic–empirical simulation of how **vehicle lateral trajectory** (wheel path distribution) affects **flexible pavement damage**. The code evaluates the “Precision Paradox”: high-precision lane-keeping concentrates loads and accelerates damage, while **Infrastructure-Aware Trajectory Scheduling (IATS)** redistributes wheel paths within the lane to extend pavement life.

## What This Repository Does

- **Pavement model**: Transverse strain from each wheel pass (Gaussian strain basin, temperature-scaled), fatigue damage accumulation (Miner’s rule, strain^4), daily temperature cycle.
- **Trajectory policies**: Baseline (precision), human-like wander, uniform (IATS Level 1), damage-aware feedback (IATS Level 2), center-avoid, complementary scheduling.
- **Monte Carlo**: Multiple trials per configuration; statistics and averaged transverse damage profiles.
- **Mixed traffic**: AV penetration 0–100%; comparison of precision vs feedback vs center-avoid vs complementary feedback.
- **LCA**: Annualized carbon footprint from damage-based relative lifespan (embodied CO₂ per km amortized over life).

## Directory Layout

| Path | Description |
|------|-------------|
| `main.cpp` | C++ entry point: trajectory policies, Monte Carlo loops, mixed-traffic suites, CSV output. |
| `pavement.hpp` | Pavement class: strain basin, temperature, damage accumulation, profile access. |
| `distributions.hpp` | Trajectory generators: Precision, Human, Uniform, Feedback (interface). |
| `visualize.py` | Python script: reads `results/*.csv`, produces `figures/figure1.svg`–`figure4.svg` and LCA bars. |
| `Makefile` | Builds `run_sim`, optional `make run` (sim + plots), `make plot` (plots only), `make clean`. |
| `results/` | Output CSVs: `stats_*`, `profile_*`, `history_*`, `mix_summary.csv`, `mix_grid_summary*.csv`, etc. |
| `figures/` | Generated figures (SVG): transverse damage profile, degradation history, impact/LCA, mixed-traffic, channelized sensitivity. |

## Requirements

- **Build**: C++17 compiler (e.g. `g++`), `make`.
- **Run**: The executable is self-contained; no external data files required.
- **Plots**: Python 3 with `matplotlib`, `pandas`, `numpy`. Install with e.g. `pip install matplotlib pandas numpy`.

## Build and Run

```bash
# Build the C++ simulator
make

# Run simulation only (writes results/*.csv)
./run_sim

# Run simulation and then generate figures
make run

# Regenerate figures from existing results/
make plot
```

The first run creates `results/` and `figures/` (Makefile ensures directories exist for `make run`).

## Main Parameters (C++)

| Symbol | Value | Meaning |
|--------|--------|--------|
| `WIDTH_CM` | 375 | Lane width (cm). |
| `NUM_PASSES` | 100,000 | Vehicle passes per trial. |
| `NUM_TRIALS` | 10 | Monte Carlo trials per configuration. |
| `TRACK_HALF_CM` | 90 | Half axle track (cm); wheel centers at ±90 cm from vehicle center. |
| `OFFSET_LIMIT_CM` | 30 | Max lateral offset (cm); comfort/lane-keeping. |
| `CENTER_AVOID_CM` | 15 | Center band width (cm) for “avoid center” policy. |

Pavement (`pavement.hpp`): `STRAIN_BASIN_SIGMA` 15 cm, `STRAIN_PEAK` 200 µε, `FATIGUE_EXPONENT` 4, temperature scaling ~4%/°C.

## Trajectory Groups and Mixed Traffic

- **Group A (Precision)**: Normal(0, 2 cm) — high-precision lane-keeping.
- **Group B (Human)**: Normal(0, 25 cm) — human-like lateral wander.
- **Group C (Uniform)**: Uniform(±30 cm) — IATS Level 1.
- **Group D (Feedback)**: Damage-aware; chooses lateral offset to minimize incremental damage (IATS Level 2).

Mixed-traffic runs vary **AV penetration** (0, 25, 50, 75, 100%) and compare:

- Naive precision (AVs on centerline),
- Feedback (damage-aware),
- Feedback + center-avoid,
- Complementary feedback (penalize overlap with human lateral density),
- Avoid-center only (no pavement sensing).

Channelized sensitivity (Group F) uses a narrower non-AV lateral distribution (σ = 10 cm).

## Outputs

- **`results/stats_<name>.csv`**: `mean`, `std` of max damage over trials.
- **`results/profile_<name>.csv`**: Transverse damage profile (position, damage) — Monte Carlo average.
- **`results/history_<name>.csv`**: Optional; pass index vs max damage for one trial.
- **`results/mix_summary.csv`**: Mixed-traffic summary (mode, pct, mean, std).
- **`results/mix_grid_summary.csv`**, **`mix_grid_summary_channelized.csv`**: Grid over center-avoid width and penetration.
- **`results/mix_complementary_grid.csv`**, **`mix_avoidonly_grid.csv`**: Complementary and avoid-center-only sweeps.

**Figures** (in `figures/`):

- **figure1.svg**: Transverse damage profile + degradation history (single trial) for Groups A–D.
- **figure2.svg**: Relative lifespan and annualized carbon footprint (LCA) by group.
- **figure3.svg**: Mixed-traffic: normalized damage and annualized CO₂ vs AV penetration.
- **figure4.svg**: Channelized non-AV sensitivity: damage and CO₂ vs penetration.

## LCA (Carbon Footprint)

Implemented in **`visualize.py`** only (not in C++):

- **Functional unit**: 1 km of standard lane (3.75 m width, 0.2 m depth).
- **Binder content**: Binder content in the asphalt layer is approximately 4–6% by weight of mix, with the remainder being predominantly aggregate [NCHRP Report 673, 2014].
- **Embodied carbon**: ~4–5 tons CO₂e per km (from 50 kg CO₂e/t **binder** × binder mass; binder is ~4–6% of mix, so much lower than applying a per-tonne factor to full mix); amortized over **effective lifespan** (damage-based relative life × 15 years).
- **Annualized footprint**: `CO2_PER_KM / actual_life` (tons CO₂e/km/year); used for bar charts and mixed-traffic curves.

## Citation

If you use this simulation in academic work, please cite the paper that accompanies this code (Infrastructure-Aware Trajectory Scheduling / IATS for pavement damage mitigation).

## License

See repository root or project license file.
