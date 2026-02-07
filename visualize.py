import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import pandas as pd
import numpy as np
import os

# Nature Style Settings
plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.sans-serif'] = ['Arial']
plt.rcParams['font.size'] = 7
plt.rcParams['axes.linewidth'] = 0.5
plt.rcParams['xtick.major.width'] = 0.5
plt.rcParams['ytick.major.width'] = 0.5
plt.rcParams['legend.frameon'] = False

OUTPUT_DIR = "figures/"
SIM_DIR = "results/"

if not os.path.exists(OUTPUT_DIR):
    os.makedirs(OUTPUT_DIR)

# --- LCA Calculation Function ---
# Functional unit: 1 km of standard lane (3.75 m width, 0.2 m depth).
# Binder content in the asphalt layer is approximately 4--6% by weight of mix,
# with the remainder predominantly aggregate (NCHRP Manual for Design of HMA).
def calculate_lca(lifespans, lifespans_err):
    CO2_PER_KM = 90.0  # Tons CO2e per km (embodied carbon, amortized over lifespan)
    BASELINE_LIFE = 15.0  # Years
    
    annual_co2 = []
    annual_co2_err = []
    
    for relative_life, rel_err in zip(lifespans, lifespans_err):
        actual_life = BASELINE_LIFE * relative_life
        
        # Error propagation for division: Z = A/B -> dZ/Z = dB/B
        # Annual = K / Life
        annual = CO2_PER_KM / actual_life
        err = annual * (rel_err / relative_life)
        
        annual_co2.append(annual)
        annual_co2_err.append(err)
        
    return annual_co2, annual_co2_err

# --- Figure 1: Concept & Data ---
def plot_figure1():
    fig = plt.figure(figsize=(7.2, 3.5)) 
    gs = gridspec.GridSpec(1, 2, width_ratios=[1, 1], wspace=0.25)
    
    # Panel A: Transverse Profile (Averaged over Monte Carlo)
    ax1 = plt.subplot(gs[0])
    
    files = {
        "GroupA_Precision": ("Baseline", "#d62728", "-"), 
        "GroupB_Human": ("Human", "#1f77b4", "--"), 
        "GroupC_Uniform": ("Uniform", "#2ca02c", "-."), 
        "GroupD_Feedback": ("Feedback", "black", ":")
    }
    
    stats = {}
    
    for name, (label, color, style) in files.items():
        try:
            # Load Averaged Profile
            df = pd.read_csv(f"{SIM_DIR}profile_{name}.csv", names=["pos", "damage"])
            center_idx = 375.0 / 2.0
            ax1.plot(df["pos"] - center_idx, df["damage"]/1e14, label=label, color=color, linestyle=style, linewidth=1)
            
            # Load Stats (Mean/Std)
            df_stat = pd.read_csv(f"{SIM_DIR}stats_{name}.csv")
            mean_dmg = df_stat["mean"][0]
            std_dmg = df_stat["std"][0]
            # Handle potential NaN in std (e.g. if all trials identical for Precision)
            if np.isnan(std_dmg): std_dmg = 0.0
            
            stats[name] = (mean_dmg, std_dmg)
            
        except Exception as e:
            print(f"Error reading {name}: {e}")
            stats[name] = (1.0, 0.0)

    ax1.set_xlabel("Lateral Position (cm)")
    ax1.set_ylabel("Accumulated Damage ($10^{14}$)")
    ax1.set_title("a. Transverse Damage Profile", loc='left', fontweight='bold')
    ax1.set_xlim(-180, 180)
    ax1.axvline(-180, color='gray', linewidth=0.5)
    ax1.axvline(180, color='gray', linewidth=0.5)
    ax1.text(0, ax1.get_ylim()[1]*0.55, "Lane Width (3.75m)", ha='center', fontsize=8, color='gray', fontweight='bold')
    ax1.legend(fontsize=6)

    # Panel B: Degradation History (Representative Single Trial)
    ax2 = plt.subplot(gs[1])
    for name, (label, color, style) in files.items():
        try:
            df = pd.read_csv(f"{SIM_DIR}history_{name}.csv")
            ax2.plot(df["pass"]/1000, df["max_damage"]/1e14, label=label, color=color, linestyle=style, linewidth=1)
        except Exception as e:
            print(f"Error reading history {name}: {e}")

    ax2.set_xlabel("Vehicle Passes ($10^3$)")
    ax2.set_ylabel("Peak Damage ($10^{14}$)")
    ax2.set_title("b. Degradation History (Single Trial)", loc='left', fontweight='bold')
    ax2.grid(True, linestyle=':', alpha=0.3)
    
    plt.savefig(f"{OUTPUT_DIR}figure1.svg")
    plt.close()
    
    return stats

# --- Figure 2: Impact with Error Bars ---
def plot_figure2(stats):
    fig = plt.figure(figsize=(7.2, 3.0))
    gs = gridspec.GridSpec(1, 2, width_ratios=[1, 1], wspace=0.3)
    
    baseline_mean = stats["GroupA_Precision"][0]
    # Baseline std is typically 0 for deterministic seed 42, but Monte Carlo might show small var if seeds differ? 
    # For Precision, it's very deterministic.
    
    lifespans = []
    lifespans_err = []
    
    groups_ordered = ["GroupA_Precision", "GroupB_Human", "GroupC_Uniform", "GroupD_Feedback"]
    labels = ["Baseline", "Human", "Uniform", "Feedback"]
    colors = ['#d62728', '#1f77b4', '#2ca02c', 'black']
    
    for name in groups_ordered:
        mean, std = stats[name]
        
        # Lifespan = Baseline_Damage / Damage
        # Error propagation: Z = A/B -> dZ/Z = sqrt((dA/A)^2 + (dB/B)^2)
        # Here A (Baseline) is effectively constant/reference = 1.0 relative
        # So dZ/Z = dB/B -> dZ = Z * (std / mean)
        
        rel_life = baseline_mean / mean
        rel_err = rel_life * (std / mean) if mean > 0 else 0
        
        lifespans.append(rel_life)
        lifespans_err.append(rel_err)

    # Panel A: Relative Lifespan
    ax1 = plt.subplot(gs[0])
    bars = ax1.bar(labels, lifespans, yerr=lifespans_err, capsize=3, color=colors, alpha=0.8, width=0.6, error_kw={'linewidth':0.8})
    ax1.set_ylabel("Relative Lifespan ($N_f / N_{base}$)")
    ax1.set_title("a. Infrastructure Longevity", loc='left', fontweight='bold')
    ax1.axhline(1.0, color='gray', linestyle='--', linewidth=0.5)
    
    for bar in bars:
        height = bar.get_height()
        ax1.text(bar.get_x() + bar.get_width()/2., height + 0.2, # Shift up for error bar
                f'{height:.1f}x', ha='center', va='bottom', fontsize=7)
    ylo, yhi = ax1.get_ylim()
    ax1.set_ylim(ylo, ylo + 1.2 * (yhi - ylo))

    # Panel B: Carbon Reduction
    annual_co2, annual_err = calculate_lca(lifespans, lifespans_err)
    
    ax2 = plt.subplot(gs[1])
    bars2 = ax2.bar(labels, annual_co2, yerr=annual_err, capsize=3, color=colors, alpha=0.8, width=0.6, error_kw={'linewidth':0.8})
    
    ax2.set_ylabel("Annualized Carbon Footprint\n(Tons $CO_2e$ / km / year)")
    ax2.set_title("b. Environmental Impact (LCA)", loc='left', fontweight='bold')
    
    for i, bar in enumerate(bars2):
        height = bar.get_height()
        ax2.text(bar.get_x() + bar.get_width()/2., height + 0.2,
                f'{height:.1f}', ha='center', va='bottom', fontsize=7, fontweight='bold')
    ylo, yhi = ax2.get_ylim()
    ax2.set_ylim(ylo, ylo + 1.2 * (yhi - ylo))

    plt.savefig(f"{OUTPUT_DIR}figure2.svg")
    plt.close()

def plot_figure3_mixed():
    # Prefer the grid file (contains avoid_cm sweep); fall back to mix_summary
    grid_path = f"{SIM_DIR}mix_grid_summary.csv"
    if os.path.exists(grid_path):
        df = pd.read_csv(grid_path)
        # Standardize column names
        if "avoid_cm" not in df.columns:
            df["avoid_cm"] = 0.0
    else:
        df = pd.read_csv(f"{SIM_DIR}mix_summary.csv")
        df["avoid_cm"] = 0.0

    # Normalize to human-only (p=0), which equals MixPrecision at 0%
    base = df[(df["mode"] == "MixPrecision") & (df["pct"] == 0)].iloc[0]
    base_damage = float(base["mean"])

    # Build adaptive (best) center-avoid curve from the grid sweep
    adaptive_rows = []
    fixed_rows = []
    if os.path.exists(grid_path):
        for pct in sorted(df["pct"].unique()):
            sub = df[(df["mode"] == "MixFeedbackAvoidCenterGrid") & (df["pct"] == pct)]
            if len(sub) > 0:
                best = sub.loc[sub["mean"].idxmin()]
                adaptive_rows.append(best)
                fixed = sub[np.isclose(sub["avoid_cm"], 15.0)]
                if len(fixed) > 0:
                    fixed_rows.append(fixed.iloc[0])

    adaptive_df = pd.DataFrame(adaptive_rows) if len(adaptive_rows) > 0 else None
    fixed_df = pd.DataFrame(fixed_rows) if len(fixed_rows) > 0 else None

    # Load complementary scheduling grid (lambda0 sweep) if available
    comp_path = f"{SIM_DIR}mix_complementary_grid.csv"
    comp_best = None
    if os.path.exists(comp_path):
        cdf = pd.read_csv(comp_path)
        best_rows = []
        for pct in sorted(cdf["pct"].unique()):
            sub = cdf[cdf["pct"] == pct]
            if len(sub) > 0:
                best_rows.append(sub.loc[sub["mean"].idxmin()])
        comp_best = pd.DataFrame(best_rows) if len(best_rows) > 0 else None

    # Load distribution-only center-avoid grid if available
    avoid_path = f"{SIM_DIR}mix_avoidonly_grid.csv"
    avoid_best = None
    if os.path.exists(avoid_path):
        adf = pd.read_csv(avoid_path)
        best_rows = []
        for pct in sorted(adf["pct"].unique()):
            sub = adf[adf["pct"] == pct]
            if len(sub) > 0:
                best_rows.append(sub.loc[sub["mean"].idxmin()])
        avoid_best = pd.DataFrame(best_rows) if len(best_rows) > 0 else None

    fig = plt.figure(figsize=(7.2, 3.0))
    gs = gridspec.GridSpec(1, 2, width_ratios=[1, 1], wspace=0.3)

    # Panel A: Peak damage factor vs AV penetration
    ax1 = plt.subplot(gs[0])
    for mode, label, color, ls in [
        ("MixPrecision", "Mixed: AV Precision", "#d62728", "-"),
        ("MixFeedback", "Mixed: AV Feedback", "black", "-"),
    ]:
        sub = df[df["mode"] == mode].sort_values("pct")
        p = sub["pct"].values
        dmg = sub["mean"].values / base_damage
        err = sub["std"].values / base_damage
        ax1.errorbar(p, dmg, yerr=err, label=label, color=color, linestyle=ls, linewidth=1, capsize=3)

    if adaptive_df is not None and len(adaptive_df) > 0:
        sub = adaptive_df.sort_values("pct")
        ax1.errorbar(
            sub["pct"].values,
            sub["mean"].values / base_damage,
            yerr=sub["std"].values / base_damage,
            label="Mixed: Adaptive Center-Avoid (best w)",
            color="#9467bd",
            linestyle="--",
            linewidth=1,
            capsize=3,
        )

    if fixed_df is not None and len(fixed_df) > 0:
        sub = fixed_df.sort_values("pct")
        ax1.errorbar(
            sub["pct"].values,
            sub["mean"].values / base_damage,
            yerr=sub["std"].values / base_damage,
            label="Mixed: Center-Avoid (w=15 cm)",
            color="#8c564b",
            linestyle=":",
            linewidth=1,
            capsize=3,
        )

    if comp_best is not None and len(comp_best) > 0:
        sub = comp_best.sort_values("pct")
        ax1.errorbar(
            sub["pct"].values,
            sub["mean"].values / base_damage,
            yerr=sub["std"].values / base_damage,
            label="Mixed: Complementary Feedback (best λ)",
            color="#ff7f0e",
            linestyle="-.",
            linewidth=1,
            capsize=3,
        )

    if avoid_best is not None and len(avoid_best) > 0:
        sub = avoid_best.sort_values("pct")
        ax1.errorbar(
            sub["pct"].values,
            sub["mean"].values / base_damage,
            yerr=sub["std"].values / base_damage,
            label="Mixed: Avoid-Center Only (best w, no sensing)",
            color="#2ca02c",
            linestyle="--",
            linewidth=1,
            capsize=3,
        )

    ax1.set_xlabel("AV Penetration (%)")
    ax1.set_ylabel("Normalized Peak Damage\n(relative to Human-only)")
    ax1.set_title("a. Mixed-Traffic Damage Scaling", loc="left", fontweight="bold")
    ax1.grid(True, linestyle=":", alpha=0.3)
    # Legend at bottom, 0.5 cm right of center (ax1 width ~3.6 in)
    legend_x = 1 + (0.5 / 2.54) / 3.6
    ax1.legend(fontsize=6, loc="upper center", bbox_to_anchor=(legend_x, -0.22),
               ncol=3, frameon=False)

    # Panel B: Annualized CO2 vs AV penetration (using damage-derived lifespan)
    ax2 = plt.subplot(gs[1])
    for mode, label, color, ls, src in [
        ("MixPrecision", "Mixed: AV Precision", "#d62728", "-", df[df["mode"] == "MixPrecision"].sort_values("pct")),
        ("MixFeedback", "Mixed: AV Feedback", "black", "-", df[df["mode"] == "MixFeedback"].sort_values("pct")),
    ]:
        mean = src["mean"].values
        std = src["std"].values
        life = base_damage / mean
        life_err = life * (std / mean)
        annual, annual_err = calculate_lca(life, life_err)
        ax2.errorbar(src["pct"].values, annual, yerr=annual_err, label=label, color=color, linestyle=ls, linewidth=1, capsize=3)

    if adaptive_df is not None and len(adaptive_df) > 0:
        src = adaptive_df.sort_values("pct")
        mean = src["mean"].values
        std = src["std"].values
        life = base_damage / mean
        life_err = life * (std / mean)
        annual, annual_err = calculate_lca(life, life_err)
        ax2.errorbar(src["pct"].values, annual, yerr=annual_err, label="Mixed: Adaptive Center-Avoid (best w)", color="#9467bd", linestyle="--", linewidth=1, capsize=3)

    if fixed_df is not None and len(fixed_df) > 0:
        src = fixed_df.sort_values("pct")
        mean = src["mean"].values
        std = src["std"].values
        life = base_damage / mean
        life_err = life * (std / mean)
        annual, annual_err = calculate_lca(life, life_err)
        ax2.errorbar(src["pct"].values, annual, yerr=annual_err, label="Mixed: Center-Avoid (w=15 cm)", color="#8c564b", linestyle=":", linewidth=1, capsize=3)

    if comp_best is not None and len(comp_best) > 0:
        src = comp_best.sort_values("pct")
        mean = src["mean"].values
        std = src["std"].values
        life = base_damage / mean
        life_err = life * (std / mean)
        annual, annual_err = calculate_lca(life, life_err)
        ax2.errorbar(src["pct"].values, annual, yerr=annual_err, label="Mixed: Complementary Feedback (best λ)", color="#ff7f0e", linestyle="-.", linewidth=1, capsize=3)

    if avoid_best is not None and len(avoid_best) > 0:
        src = avoid_best.sort_values("pct")
        mean = src["mean"].values
        std = src["std"].values
        life = base_damage / mean
        life_err = life * (std / mean)
        annual, annual_err = calculate_lca(life, life_err)
        ax2.errorbar(src["pct"].values, annual, yerr=annual_err, label="Mixed: Avoid-Center Only (best w)", color="#2ca02c", linestyle="--", linewidth=1, capsize=3)

    ax2.set_xlabel("AV Penetration (%)")
    ax2.set_ylabel("Annualized Carbon Footprint\n(Tons $CO_2e$ / km / year)")
    ax2.set_title("b. Deployment-Phase Sustainability", loc="left", fontweight="bold")
    ax2.grid(True, linestyle=":", alpha=0.3)

    plt.tight_layout()
    plt.subplots_adjust(bottom=0.28)
    plt.savefig(f"{OUTPUT_DIR}figure3.svg", bbox_inches="tight")
    plt.close()

def plot_figure4_channelized():
    path = f"{SIM_DIR}mix_grid_summary_channelized.csv"
    if not os.path.exists(path):
        return

    df = pd.read_csv(path)

    base = df[(df["mode"] == "MixPrecision") & (df["pct"] == 0)].iloc[0]
    base_damage = float(base["mean"])

    # Adaptive best avoid width per penetration
    adaptive_rows = []
    fixed_rows = []
    for pct in sorted(df["pct"].unique()):
        sub = df[(df["mode"] == "MixFeedbackAvoidCenterGrid") & (df["pct"] == pct)]
        if len(sub) > 0:
            best = sub.loc[sub["mean"].idxmin()]
            adaptive_rows.append(best)
            fixed = sub[np.isclose(sub["avoid_cm"], 15.0)]
            if len(fixed) > 0:
                fixed_rows.append(fixed.iloc[0])

    adaptive_df = pd.DataFrame(adaptive_rows) if len(adaptive_rows) > 0 else None
    fixed_df = pd.DataFrame(fixed_rows) if len(fixed_rows) > 0 else None

    fig = plt.figure(figsize=(7.2, 3.0))
    gs = gridspec.GridSpec(1, 2, width_ratios=[1, 1], wspace=0.3)

    ax1 = plt.subplot(gs[0])
    for mode, label, color, ls in [
        ("MixPrecision", "Channelized non-AV (σ=10 cm): AV Precision", "#d62728", "-"),
        ("MixFeedback", "Channelized non-AV (σ=10 cm): AV Feedback", "black", "-"),
    ]:
        sub = df[df["mode"] == mode].sort_values("pct")
        ax1.errorbar(
            sub["pct"].values,
            sub["mean"].values / base_damage,
            yerr=sub["std"].values / base_damage,
            label=label,
            color=color,
            linestyle=ls,
            linewidth=1,
            capsize=3,
        )

    if adaptive_df is not None and len(adaptive_df) > 0:
        sub = adaptive_df.sort_values("pct")
        ax1.errorbar(
            sub["pct"].values,
            sub["mean"].values / base_damage,
            yerr=sub["std"].values / base_damage,
            label="Channelized non-AV: Adaptive Center-Avoid (best w)",
            color="#9467bd",
            linestyle="--",
            linewidth=1,
            capsize=3,
        )

    if fixed_df is not None and len(fixed_df) > 0:
        sub = fixed_df.sort_values("pct")
        ax1.errorbar(
            sub["pct"].values,
            sub["mean"].values / base_damage,
            yerr=sub["std"].values / base_damage,
            label="Channelized non-AV: Center-Avoid (w=15 cm)",
            color="#8c564b",
            linestyle=":",
            linewidth=1,
            capsize=3,
        )

    ax1.set_xlabel("AV Penetration (%)")
    ax1.set_ylabel("Normalized Peak Damage\n(relative to channelized non-AV only)")
    ax1.set_title("a. Channelized Non-AV Sensitivity", loc="left", fontweight="bold")
    ax1.grid(True, linestyle=":", alpha=0.3)
    ax1.legend(fontsize=6, loc="upper center", bbox_to_anchor=(0.8, -0.22), ncol=2, frameon=False)

    ax2 = plt.subplot(gs[1])
    for mode, label, color, ls, src in [
        ("MixPrecision", "AV Precision", "#d62728", "-", df[df["mode"] == "MixPrecision"].sort_values("pct")),
        ("MixFeedback", "AV Feedback", "black", "-", df[df["mode"] == "MixFeedback"].sort_values("pct")),
    ]:
        mean = src["mean"].values
        std = src["std"].values
        life = base_damage / mean
        life_err = life * (std / mean)
        annual, annual_err = calculate_lca(life, life_err)
        ax2.errorbar(src["pct"].values, annual, yerr=annual_err, label=label, color=color, linestyle=ls, linewidth=1, capsize=3)

    if adaptive_df is not None and len(adaptive_df) > 0:
        src = adaptive_df.sort_values("pct")
        mean = src["mean"].values
        std = src["std"].values
        life = base_damage / mean
        life_err = life * (std / mean)
        annual, annual_err = calculate_lca(life, life_err)
        ax2.errorbar(src["pct"].values, annual, yerr=annual_err, label="Adaptive Center-Avoid (best w)", color="#9467bd", linestyle="--", linewidth=1, capsize=3)

    ax2.set_xlabel("AV Penetration (%)")
    ax2.set_ylabel("Annualized Carbon Footprint\n(Tons $CO_2e$ / km / year)")
    ax2.set_title("b. Channelized Deployment Impact", loc="left", fontweight="bold")
    ax2.grid(True, linestyle=":", alpha=0.3)

    plt.tight_layout()
    plt.subplots_adjust(bottom=0.28)
    plt.savefig(f"{OUTPUT_DIR}figure4.svg", bbox_inches="tight")
    plt.close()

if __name__ == "__main__":
    stats = plot_figure1()
    plot_figure2(stats)
    plot_figure3_mixed()
    plot_figure4_channelized()
    print("Nature-style figures with Error Bars generated.")
