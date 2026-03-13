#!/usr/bin/env python3
"""Plot all experiments from the experiments folder."""

import glob
import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

EXP_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(EXP_DIR, "plots")
os.makedirs(OUT_DIR, exist_ok=True)

# Collect all experiment timestamps
timestamps = sorted(set(
    f.split("_", 1)[1].replace(".csv", "")
    for f in os.listdir(EXP_DIR) if f.endswith(".csv")
))

joint_labels = [f"J{i+1}" for i in range(6)]

for ts in timestamps:
    teleop_file = os.path.join(EXP_DIR, f"teleop_{ts}.csv")
    collision_file = os.path.join(EXP_DIR, f"collisions_{ts}.csv")

    if not os.path.exists(teleop_file):
        continue

    df = pd.read_csv(teleop_file)
    df["t"] = df["timestamp"] - df["timestamp"].iloc[0]

    fig, axes = plt.subplots(4, 2, figsize=(18, 20), sharex=True)
    fig.suptitle(f"Experiment {ts}", fontsize=16, fontweight="bold")

    # 1) Master vs Slave positions per joint
    ax = axes[0, 0]
    for j in range(6):
        ax.plot(df["t"], df[f"master_pos_j{j+1}"], label=f"Master J{j+1}", linestyle="--", alpha=0.7)
        ax.plot(df["t"], df[f"slave_pos_j{j+1}"], label=f"Slave J{j+1}", alpha=0.7)
    ax.set_ylabel("Position (rad)")
    ax.set_title("Master vs Slave Joint Positions")
    ax.legend(fontsize=6, ncol=4)
    ax.grid(True, alpha=0.3)

    # 2) Position error
    ax = axes[0, 1]
    for j in range(6):
        ax.plot(df["t"], df[f"q_err_j{j+1}"], label=f"J{j+1}")
    ax.set_ylabel("Error (rad)")
    ax.set_title("Joint Position Error (Master - Slave)")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # 3) Slave effort
    ax = axes[1, 0]
    for j in range(6):
        ax.plot(df["t"], df[f"slave_effort_j{j+1}"], label=f"J{j+1}")
    ax.set_ylabel("Torque (Nm)")
    ax.set_title("Slave Joint Efforts")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # 4) Slave external torque
    ax = axes[1, 1]
    for j in range(6):
        ax.plot(df["t"], df[f"slave_ext_torque_j{j+1}"], label=f"J{j+1}")
    ax.set_ylabel("Torque (Nm)")
    ax.set_title("Slave External Torques")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # 5) Reflected torques
    ax = axes[2, 0]
    for j in range(6):
        ax.plot(df["t"], df[f"reflected_tau_j{j+1}"], label=f"J{j+1}")
    ax.set_ylabel("Torque (Nm)")
    ax.set_title("Reflected Torques (Feedback to Master)")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # 6) Force sensor + contact
    ax = axes[2, 1]
    ax.plot(df["t"], df["force_sensor"], label="Force Sensor", color="blue")
    ax.set_ylabel("Force Sensor (raw)", color="blue")
    ax.set_title("Force Sensor & Contact Detection")
    ax.grid(True, alpha=0.3)
    ax2 = ax.twinx()
    ax2.fill_between(df["t"], df["in_contact"], alpha=0.3, color="red", label="In Contact")
    ax2.fill_between(df["t"], df["effort_collision"], alpha=0.3, color="orange", label="Effort Collision")
    ax2.set_ylabel("Contact/Collision Flag")
    ax2.set_ylim(-0.1, 1.5)
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, fontsize=7)

    # 7) Master efforts
    ax = axes[3, 0]
    for j in range(6):
        ax.plot(df["t"], df[f"master_effort_j{j+1}"], label=f"J{j+1}")
    ax.set_ylabel("Torque (Nm)")
    ax.set_xlabel("Time (s)")
    ax.set_title("Master Joint Efforts")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # 8) Collisions timeline
    ax = axes[3, 1]
    ax.set_xlabel("Time (s)")
    if os.path.exists(collision_file):
        dfc = pd.read_csv(collision_file)
        if len(dfc) > 0:
            dfc["t"] = dfc["timestamp"] - df["timestamp"].iloc[0]
            for eid in dfc["event_id"].unique():
                ev = dfc[dfc["event_id"] == eid]
                trigger = ev["trigger"].iloc[0]
                color = "red" if trigger == "effort" else "blue"
                ax.axvspan(ev["t"].iloc[0], ev["t"].iloc[-1], alpha=0.3, color=color, label=f"#{eid} ({trigger})")
                # Plot force sensor during collision
                ax.plot(ev["t"], ev["force_sensor_raw"], color=color, alpha=0.7)
            ax.set_title("Collision Events")
            ax.set_ylabel("Force Sensor (raw)")
            handles, labels_leg = ax.get_legend_handles_labels()
            by_label = dict(zip(labels_leg, handles))
            ax.legend(by_label.values(), by_label.keys(), fontsize=6)
            ax.grid(True, alpha=0.3)
        else:
            ax.text(0.5, 0.5, "No collisions", ha="center", va="center", transform=ax.transAxes)
            ax.set_title("Collision Events")
    else:
        ax.text(0.5, 0.5, "No collision file", ha="center", va="center", transform=ax.transAxes)
        ax.set_title("Collision Events")

    plt.tight_layout()
    out_path = os.path.join(OUT_DIR, f"experiment_{ts}.png")
    plt.savefig(out_path, dpi=150)
    plt.close()
    print(f"Saved: {out_path}")

print(f"\nAll plots saved to {OUT_DIR}/")
