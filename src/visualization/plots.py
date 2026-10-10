"""Matplotlib report figures saved after every run."""
from __future__ import annotations

import matplotlib

matplotlib.use("Agg")  # render to files, no window needed
import matplotlib.pyplot as plt
import numpy as np

from src.simulation.world import World

STATE_ORDER = ["PLANNING", "FOLLOWING", "EMERGENCY_STOP", "RECOVERY", "GOAL_REACHED", "FAILED"]


def plot_trajectory(world: World, trajectory: np.ndarray, first_plan, out_path: str, title: str) -> None:
    fig, ax = plt.subplots(figsize=(11, 7))
    ax.imshow(world.static_mask, cmap="Greys", origin="upper",
              extent=(0, world.width, world.height, 0), alpha=0.85)
    for obs in world.dynamic_obstacles:
        wp = np.array(obs.waypoints)
        ax.plot(wp[:, 0], wp[:, 1], ":", color="tab:red", lw=1.5)
    ax.plot([], [], ":", color="tab:red", label="moving obstacle route")
    if first_plan is not None:
        ax.plot(first_plan[:, 0], first_plan[:, 1], "--", color="tab:blue", lw=1.5,
                label="initial plan (map unknown)")
    if len(trajectory):
        ax.plot(trajectory[:, 0], trajectory[:, 1], "-", color="tab:orange", lw=2.5, label="driven trajectory")
    ax.plot(*world.start[:2], "o", color="black", ms=9, label="start")
    ax.plot(*world.goal, "*", color="tab:green", ms=18, label="goal")
    ax.set_xlim(0, world.width)
    ax.set_ylim(world.height, 0)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(title)
    fig.legend(loc="lower center", ncol=5, fontsize=9, frameon=False)
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def plot_telemetry(t, speed, steer_deg, clearance, states, out_path: str, max_speed: float) -> None:
    fig, axes = plt.subplots(4, 1, figsize=(11, 9), sharex=True)
    axes[0].plot(t, speed, color="tab:green")
    axes[0].axhline(max_speed, ls="--", color="grey", lw=1)
    axes[0].set_ylabel("speed [m/s]")
    axes[1].plot(t, steer_deg, color="tab:purple")
    axes[1].set_ylabel("steering [deg]")
    axes[2].plot(t, clearance, color="tab:blue")
    axes[2].axhline(0, color="tab:red", lw=1)
    axes[2].set_ylabel("clearance [m]")
    codes = [STATE_ORDER.index(s) for s in states]
    axes[3].step(t, codes, where="post", color="tab:orange")
    axes[3].set_yticks(range(len(STATE_ORDER)))
    axes[3].set_yticklabels(STATE_ORDER, fontsize=8)
    axes[3].set_xlabel("time [s]")
    for ax in axes:
        ax.grid(alpha=0.3)
    fig.suptitle("Run telemetry")
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)
