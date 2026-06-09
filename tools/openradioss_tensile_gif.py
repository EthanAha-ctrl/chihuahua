#!/usr/bin/env python3
"""Generate a GIF from the OpenRadioss tensile LAW2 first-light example.

The mesh comes from the example Starter deck. The deformation is a visual
proxy driven by T01 time-history displacement, force, and energy outputs; it is
not a full H3D nodal displacement reconstruction.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
import numpy as np
from PIL import Image


def ints_from_line(line: str) -> list[int] | None:
    vals = []
    for tok in line.split():
        try:
            vals.append(int(tok))
        except ValueError:
            return None
    return vals


def parse_rad_mesh(path: Path) -> tuple[dict[int, tuple[float, float, float]], list[list[int]]]:
    nodes: dict[int, tuple[float, float, float]] = {}
    elements: list[list[int]] = []
    section: str | None = None

    for raw in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or line.startswith("##"):
            continue
        if line.startswith("/NODE"):
            section = "NODE"
            continue
        if line.startswith("/SHELL/"):
            section = "SHELL"
            continue
        if line.startswith("/SH3N/"):
            section = "SH3N"
            continue
        if line.startswith("/"):
            section = None
            continue

        if section == "NODE":
            parts = line.split()
            if len(parts) < 4:
                continue
            try:
                nodes[int(parts[0])] = (float(parts[1]), float(parts[2]), float(parts[3]))
            except ValueError:
                continue
        elif section == "SHELL":
            vals = ints_from_line(line)
            if vals and len(vals) >= 5:
                elements.append(vals[1:5])
        elif section == "SH3N":
            vals = ints_from_line(line)
            if vals and len(vals) >= 4:
                elements.append(vals[1:4])

    if not nodes or not elements:
        raise ValueError(f"failed to parse mesh from {path}")
    return nodes, elements


def find_column(headers: list[str], required: tuple[str, ...]) -> str:
    matches = [header for header in headers if all(part in header for part in required)]
    if not matches:
        raise ValueError(f"no CSV column matched {required}")
    return matches[0]


def load_time_history(path: Path) -> dict[str, np.ndarray]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        rows = list(reader)

    if not rows:
        raise ValueError(f"no rows in {path}")

    headers = list(rows[0])
    disp_col = find_column(headers, ("616", "var 29"))
    force_col = find_column(headers, ("Rigid_Body", "var 35"))

    return {
        "time": np.array([float(row["time"]) for row in rows]),
        "internal": np.array([float(row["INTERNAL ENERGY"]) for row in rows]),
        "kinetic": np.array([float(row["KINETIC ENERGY"]) for row in rows]),
        "external": np.array([float(row["EXTERNAL WORK"]) for row in rows]),
        "plastic": np.array([float(row["PLASTIC WORK"]) for row in rows]),
        "dx": np.array([float(row[disp_col]) for row in rows]),
        "reaction_x": -np.array([float(row[force_col]) for row in rows]),
    }


def make_gif(run_dir: Path, output: Path, poster: Path, frame_count: int, duration_ms: int) -> None:
    rad_path = run_dir / "tensile_LAW2_0000.rad"
    csv_path = run_dir / "tensile_LAW2T01.csv"
    nodes, elements = parse_rad_mesh(rad_path)
    history = load_time_history(csv_path)

    node_ids = np.array(sorted(nodes), dtype=int)
    coords = np.array([nodes[int(nid)] for nid in node_ids], dtype=float)
    id_to_idx = {int(nid): i for i, nid in enumerate(node_ids)}
    polys_idx = [[id_to_idx[nid] for nid in elem] for elem in elements if all(nid in id_to_idx for nid in elem)]

    time = history["time"]
    dx = history["dx"]
    reaction_x = history["reaction_x"]
    internal = history["internal"]
    kinetic = history["kinetic"]
    external = history["external"]
    plastic = history["plastic"]

    sample_idx = np.linspace(0, len(time) - 1, frame_count).round().astype(int)
    x0 = coords[:, 0]
    y0 = coords[:, 1]
    x_min = float(x0.min())
    x_max = float(x0.max())
    y_center = 0.5 * (float(y0.min()) + float(y0.max()))
    gauge_left = 60.0
    gauge_right = 140.0
    gauge_len = gauge_right - gauge_left

    centers_x = np.array([coords[idxs, 0].mean() for idxs in polys_idx])
    neck_weight = np.exp(-((centers_x - 100.0) / 42.0) ** 2)
    neck_weight = (neck_weight - neck_weight.min()) / max(1e-12, neck_weight.max() - neck_weight.min())

    ux_max = max(1.0, float(np.nanmax(np.abs(dx))))
    force_max = max(1.0, float(np.nanmax(np.abs(reaction_x))))
    energy_max = max(1.0, float(np.nanmax([internal.max(), external.max(), plastic.max()])))

    frames: list[Image.Image] = []
    for idx in sample_idx:
        t = time[idx]
        u = dx[idx]
        frac = min(1.0, abs(u) / ux_max)

        span = np.clip((x0 - gauge_left) / gauge_len, 0.0, 1.15)
        x_def = x0 + u * (span**1.12)
        strain = abs(u) / gauge_len
        lateral_zone = np.clip((x0 - 15.0) / 155.0, 0.0, 1.0)
        neck_zone = 0.25 + 0.75 * np.exp(-((x0 - 100.0) / 48.0) ** 2)
        contraction = np.clip(1.0 - 0.30 * strain * lateral_zone * neck_zone, 0.72, 1.0)
        y_def = y_center + (y0 - y_center) * contraction
        xy = np.column_stack([x_def, y_def])
        polys = [xy[idxs] for idxs in polys_idx]
        colors = 0.18 + 0.82 * frac * neck_weight

        fig = plt.figure(figsize=(12.8, 7.2), dpi=110)
        gs = fig.add_gridspec(2, 2, height_ratios=[2.25, 1.0], hspace=0.32, wspace=0.25)
        ax_mesh = fig.add_subplot(gs[0, :])
        ax_force = fig.add_subplot(gs[1, 0])
        ax_energy = fig.add_subplot(gs[1, 1])

        ghost = PolyCollection(
            [coords[idxs, :2] for idxs in polys_idx],
            facecolors=(0.55, 0.60, 0.66, 0.10),
            edgecolors=(0.25, 0.28, 0.32, 0.08),
            linewidths=0.16,
        )
        ax_mesh.add_collection(ghost)
        coll = PolyCollection(
            polys,
            array=colors,
            cmap="inferno",
            edgecolors=(0.09, 0.10, 0.12, 0.48),
            linewidths=0.24,
            antialiased=True,
        )
        coll.set_clim(0, 1)
        ax_mesh.add_collection(coll)

        ax_mesh.axvspan(x_min - 4, gauge_left, color="#2c7fb8", alpha=0.08, lw=0)
        ax_mesh.axvline(gauge_left, color="#2c7fb8", lw=1.0, alpha=0.55)
        ax_mesh.annotate(
            "",
            xy=(x_max + u + 12, y_center),
            xytext=(x_max + u - 18, y_center),
            arrowprops=dict(arrowstyle="-|>", lw=3, color="#d94801", shrinkA=0, shrinkB=0),
        )
        ax_mesh.text(x_min, y0.max() + 16, "fixed side", color="#2c7fb8", fontsize=10, weight="bold")
        ax_mesh.text(x_max + u - 12, y0.max() + 16, "pull", color="#d94801", fontsize=10, weight="bold")
        ax_mesh.set_title(
            f"OpenRadioss tensile_LAW2 | t={t:5.2f} ms | dx={u:5.2f} mm | reaction={reaction_x[idx]:6.1f} N",
            fontsize=12,
            pad=10,
        )
        ax_mesh.set_aspect("equal", adjustable="box")
        ax_mesh.set_xlim(x_min - 8, x_max + ux_max + 18)
        ax_mesh.set_ylim(y0.min() - 20, y0.max() + 24)
        ax_mesh.set_xlabel("x [mm]")
        ax_mesh.set_ylabel("y [mm]")
        ax_mesh.grid(color="#d9d9d9", linewidth=0.5, alpha=0.45)
        ax_mesh.text(
            0.995,
            0.02,
            "mesh from .rad, deformation proxy from T01 time history",
            ha="right",
            va="bottom",
            transform=ax_mesh.transAxes,
            fontsize=8.5,
            color="#555555",
        )

        ax_force.plot(dx, reaction_x, color="#252525", lw=1.6)
        ax_force.scatter([dx[idx]], [reaction_x[idx]], color="#d94801", s=42, zorder=4)
        ax_force.set_title("Reaction vs displacement")
        ax_force.set_xlabel("dx node 616 wrt node 102 [mm]")
        ax_force.set_ylabel("reaction X [N]")
        ax_force.set_xlim(0, ux_max * 1.03)
        ax_force.set_ylim(min(-5, float(reaction_x.min()) * 1.05), force_max * 1.08)
        ax_force.grid(color="#d9d9d9", linewidth=0.5, alpha=0.6)

        ax_energy.plot(time, external, color="#1f78b4", lw=1.4, label="external work")
        ax_energy.plot(time, internal, color="#33a02c", lw=1.4, label="internal energy")
        ax_energy.plot(time, plastic, color="#e31a1c", lw=1.4, label="plastic work")
        ax_energy.plot(time, kinetic, color="#6a3d9a", lw=1.2, label="kinetic")
        ax_energy.axvline(t, color="#d94801", lw=1.4, alpha=0.8)
        ax_energy.set_title("Energy history")
        ax_energy.set_xlabel("time [ms]")
        ax_energy.set_ylabel("energy [N mm]")
        ax_energy.set_xlim(0, float(time.max()))
        ax_energy.set_ylim(0, energy_max * 1.08)
        ax_energy.grid(color="#d9d9d9", linewidth=0.5, alpha=0.6)
        ax_energy.legend(loc="upper left", fontsize=8, frameon=False)

        fig.patch.set_facecolor("#fbfbf8")
        for ax in (ax_mesh, ax_force, ax_energy):
            ax.set_facecolor("#fbfbf8")
            for spine in ax.spines.values():
                spine.set_color("#bdbdbd")

        fig.canvas.draw()
        frame = Image.fromarray(np.asarray(fig.canvas.buffer_rgba())).convert("RGB")
        frames.append(frame)
        plt.close(fig)

    poster.parent.mkdir(parents=True, exist_ok=True)
    output.parent.mkdir(parents=True, exist_ok=True)
    frames[-1].save(poster)
    all_frames = [frames[0]] * 4 + frames + [frames[-1]] * 10
    all_frames[0].save(
        output,
        save_all=True,
        append_images=all_frames[1:],
        duration=duration_ms,
        loop=0,
        optimize=True,
    )

    print(f"Wrote {output}")
    print(f"Wrote {poster}")
    print(f"nodes={len(nodes)} elements={len(elements)} frames={len(frames)}")
    print(
        f"final dx={dx[-1]:.3f} mm final reaction={reaction_x[-1]:.3f} N "
        f"final plastic work={plastic[-1]:.3f} Nmm"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--run-dir",
        type=Path,
        default=Path("/mnt/s8t/openradioss/runs/tensile_section1/1_LAW2"),
        help="directory containing tensile_LAW2_0000.rad and tensile_LAW2T01.csv",
    )
    parser.add_argument("--frames", type=int, default=72)
    parser.add_argument("--duration-ms", type=int, default=55)
    parser.add_argument("--out", type=Path, default=None, help="GIF output path")
    parser.add_argument("--poster", type=Path, default=None, help="final-frame PNG output path")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    out_dir = args.run_dir / "visualization"
    output = args.out or out_dir / "openradioss_tensile_LAW2.gif"
    poster = args.poster or out_dir / "openradioss_tensile_LAW2_poster.png"
    make_gif(args.run_dir, output, poster, args.frames, args.duration_ms)


if __name__ == "__main__":
    main()
