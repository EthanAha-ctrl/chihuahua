#!/usr/bin/env python3
"""Generate a Kattappa-readable dashboard for the whole-body beam smoke run.

The upper panel shows the OpenRadioss beam deck. The lower panels show T01 time
history. This script does not invent deformation: if the run has no load case,
the dashboard shows zero energy and no displacement claim.
"""

from __future__ import annotations

import argparse
import csv
import io
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image


def parse_beam_deck(path: Path) -> tuple[dict[int, np.ndarray], list[tuple[int, int, int]], dict[int, float]]:
    nodes: dict[int, np.ndarray] = {}
    beams: list[tuple[int, int, int]] = []
    admas: dict[int, float] = {}
    section: str | None = None

    for raw in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if line == "/NODE":
            section = "NODE"
            continue
        if line.startswith("/BEAM/"):
            section = "BEAM"
            continue
        if line.startswith("/ADMAS/5/"):
            section = "ADMAS"
            continue
        if line.startswith("/"):
            section = None
            continue

        parts = line.split()
        if section == "NODE" and len(parts) >= 4:
            try:
                nodes[int(parts[0])] = np.array([float(parts[1]), float(parts[2]), float(parts[3])], dtype=float)
            except ValueError:
                continue
        elif section == "BEAM" and len(parts) >= 3:
            try:
                beams.append((int(parts[0]), int(parts[1]), int(parts[2])))
            except ValueError:
                continue
            section = None
        elif section == "ADMAS" and len(parts) >= 2:
            try:
                admas[int(parts[1])] = admas.get(int(parts[1]), 0.0) + float(parts[0])
            except ValueError:
                continue

    if not nodes or not beams:
        raise ValueError(f"failed to parse beam deck from {path}")
    return nodes, beams, admas


def load_t01_csv(path: Path) -> dict[str, np.ndarray]:
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError(f"no rows in {path}")

    def col(name: str) -> np.ndarray:
        return np.array([float(row[name]) for row in rows], dtype=float)

    return {
        "time": col("time"),
        "mass": col("MASS"),
        "dt": col("TIME STEP"),
        "internal": col("INTERNAL ENERGY"),
        "kinetic": col("KINETIC ENERGY"),
        "external": col("EXTERNAL WORK"),
        "plastic": col("PLASTIC WORK"),
        "contact": col("CONTACT ENERGY"),
    }


def set_equal_3d(ax: object, coords: np.ndarray) -> None:
    mins = coords.min(axis=0)
    maxs = coords.max(axis=0)
    center = 0.5 * (mins + maxs)
    span = max(float((maxs - mins).max()), 1.0)
    radius = 0.58 * span
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(max(0.0, center[2] - radius * 0.72), center[2] + radius * 0.84)
    try:
        ax.set_box_aspect((1, 1, 0.60), zoom=1.23)
    except TypeError:
        ax.set_box_aspect((1, 1, 0.60))


def draw_mesh_panel(ax: object, nodes: dict[int, np.ndarray], beams: list[tuple[int, int, int]], admas: dict[int, float]) -> None:
    coords = np.array(list(nodes.values()), dtype=float)
    min_z = float(coords[:, 2].min())
    contact_ids = [nid for nid, xyz in nodes.items() if abs(float(xyz[2]) - min_z) < 1.0e-6]

    for _beam_id, n1, n2 in beams:
        a = nodes[n1]
        b = nodes[n2]
        ax.plot(
            [a[0], b[0]],
            [a[1], b[1]],
            [a[2], b[2]],
            color="#7a8796",
            linewidth=2.4,
            alpha=0.92,
        )

    ax.scatter(coords[:, 0], coords[:, 1], coords[:, 2], s=18, c="#f8fafc", edgecolors="#111827", linewidths=0.6)

    if contact_ids:
        contact = np.array([nodes[nid] for nid in contact_ids], dtype=float)
        ax.scatter(
            contact[:, 0],
            contact[:, 1],
            contact[:, 2],
            marker="v",
            s=76,
            c="none",
            edgecolors="#f97316",
            linewidths=1.3,
            label="toe/contact candidate, not fixed",
        )

    if admas:
        mass_ids = np.array(sorted(admas), dtype=int)
        mass_coords = np.array([nodes[int(nid)] for nid in mass_ids], dtype=float)
        masses = np.array([admas[int(nid)] for nid in mass_ids], dtype=float)
        sizes = 22.0 + 320.0 * masses / max(float(masses.max()), 1.0e-12)
        ax.scatter(
            mass_coords[:, 0],
            mass_coords[:, 1],
            mass_coords[:, 2],
            marker="D",
            s=sizes,
            c="#fb7185",
            edgecolors="#881337",
            linewidths=0.8,
            alpha=0.62,
            label="ADMAS lumped mass nodes",
        )

    ax.text2D(0.02, 0.93, "NO GRAVITY", transform=ax.transAxes, color="#b91c1c", fontsize=10, weight="bold")
    ax.text2D(0.18, 0.93, "NO FIXED FEET", transform=ax.transAxes, color="#b91c1c", fontsize=10, weight="bold")
    ax.text2D(0.37, 0.93, "NO SUPPORT REACTION", transform=ax.transAxes, color="#b91c1c", fontsize=10, weight="bold")
    ax.text2D(0.68, 0.93, "NO LOAD CASE", transform=ax.transAxes, color="#b91c1c", fontsize=10, weight="bold")
    ax.text2D(
        0.99,
        0.03,
        "beam mesh from .rad; smoke-run history from T01.csv",
        transform=ax.transAxes,
        ha="right",
        va="bottom",
        fontsize=8.5,
        color="#555555",
    )

    ax.set_xlabel("x [mm]")
    ax.set_ylabel("y [mm]")
    ax.set_zlabel("z [mm]")
    ax.view_init(elev=27.0, azim=-42.0)
    ax.grid(color="#d9d9d9", linewidth=0.45, alpha=0.35)
    set_equal_3d(ax, coords)


def render_frame(
    nodes: dict[int, np.ndarray],
    beams: list[tuple[int, int, int]],
    admas: dict[int, float],
    history: dict[str, np.ndarray],
    idx: int,
) -> Image.Image:
    time = history["time"]
    mass = history["mass"]
    dt = history["dt"]
    internal = history["internal"]
    kinetic = history["kinetic"]
    external = history["external"]
    plastic = history["plastic"]
    contact = history["contact"]

    fig = plt.figure(figsize=(12.8, 7.2), dpi=110)
    fig.patch.set_facecolor("#fbfbf8")
    gs = fig.add_gridspec(2, 2, height_ratios=[2.25, 1.0], hspace=0.35, wspace=0.26)
    ax_mesh = fig.add_subplot(gs[0, :], projection="3d")
    ax_mass = fig.add_subplot(gs[1, 0])
    ax_energy = fig.add_subplot(gs[1, 1])

    draw_mesh_panel(ax_mesh, nodes, beams, admas)
    ax_mesh.set_title(
        "OpenRadioss whole_body_beam | "
        f"t={time[idx]:.5f} ms | mass={mass[idx]:.6f} kg | dt={dt[idx]:.5f} ms | no load",
        fontsize=12,
        pad=10,
    )

    ax_mass.plot(time, mass, color="#252525", lw=1.6, label="mass")
    ax_mass.scatter([time[idx]], [mass[idx]], color="#d94801", s=42, zorder=4)
    ax_mass.set_title("Mass / time-step smoke check")
    ax_mass.set_xlabel("time [ms]")
    ax_mass.set_ylabel("mass [kg]")
    mass_pad = max(1.0e-6, abs(float(mass.max() - mass.min())) * 0.8)
    ax_mass.set_ylim(float(mass.min()) - mass_pad, float(mass.max()) + mass_pad)
    ax_dt = ax_mass.twinx()
    ax_dt.plot(time, dt, color="#1f78b4", lw=1.2, label="time step")
    ax_dt.set_ylabel("time step [ms]", color="#1f78b4")
    ax_dt.tick_params(axis="y", labelcolor="#1f78b4")
    ax_mass.grid(color="#d9d9d9", linewidth=0.5, alpha=0.6)

    ax_energy.plot(time, external, color="#1f78b4", lw=1.4, label="external work")
    ax_energy.plot(time, internal, color="#33a02c", lw=1.4, label="internal energy")
    ax_energy.plot(time, plastic, color="#e31a1c", lw=1.4, label="plastic work")
    ax_energy.plot(time, kinetic, color="#6a3d9a", lw=1.2, label="kinetic")
    ax_energy.plot(time, contact, color="#ff7f00", lw=1.0, label="contact")
    ax_energy.axvline(time[idx], color="#d94801", lw=1.4, alpha=0.8)
    ax_energy.set_title("Energy history")
    ax_energy.set_xlabel("time [ms]")
    ax_energy.set_ylabel("energy [N mm]")
    energy_max = max(1.0, float(np.nanmax([external.max(), internal.max(), plastic.max(), kinetic.max(), contact.max()])))
    ax_energy.set_ylim(-0.05 * energy_max, 1.05 * energy_max)
    ax_energy.grid(color="#d9d9d9", linewidth=0.5, alpha=0.6)
    ax_energy.legend(loc="upper left", fontsize=8, frameon=False)

    fig.tight_layout()
    buffer = io.BytesIO()
    fig.savefig(buffer, format="png", facecolor=fig.get_facecolor(), bbox_inches="tight", pad_inches=0.15)
    plt.close(fig)
    buffer.seek(0)
    return Image.open(buffer).convert("RGB")


def make_dashboard(run_dir: Path, output: Path, poster: Path, frame_count: int, duration_ms: int) -> None:
    rad_path = run_dir / "stage2_whole_body_beam_0000.rad"
    csv_path = run_dir / "stage2_whole_body_beamT01.csv"
    nodes, beams, admas = parse_beam_deck(rad_path)
    history = load_t01_csv(csv_path)

    output.parent.mkdir(parents=True, exist_ok=True)
    sample_idx = np.linspace(0, len(history["time"]) - 1, frame_count).round().astype(int)
    images = [render_frame(nodes, beams, admas, history, int(idx)) for idx in sample_idx]
    poster_image = render_frame(nodes, beams, admas, history, int(sample_idx[-1]))
    poster_image.save(poster)
    images[0].save(output, save_all=True, append_images=images[1:], duration=duration_ms, loop=0)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=24)
    parser.add_argument("--duration-ms", type=int, default=120)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--poster", type=Path, default=None)
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    out_dir = args.run_dir / "visualization"
    output = args.output or out_dir / "openradioss_whole_body_beam_dashboard.gif"
    poster = args.poster or out_dir / "openradioss_whole_body_beam_dashboard_poster.png"
    make_dashboard(args.run_dir, output, poster, max(1, args.frames), max(20, args.duration_ms))
    print(f"wrote {output}")
    print(f"wrote {poster}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
