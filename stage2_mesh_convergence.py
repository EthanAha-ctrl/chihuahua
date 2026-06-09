#!/usr/bin/env python3
"""Run Stage 2 OpenRadioss mesh convergence for the whole-body beam case."""

from __future__ import annotations

import argparse
import csv
import os
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

from dog_description import DEFAULT_DESCRIPTION_PATH, load_dog_description
import mass_model as stage1
import stage2_openradioss_periodic_motion as periodic


DEFAULT_OPENRADIOSS_PATH = Path("/mnt/s8t/openradioss/latest-20260520/OpenRadioss")
DEFAULT_OUT_ROOT = Path("/mnt/s8t/openradioss/runs/chihuahua_stage2_mesh_convergence_radius12_native_beam")


@dataclass(frozen=True)
class ConvergenceResult:
    element_length_mm: float
    run_dir: Path
    solver_nodes: int
    beam_elements: int
    beam_resultant_elements: int
    starter_ok: bool
    engine_ok: bool
    engine_cycles: int
    frame_index: int
    time_ms: float
    max_displacement_mm: float
    max_abs_strain: float
    max_abs_stress_mpa: float
    hottest_element: str


def run_command(command: list[str], cwd: Path, env: dict[str, str], log_path: Path) -> None:
    with log_path.open("w", encoding="utf-8") as handle:
        proc = subprocess.run(command, cwd=cwd, env=env, stdout=handle, stderr=subprocess.STDOUT, text=True)
    if proc.returncode != 0:
        raise RuntimeError(f"command failed ({proc.returncode}): {' '.join(command)}; see {log_path}")


def openradioss_env(openradioss_path: Path) -> dict[str, str]:
    env = os.environ.copy()
    env["OPENRADIOSS_PATH"] = str(openradioss_path)
    env["RAD_CFG_PATH"] = str(openradioss_path / "hm_cfg_files")
    env["RAD_H3D_PATH"] = str(openradioss_path / "extlib/h3d/lib/linux64")
    env.setdefault("OMP_STACKSIZE", "400m")
    env.setdefault("OMP_NUM_THREADS", "2")
    libs = [
        str(openradioss_path / "extlib/hm_reader/linux64"),
        str(openradioss_path / "extlib/h3d/lib/linux64"),
    ]
    if env.get("LD_LIBRARY_PATH"):
        libs.append(env["LD_LIBRARY_PATH"])
    env["LD_LIBRARY_PATH"] = ":".join(libs)
    return env


def build_case(args: argparse.Namespace, element_length_mm: float, run_dir: Path) -> periodic.PeriodicMotionCase:
    description = load_dog_description(args.description)
    catalog = stage1.load_catalog(args.materials, args.actuators, args.batteries)
    case = periodic.build_periodic_motion_case(
        description,
        catalog,
        out_dir=run_dir,
        run_name=args.run_name,
        sample_count=args.samples,
        solver_duration_ms=args.solver_duration_ms,
        viewer_start_seconds=args.viewer_start_seconds,
        viewer_motion_seconds=args.viewer_motion_seconds,
        babble_scale=args.babble_scale,
        motion_scale=args.motion_scale,
        target_element_length_mm=element_length_mm,
        use_nominal_radius_for_massless_members=not args.minimum_radius_for_massless_members,
        uniform_radius_mm=args.uniform_radius_mm,
        case_name=args.case_name,
        control_policy="stage1-torque-replay",
        torque_scale=args.torque_scale,
    )
    periodic.write_case(case, make_preview_gif=False, preview_frames=2, preview_duration_ms=40)
    return case


def normal_termination(path: Path) -> bool:
    text = path.read_text(encoding="utf-8", errors="ignore")
    return "NORMAL TERMINATION" in text


def engine_cycles(path: Path) -> int:
    text = path.read_text(encoding="utf-8", errors="ignore")
    marker = "TOTAL NUMBER OF CYCLES"
    for line in reversed(text.splitlines()):
        if marker in line:
            return int(line.rsplit(":", 1)[1].strip())
    return 0


def summarize_case(case: periodic.PeriodicMotionCase, result_csv: Path, starter_log: Path, engine_log: Path) -> ConvergenceResult:
    global_history, displacements, reactions = periodic.parse_displacement_history(result_csv, case.deck.node_ids)
    stresses_mpa, strains = periodic.load_beam_resultant_strains(result_csv, case)
    if not stresses_mpa or not strains:
        raise RuntimeError(f"missing /TH/BEAM resultants in {result_csv}")

    metrics = periodic.result_metrics(case, displacements, reactions, strains)
    frame_index = int(np.argmax(metrics["max_abs_strain"]))
    hottest_element = max(strains, key=lambda name: abs(float(strains[name][frame_index])))
    max_abs_stress = max(abs(float(values[frame_index])) for values in stresses_mpa.values())

    return ConvergenceResult(
        element_length_mm=float(case.deck.target_element_length_mm or 0.0),
        run_dir=case.deck.starter_path.parent,
        solver_nodes=len(case.deck.node_ids),
        beam_elements=len(case.deck.members),
        beam_resultant_elements=len(strains),
        starter_ok=normal_termination(starter_log),
        engine_ok=normal_termination(engine_log),
        engine_cycles=engine_cycles(engine_log),
        frame_index=frame_index,
        time_ms=float(global_history["time"][frame_index]),
        max_displacement_mm=float(metrics["max_disp"][frame_index]),
        max_abs_strain=float(metrics["max_abs_strain"][frame_index]),
        max_abs_stress_mpa=max_abs_stress,
        hottest_element=hottest_element,
    )


def write_summary_csv(results: list[ConvergenceResult], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(ConvergenceResult.__dataclass_fields__))
        writer.writeheader()
        for item in results:
            row = {field: getattr(item, field) for field in writer.fieldnames or []}
            row["run_dir"] = str(item.run_dir)
            writer.writerow(row)


def write_summary_yaml(results: list[ConvergenceResult], path: Path) -> None:
    data = []
    for item in results:
        row = {field: getattr(item, field) for field in ConvergenceResult.__dataclass_fields__}
        row["run_dir"] = str(item.run_dir)
        data.append(row)
    path.write_text(yaml.safe_dump({"mesh_convergence": data}, sort_keys=False), encoding="utf-8")


def write_convergence_plot(results: list[ConvergenceResult], path: Path) -> None:
    ordered = sorted(results, key=lambda item: item.element_length_mm)
    x = np.array([item.element_length_mm for item in ordered], dtype=float)
    stress = np.array([item.max_abs_stress_mpa for item in ordered], dtype=float)
    strain = np.array([item.max_abs_strain for item in ordered], dtype=float)
    elements = np.array([item.beam_elements for item in ordered], dtype=float)

    fig, axes = plt.subplots(1, 3, figsize=(12.8, 4.2), dpi=130)
    fig.patch.set_facecolor("#fbfbf8")
    for ax in axes:
        ax.grid(color="#d9d9d9", linewidth=0.55, alpha=0.55)
        ax.set_xlabel("target element length [mm]")
        ax.invert_xaxis()

    axes[0].plot(x, stress, marker="o", color="#b91c1c", linewidth=2.0)
    axes[0].set_ylabel("max outer-fiber stress [MPa]")
    axes[0].set_title("stress")

    axes[1].plot(x, strain, marker="o", color="#dc2626", linewidth=2.0)
    axes[1].set_ylabel("max outer-fiber strain")
    axes[1].set_title("strain")

    axes[2].plot(x, elements, marker="o", color="#1f2937", linewidth=2.0)
    axes[2].set_ylabel("beam elements")
    axes[2].set_title("mesh size")

    fig.suptitle("Stage 2 native beam-resultant mesh convergence", fontsize=13)
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.94))
    fig.savefig(path, facecolor=fig.get_facecolor(), bbox_inches="tight")
    plt.close(fig)


def run_one(args: argparse.Namespace, env: dict[str, str], element_length_mm: float) -> ConvergenceResult:
    label = f"{element_length_mm:g}mm".replace(".", "p")
    run_dir = args.out_root / f"element_{label}"
    case = build_case(args, element_length_mm, run_dir)

    starter = Path(env["OPENRADIOSS_PATH"]) / "exec/starter_linux64_gf"
    engine = Path(env["OPENRADIOSS_PATH"]) / "exec/engine_linux64_gf"
    th_to_csv = Path(env["OPENRADIOSS_PATH"]) / "exec/th_to_csv_linux64_gf"

    run_command([str(starter), "-i", case.deck.starter_path.name], run_dir, env, run_dir / "starter.log")
    run_command([str(engine), "-i", case.deck.engine_path.name], run_dir, env, run_dir / "engine.log")
    run_command([str(th_to_csv), f"{case.deck.run_name}T01"], run_dir, env, run_dir / "th_to_csv.log")
    return summarize_case(case, run_dir / f"{case.deck.run_name}T01.csv", run_dir / "starter.log", run_dir / "engine.log")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-root", type=Path, default=DEFAULT_OUT_ROOT)
    parser.add_argument("--openradioss-path", type=Path, default=Path(os.environ.get("OPENRADIOSS_PATH", DEFAULT_OPENRADIOSS_PATH)))
    parser.add_argument("--element-lengths-mm", type=float, nargs="+", default=[8.0, 6.0, 4.0])
    parser.add_argument("--description", type=Path, default=DEFAULT_DESCRIPTION_PATH)
    parser.add_argument("--materials", type=Path, default=stage1.DEFAULT_MATERIALS_PATH)
    parser.add_argument("--actuators", type=Path, default=stage1.DEFAULT_ACTUATORS_PATH)
    parser.add_argument("--batteries", type=Path, default=stage1.DEFAULT_BATTERIES_PATH)
    parser.add_argument("--case-name", default="stage2_viewer_periodic_motion")
    parser.add_argument("--run-name", default=periodic.RUN_NAME)
    parser.add_argument("--samples", type=int, default=25)
    parser.add_argument("--solver-duration-ms", type=float, default=8.0)
    parser.add_argument("--viewer-start-seconds", type=float, default=0.2)
    parser.add_argument("--viewer-motion-seconds", type=float, default=0.005)
    parser.add_argument("--babble-scale", type=float, default=1.0)
    parser.add_argument("--motion-scale", type=float, default=1.0)
    parser.add_argument("--uniform-radius-mm", type=float, default=12.0)
    parser.add_argument("--torque-scale", type=float, default=1.0)
    parser.add_argument("--minimum-radius-for-massless-members", action="store_true")
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    env = openradioss_env(args.openradioss_path)
    args.out_root.mkdir(parents=True, exist_ok=True)

    results = [run_one(args, env, element_length_mm) for element_length_mm in args.element_lengths_mm]
    write_summary_csv(results, args.out_root / "mesh_convergence_summary.csv")
    write_summary_yaml(results, args.out_root / "mesh_convergence_summary.yaml")
    write_convergence_plot(results, args.out_root / "mesh_convergence_summary.png")

    for item in results:
        print(
            f"{item.element_length_mm:g} mm | elements={item.beam_elements} | "
            f"stress={item.max_abs_stress_mpa:.6g} MPa | strain={item.max_abs_strain:.6g} | "
            f"hot={item.hottest_element}"
        )
    print(f"summary: {args.out_root / 'mesh_convergence_summary.csv'}")
    print(f"plot: {args.out_root / 'mesh_convergence_summary.png'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
