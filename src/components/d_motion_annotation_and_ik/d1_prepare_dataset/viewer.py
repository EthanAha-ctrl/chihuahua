"""Raw MANN, DogML and InterPet4D motion display in one PyVista viewer.

Run with the repository-root .venv/bin/python (dependencies: uv sync).
--dataset selects mann (default), dogml or interpet4d. --list lists its clips;
--clip NAME selects one. DogML also accepts --dogml-mode dog or robot.
MANN accepts --archive PATH or --bvh PATH.
"""

import argparse
from dataclasses import dataclass
from functools import wraps
import importlib.util
import inspect
import json
from math import atan2, cos, hypot, isfinite, pi, sin
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import time
import traceback
from zipfile import ZipFile

import numpy as np


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
ARCHIVE = ROOT / "artifacts/MANN-MotionCapture.zip"
DEFAULT_CLIP = "D1_007_KAN01_001.bvh"
DOGML = ROOT / "artifacts/DogML-dataset/dataset"
INTERPET = ROOT / "artifacts/InterPet4D-motion/pet_npy"
# The author's annotation describes standing up and walking forward.
DEFAULT_DOGML_CLIP = "D1_007_KAN01_001_F1_F592_001.npy"
# Keep the clip used by the original InterPet4D preview for visual comparison.
DEFAULT_INTERPET_CLIP = "interpet_dog02_p05_take04_ego_001.npy"

# Published metadata, transcribed without changing the graph or joint order:
# https://arxiv.org/html/2607.10287v1#A2.T7
INTERPET_NAMES = (
    "L_Eye", "R_Eye", "L_EarBase", "R_EarBase", "Nose", "Throat",
    "TailBase", "Withers", "L_F_Knee", "R_F_Knee", "L_B_Knee", "R_B_Knee",
    "L_F_Elbow", "R_F_Elbow", "L_B_Elbow", "R_B_Elbow", "L_F_Paw",
    "R_F_Paw", "L_B_Paw", "R_B_Paw",
)
INTERPET_PARENTS = (None, 0, 0, 1, 2, 0, 7, 5, 5, 5, 6, 6, 8, 9, 10, 11, 12, 13, 14, 15)


@dataclass(frozen=True)
class MotionClip:
    name: str
    dataset: str
    xyz: np.ndarray
    edges: tuple
    fps: float
    follow_joint: int
    follow_label: str
    up_axis: int
    confidence: np.ndarray | None = None
    frame_times: np.ndarray | None = None
    edge_colors: tuple | None = None


def read_bvh(path):
    from ai4animation.Import.BVHImporter import BVH

    bvh = BVH(str(path), scale=1.0)
    # Upstream's default selection omits End Sites. Keep every parsed node.
    motion = bvh.LoadMotion(names=bvh._names)
    parents = motion.Hierarchy.ParentIndices
    roots = [i for i, parent in enumerate(parents) if parent is None]
    if len(roots) != 1:
        raise ValueError("BVH follow requires exactly one skeleton root")
    return MotionClip(
        name=path.name, dataset="MANN / BVH", xyz=motion.GetBonePositions(),
        edges=tuple((parent, child) for child, parent in enumerate(parents)
                    if parent is not None),
        fps=float(motion.Framerate), follow_joint=roots[0],
        follow_label=motion.Hierarchy.BoneNames[roots[0]], up_axis=1,
    )


def read_archive(archive, clip):
    with ZipFile(archive) as source, TemporaryDirectory(prefix="ai4a-bvh-") as tmp:
        path = Path(tmp) / Path(clip).name
        path.write_bytes(source.read(clip))
        return read_bvh(path)


def dogml_tools():
    path = DOGML / "vis.py"
    if not path.is_file():
        raise FileNotFoundError(f"DogML's upstream visualization tools are missing: {path}")
    spec = importlib.util.spec_from_file_location("_dogml_upstream_vis", path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot import DogML's upstream tools: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def read_dogml(path, mode):
    tools = dogml_tools()
    joint_count = {"dog": 27, "robot": 16}[mode]
    data = np.load(path, allow_pickle=False)
    expected_width = {"dog": 323, "robot": 191}[mode]
    if data.ndim != 2 or data.shape[1] != expected_width or not np.isfinite(data).all():
        raise ValueError(f"Invalid DogML {mode} motion array: {path}, shape={data.shape}")
    # All quaternion integration and coordinate recovery stay in the author's code.
    xyz = tools.recover_from_ric(data, joint_count)
    chains = tools.kinematic_chain_dog if mode == "dog" else tools.kinematic_chain_robot
    fps = inspect.signature(tools.plot_3d_keypoints).parameters["fps"].default
    return MotionClip(
        name=path.name, dataset=f"DogML / {mode}", xyz=xyz,
        edges=tuple((a, b) for chain in chains for a, b in zip(chain, chain[1:])),
        fps=float(fps), follow_joint=0, follow_label="root (joint 0)", up_axis=1,
    )


def read_interpet(path):
    # The official dataset loading example uses np.load directly for pet_npy:
    # https://huggingface.co/datasets/ohicarip/interpet4d#loading-example
    data = np.load(path, allow_pickle=False)
    if data.ndim != 3 or data.shape[1:] != (20, 4):
        raise ValueError(f"Invalid InterPet4D array: {path}, shape={data.shape}")
    dataset = path.parent.parent
    manifest = json.loads((dataset / "cleanup_smal/manifest.json").read_text())
    source_fps = float(manifest["source_fps"])
    with np.load(dataset / "smal_npy" / f"{path.stem}.npz", allow_pickle=False) as source:
        frame_indices = source["frame_idx"].copy()
    if (frame_indices.shape != (len(data),) or len(data) < 2
            or frame_indices.dtype.kind not in "iu"
            or not np.isfinite(source_fps) or source_fps <= 0
            or not (frame_indices[1:] > frame_indices[:-1]).all()):
        raise ValueError(f"Invalid or unaligned InterPet4D source timestamps: {path}")
    # Parse source frame timestamps without interpolating or changing motion data.
    frame_times = (frame_indices - frame_indices[0]) / source_fps
    edges = tuple((parent, child) for child, parent in enumerate(INTERPET_PARENTS)
                  if parent is not None)
    # Restore the original viewer's left/right/body colors, using published names.
    edge_colors = tuple("#1689b2" if INTERPET_NAMES[child].startswith("L_")
                        else "#dc8834" if INTERPET_NAMES[child].startswith("R_")
                        else "#596875" for _, child in edges)
    # No training normalization, SMAL reconstruction or confidence filtering.
    # Z-up: reference/interpet4d_post/models/wan_pet_vqvae.py height_axis=2.
    return MotionClip(
        name=path.name, dataset="InterPet4D / pet_npy", xyz=data[..., :3],
        edges=edges, fps=float(1.0 / np.median(np.diff(frame_times))),
        follow_joint=7, follow_label=INTERPET_NAMES[7], up_axis=2,
        confidence=data[..., 3], frame_times=frame_times, edge_colors=edge_colors,
    )


class Viewer:
    def __init__(self, clip):
        import pyvista as pv

        self.xyz = clip.xyz
        self.fps = clip.fps
        edges = np.asarray(clip.edges)
        if (self.xyz.ndim != 3 or self.xyz.shape[2] != 3
                or len(self.xyz) < 2 or self.xyz.shape[1] == 0
                or not np.isfinite(self.xyz).all()
                or not np.isfinite(self.fps) or self.fps <= 0
                or edges.ndim != 2 or edges.shape[1] != 2
                or edges.dtype.kind not in "iu"
                or (edges < 0).any() or (edges >= self.xyz.shape[1]).any()
                or not 0 <= clip.follow_joint < self.xyz.shape[1]
                or clip.up_axis not in (1, 2)):
            raise ValueError(f"Invalid motion coordinates, connectivity, or metadata: {clip.name}")
        self.clip = clip
        self.times = (np.arange(len(self.xyz)) / self.fps
                      if clip.frame_times is None else clip.frame_times)
        if (self.times.shape != (len(self.xyz),) or not np.isfinite(self.times).all()
                or self.times[0] != 0 or not (np.diff(self.times) > 0).all()):
            raise ValueError(f"Invalid frame timestamps: {clip.name}")
        self.times.setflags(write=False)
        if clip.edge_colors is not None and len(clip.edge_colors) != len(edges):
            raise ValueError(f"Invalid edge colors: {clip.name}")
        self.follow_root = clip.follow_joint
        self.confidence = clip.confidence
        if self.confidence is not None:
            if (self.confidence.shape != self.xyz.shape[:2]
                    or not np.isfinite(self.confidence).all()
                    or (self.confidence < 0).any() or (self.confidence > 1).any()):
                raise ValueError(f"Invalid confidence scores: {clip.name}")
            self.confidence.setflags(write=False)
        self.xyz.setflags(write=False)
        self.frame = 0
        self.playing = False
        self.error = None
        self.slider = None
        self.button = None
        self.started_at = 0.0
        self.started_frame = 0
        self.camera_yaw = atan2(1.4, 1.0)
        self.fixed_camera_pitch = atan2(0.7, hypot(1.4, 1.0))

        self.plotter = pv.Plotter(
            title=f"{clip.dataset} | Yaw only | {clip.name}",
            window_size=(1120, 780), notebook=False,
        )
        self.plotter.set_background("#f5f7fa")
        self.plotter.enable_trackball_style()
        self.plotter.iren.style.add_observer("MouseMoveEvent", self.guard(self.orbit))
        self.plotter.enable_parallel_projection()
        self.plotter.add_axes(interactive=False, viewport=(0.87, 0.79, 0.99, 0.96))
        lines = np.column_stack((np.full(len(edges), 2), edges)).ravel()
        self.mesh = pv.PolyData(self.xyz[0].copy(), lines=lines)
        line_options = {"color": "#237b98"}
        if clip.edge_colors is not None:
            self.mesh.cell_data["edge_color"] = np.asarray(
                [pv.Color(color).int_rgb for color in clip.edge_colors], dtype=np.uint8,
            )
            line_options = {"scalars": "edge_color", "rgb": True,
                            "preference": "cell", "show_scalar_bar": False}
        self.plotter.add_mesh(self.mesh, **line_options, line_width=6,
                              render_lines_as_tubes=True)
        self.point_mesh = pv.PolyData(self.xyz[0].copy())
        point_options = {"color": "#dc8834"}
        if self.confidence is not None:
            self.point_mesh.point_data["confidence"] = self.confidence[0].copy()
            point_options = {
                "scalars": "confidence", "cmap": "viridis", "clim": (0.0, 1.0),
                "scalar_bar_args": {"title": "Raw confidence (all points shown)",
                                    "position_x": 0.38, "position_y": 0.86,
                                    "width": 0.5, "height": 0.05},
            }
        self.plotter.add_mesh(self.point_mesh, style="points", **point_options,
                              point_size=11, render_points_as_spheres=True)
        # Change the camera only; the mesh keeps the upstream XYZ values.
        view = (1.4, 0.7, 1.0) if clip.up_axis == 1 else (1.4, 1.0, 0.7)
        up = (0, 1, 0) if clip.up_axis == 1 else (0, 0, 1)
        self.plotter.view_vector(view, viewup=up)
        self.plotter.render_window.AddObserver(
            "StartEvent", self.guard(self.apply_camera_yaw), 1.0,
        )
        self.fit()
        self.plotter.add_text(
            f"{clip.dataset}\n{clip.name}\n{len(self.xyz)} frames | {self.fps:.2f} Hz | "
            f"{self.xyz.shape[1]} nodes\nNative coordinates and units\n"
            f"Camera follows {clip.follow_label} | {'Y' if clip.up_axis == 1 else 'Z'}-up",
            position="upper_left", font_size=11, color="#273543",
        )
        self.status = self.plotter.add_text(
            "", position=(24, 160), font_size=10, color="#273543",
        )
        self.plotter.add_text("Play / Pause", position=(68, 111), font_size=10,
                              color="#273543")
        self.plotter.add_text(
            "Horizontal drag: yaw only | Shift+drag: pan | Scroll: zoom\n"
            "Space: play/pause | Left/Right: step | Home/End: first/last | F: fit",
            position=(24, 22), font_size=9, color="#526373",
        )
        self.button = self.plotter.add_checkbox_button_widget(
            self.guard(self.play), value=False, position=(24, 105), size=30,
            color_on="#237b98", color_off="#cbd5dc",
        )
        self.slider = self.plotter.add_slider_widget(
            self.guard(self.seek), (0, len(self.xyz) - 1), value=0,
            title="Frame index (0-based)", fmt="%.0f",
            pointa=(0.29, 0.16), pointb=(0.94, 0.16), color="#596875",
            interaction_event="always", title_height=0.020,
            slider_width=0.012, tube_width=0.004,
        )
        callbacks = {
            "space": lambda: self.play(not self.playing),
            "Left": lambda: self.seek(self.frame - 1),
            "Right": lambda: self.seek(self.frame + 1),
            "Home": lambda: self.seek(0),
            "End": lambda: self.seek(len(self.xyz) - 1),
            "f": self.fit,
        }
        for key, callback in callbacks.items():
            self.plotter.iren.clear_events_for_key(key)
            self.plotter.add_key_event(key, self.guard(callback))
        self.set_frame(0)

    def fit(self):
        self.plotter.reset_camera(render=False)
        self.move_camera(self.xyz[self.frame, self.follow_root].astype(float)
                         - self.plotter.camera.focal_point)
        self.plotter.camera.zoom(0.65)
        self.plotter.reset_camera_clipping_range()
        self.plotter.render()

    def move_camera(self, shift):
        camera = self.plotter.camera
        camera.position = tuple(p + d for p, d in zip(camera.position, shift, strict=True))
        camera.focal_point = tuple(p + d for p, d in zip(camera.focal_point, shift, strict=True))

    def orbit(self, style, _event):
        from vtkmodules.vtkRenderingCore import VTKIS_ROTATE, VTKIS_SPIN

        # Both ordinary orbit and Ctrl-drag change yaw only; retain pan/zoom.
        if style.GetState() not in (VTKIS_ROTATE, VTKIS_SPIN):
            style.OnMouseMove()
            return
        interactor = style.GetInteractor()
        mouse_x, _ = interactor.GetEventPosition()
        last_x, _ = interactor.GetLastEventPosition()
        width, _ = interactor.GetRenderWindow().GetSize()
        if width <= 0:
            raise ValueError("Invalid camera viewport width")
        self.camera_yaw = (self.camera_yaw - pi * (mouse_x - last_x) / width) % (2 * pi)
        self.apply_camera_yaw()
        self.plotter.reset_camera_clipping_range()
        if interactor.GetLightFollowCamera():
            self.plotter.renderer.UpdateLightsGeometryToFollowCamera()
        self.plotter.render()
        style.InvokeEvent("InteractionEvent")

    def apply_camera_yaw(self, *_):
        # Pin pitch and roll before every render, including widget/native redraws.
        # Use each dataset's vertical axis without rotating its motion coordinates.
        camera = self.plotter.camera
        focal = camera.focal_point
        x, y, z = (p - f for p, f in zip(camera.position, focal, strict=True))
        distance = hypot(x, y, z)
        if not isfinite(distance) or distance <= 0:
            raise ValueError("Invalid camera orbit distance")
        sy, cy = sin(self.camera_yaw), cos(self.camera_yaw)
        sp, cp = sin(self.fixed_camera_pitch), cos(self.fixed_camera_pitch)
        if self.clip.up_axis == 1:
            direction, up = (cp * sy, sp, cp * cy), (-sp * sy, cp, -sp * cy)
        else:
            direction, up = (cp * sy, cp * cy, sp), (-sp * sy, -sp * cy, cp)
        camera.position = tuple(f + distance * d for f, d in zip(focal, direction, strict=True))
        camera.up = up

    def guard(self, callback):
        @wraps(callback)
        def call(*args):
            if self.error is not None:
                return
            try:
                return callback(*args)
            except Exception as error:
                self.error = error
                self.playing = False
                print("FATAL: viewer callback failed; stopping.", file=sys.stderr,
                      flush=True)
                traceback.print_exc()
                self.plotter.iren.terminate_app()
        return call

    def set_frame(self, frame):
        frame = max(0, min(int(frame), len(self.xyz) - 1))
        # Translate the camera with the root, preserving manual orbit, pan and zoom.
        shift = (self.xyz[frame, self.follow_root].astype(float)
                 - self.xyz[self.frame, self.follow_root].astype(float))
        self.move_camera(shift)
        self.frame = frame
        self.mesh.points = self.xyz[self.frame].copy()
        self.point_mesh.points = self.xyz[self.frame].copy()
        if self.confidence is not None:
            self.point_mesh.point_data["confidence"] = self.confidence[self.frame].copy()
        self.plotter.reset_camera_clipping_range()
        if self.slider is not None:
            self.slider.GetRepresentation().SetValue(self.frame)
        if self.button is not None:
            self.button.GetRepresentation().SetState(int(self.playing))
        self.status.SetInput(
            f"{'PLAYING' if self.playing else 'PAUSED'} | "
            f"{self.frame}/{len(self.xyz) - 1} | "
            f"{self.times[self.frame]:.3f} s"
        )
        self.plotter.render()

    def seek(self, frame):
        self.playing = False
        self.set_frame(round(frame))

    def play(self, enabled):
        if enabled and self.frame == len(self.xyz) - 1:
            self.set_frame(0)
        self.playing = bool(enabled)
        self.started_frame = self.frame
        self.started_at = time.perf_counter()
        self.set_frame(self.frame)

    def tick(self, _object, _event):
        if self.playing:
            timestamp = self.times[self.started_frame] + time.perf_counter() - self.started_at
            frame = int(np.searchsorted(self.times, timestamp, side="right") - 1)
            if timestamp >= self.times[-1]:
                self.playing = False
            if frame != self.frame:
                self.set_frame(frame)

    def run(self):
        try:
            if self.error is not None:
                raise RuntimeError("Viewer initialization failed") from self.error
            self.plotter.show(interactive_update=True, auto_close=False)
            # Replace the style's timer handler to prevent its native Rotate/Spin.
            self.plotter.iren.style.add_observer("TimerEvent", self.guard(self.tick))
            timer = self.plotter.iren.create_timer(max(1, round(1000 / self.fps)))
            if timer <= 0:
                raise RuntimeError("Could not create the playback timer")
            if sys.platform == "darwin":
                from AppKit import NSApplication, NSApplicationActivationPolicyRegular

                app = NSApplication.sharedApplication()
                app.setActivationPolicy_(NSApplicationActivationPolicyRegular)
                app.activateIgnoringOtherApps_(True)
            print(f"READY: {self.clip.dataset}; paused at frame {self.frame}; {len(self.xyz)} frames, "
                  f"{self.xyz.shape[1]} nodes, {self.fps:.6f} Hz.", flush=True)
            self.plotter.iren.start()
        finally:
            self.plotter.close()
        if self.error is not None:
            raise RuntimeError("Viewer stopped after a callback failure") from self.error


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", choices=("mann", "dogml", "interpet4d"), default="mann")
    sources = parser.add_mutually_exclusive_group()
    sources.add_argument("--archive", type=Path)
    sources.add_argument("--bvh", type=Path)
    parser.add_argument("--clip")
    parser.add_argument("--dogml-mode", choices=("dog", "robot"), default="dog")
    parser.add_argument("--list", action="store_true")
    args = parser.parse_args()
    if Path(sys.prefix).resolve() != (ROOT / ".venv").resolve():
        raise RuntimeError(f"Use the repository interpreter: {ROOT / '.venv/bin/python'}")
    if args.dataset != "mann" and (args.archive or args.bvh):
        parser.error("--archive and --bvh require --dataset mann")
    if args.dataset != "dogml" and args.dogml_mode != "dog":
        parser.error("--dogml-mode requires --dataset dogml")
    if args.bvh and (args.list or args.clip):
        parser.error("--bvh cannot be combined with --list or --clip")
    if args.dataset == "mann":
        archive = args.archive or ARCHIVE
        if args.list:
            with ZipFile(archive) as source:
                clips = [name for name in source.namelist() if name.lower().endswith(".bvh")]
        else:
            clip = read_bvh(args.bvh) if args.bvh else read_archive(archive, args.clip or DEFAULT_CLIP)
    else:
        directory = DOGML / "motion" / args.dogml_mode if args.dataset == "dogml" else INTERPET
        if not directory.is_dir():
            raise FileNotFoundError(f"Dataset directory is missing: {directory}")
        if args.list:
            clips = [path.name for path in sorted(directory.glob("*.npy"))]
        elif args.dataset == "dogml":
            clip = read_dogml(directory / (args.clip or DEFAULT_DOGML_CLIP), args.dogml_mode)
        else:
            clip = read_interpet(directory / (args.clip or DEFAULT_INTERPET_CLIP))
    if args.list:
        if not clips:
            raise ValueError(f"No clips found for {args.dataset}")
        print("\n".join(clips))
        print(f"Total: {len(clips)}")
        return
    Viewer(clip).run()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        print("FATAL: motion viewer failed.", file=sys.stderr, flush=True)
        traceback.print_exc()
        raise SystemExit(1)
