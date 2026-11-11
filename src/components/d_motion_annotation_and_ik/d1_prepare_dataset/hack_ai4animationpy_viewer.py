"""AI4AnimationPy-only viewer hack: upstream BVH import, PyVista display.

Run with the repository-root .venv/bin/python (dependencies: uv sync).
Defaults to the downloaded MANN archive.
--list lists clips; --clip NAME selects an archive member; --bvh PATH opens a file.
"""

import argparse
from functools import wraps
from math import atan2, cos, hypot, isfinite, pi, sin
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import time
import traceback
from zipfile import ZipFile


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
ARCHIVE = ROOT / "artifacts/MANN-MotionCapture.zip"
DEFAULT_CLIP = "D1_007_KAN01_001.bvh"


def read_bvh(path):
    from ai4animation.Import.BVHImporter import BVH

    bvh = BVH(str(path), scale=1.0)
    # Upstream's default selection omits End Sites. Keep every parsed node.
    return bvh.LoadMotion(names=bvh._names)


def read_archive(archive, clip):
    with ZipFile(archive) as source, TemporaryDirectory(prefix="ai4a-bvh-") as tmp:
        path = Path(tmp) / Path(clip).name
        path.write_bytes(source.read(clip))
        return read_bvh(path)


class Viewer:
    def __init__(self, motion):
        import numpy as np
        import pyvista as pv

        self.xyz = motion.GetBonePositions()
        self.fps = float(motion.Framerate)
        parents = motion.Hierarchy.ParentIndices
        if (self.xyz.ndim != 3 or self.xyz.shape[2] != 3
                or len(self.xyz) < 2 or self.xyz.shape[1] != len(parents)
                or not np.isfinite(self.xyz).all()
                or not np.isfinite(self.fps) or self.fps <= 0):
            raise ValueError("Invalid motion coordinates, hierarchy, or frame rate")
        roots = [index for index, parent in enumerate(parents) if parent is None]
        if len(roots) != 1:
            raise ValueError("Third-person follow requires exactly one skeleton root")
        self.follow_root = roots[0]
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
            title=f"AI4AnimationPy BVH | Yaw only | {motion.Name}",
            window_size=(1120, 780), notebook=False,
        )
        self.plotter.set_background("#f5f7fa")
        self.plotter.enable_trackball_style()
        self.plotter.iren.style.add_observer("MouseMoveEvent", self.guard(self.orbit))
        self.plotter.enable_parallel_projection()
        self.plotter.add_axes(interactive=False, viewport=(0.87, 0.79, 0.99, 0.96))
        edges = [[2, parent, child] for child, parent in enumerate(parents)
                 if parent is not None]
        self.mesh = pv.PolyData(self.xyz[0].copy(), lines=np.asarray(edges).ravel())
        self.plotter.add_mesh(self.mesh, color="#237b98", line_width=6,
                              render_lines_as_tubes=True)
        self.plotter.add_mesh(self.mesh, style="points", color="#dc8834",
                              point_size=11, render_points_as_spheres=True)
        # Change the camera only; the mesh keeps the upstream XYZ values.
        self.plotter.view_vector((1.4, 0.7, 1.0), viewup=(0, 1, 0))
        self.plotter.render_window.AddObserver(
            "StartEvent", self.guard(self.apply_camera_yaw), 1.0,
        )
        self.fit()
        self.plotter.add_text(
            f"{motion.Name}\n{len(self.xyz)} frames | {self.fps:.2f} Hz | "
            f"{self.xyz.shape[1]} nodes\nNative BVH coordinates and units\n"
            f"Camera follows {motion.Hierarchy.BoneNames[self.follow_root]}",
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
        # The raw BVH uses Y-up; only yaw is an interactive orientation parameter.
        camera = self.plotter.camera
        focal = camera.focal_point
        x, y, z = (p - f for p, f in zip(camera.position, focal, strict=True))
        distance = hypot(x, y, z)
        if not isfinite(distance) or distance <= 0:
            raise ValueError("Invalid camera orbit distance")
        sy, cy = sin(self.camera_yaw), cos(self.camera_yaw)
        sp, cp = sin(self.fixed_camera_pitch), cos(self.fixed_camera_pitch)
        camera.position = (focal[0] + distance * cp * sy,
                           focal[1] + distance * sp,
                           focal[2] + distance * cp * cy)
        camera.up = (-sp * sy, cp, -sp * cy)

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
        self.plotter.reset_camera_clipping_range()
        if self.slider is not None:
            self.slider.GetRepresentation().SetValue(self.frame)
        if self.button is not None:
            self.button.GetRepresentation().SetState(int(self.playing))
        self.status.SetInput(
            f"{'PLAYING' if self.playing else 'PAUSED'} | "
            f"{self.frame}/{len(self.xyz) - 1} | "
            f"{self.frame / self.fps:.3f} s"
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
            frame = self.started_frame + int(
                (time.perf_counter() - self.started_at) * self.fps
            )
            if frame >= len(self.xyz) - 1:
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
            print(f"READY: paused at frame 0; {len(self.xyz)} frames, "
                  f"{self.xyz.shape[1]} nodes, {self.fps:.6f} Hz.", flush=True)
            self.plotter.iren.start()
        finally:
            self.plotter.close()
        if self.error is not None:
            raise RuntimeError("Viewer stopped after a callback failure") from self.error


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sources = parser.add_mutually_exclusive_group()
    sources.add_argument("--archive", type=Path, default=ARCHIVE)
    sources.add_argument("--bvh", type=Path)
    parser.add_argument("--clip", default=DEFAULT_CLIP)
    parser.add_argument("--list", action="store_true")
    args = parser.parse_args()
    if Path(sys.prefix).resolve() != (ROOT / ".venv").resolve():
        raise RuntimeError(f"Use the repository interpreter: {ROOT / '.venv/bin/python'}")
    if args.list:
        if args.bvh:
            parser.error("--list requires an archive")
        with ZipFile(args.archive) as archive:
            clips = [name for name in archive.namelist() if name.lower().endswith(".bvh")]
        if not clips:
            raise ValueError(f"No BVH clips in {args.archive}")
        print("\n".join(clips))
        print(f"Total: {len(clips)}")
        return
    motion = read_bvh(args.bvh) if args.bvh else read_archive(args.archive, args.clip)
    Viewer(motion).run()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        print("FATAL: AI4AnimationPy viewer failed.", file=sys.stderr, flush=True)
        traceback.print_exc()
        raise SystemExit(1)
