"""
Records the videos of the examples (as shown in the documentation) from the viser viewer.

    python recording/record.py [video ...]

The videos are listed in recording/videos.json (by default, all of them are recorded). Each example runs on a
simulated clock: scheduled loops (ischedule) and time.sleep() advance it, and a headless browser connected to the
viser server renders a frame every 1/fps of simulated time. The result doesn't depend on how fast the rendering is.

Requires playwright (pip install playwright, playwright install chromium) and imageio-ffmpeg.

Video settings (videos.json):
- example, args: the example script (relative to the repository) and its arguments
- start: simulated time at which the recording starts (the robot has reached a steady motion)
- duration: recorded duration [s]; for a looping video, the period of the motion
- sync: an example global variable, recording starts right after it changes (e.g. when a point is added to a
  trail), so that the loop seam is not visible
- camera: either "position" and "look_at" [m], or "view" [azimuth, elevation] [deg], in which case the camera is
  placed automatically to see everything that moves during the recording
- fov: vertical field of view [deg]
- gif: [width, fps] to also produce a GIF next to the video
- sliders: keyframes ([time, value], from the start, repeated every duration) moving sliders of the viewer panel
- panel: true to record the whole page, with the viewer panel (e.g. to see the sliders)
"""

import argparse
import io
import json
import os
import random
import subprocess
import sys
import tempfile
import time
import types

import imageio_ffmpeg
import numpy as np
from PIL import Image

repository = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class StopRecording(Exception):
    pass


class Recorder:
    def __init__(self, video: str, settings: dict, measure: bool, gpu: bool, width: int, height: int, fps: int):
        self.video = video
        self.settings = settings
        self.measure = measure
        self.gpu = gpu
        self.width, self.height, self.fps = width, height, fps

        self.now = 0.0
        self.preroll = settings.get("preroll", 2.0)
        self.start = None if "sync" in settings else settings.get("start", 0.0)
        self.frames = int(round(settings["duration"] * fps))
        self.frame = -int(round(self.preroll * fps))
        self.globals = {}

        # Measurement: points that are displayed during the recording
        self.points = []
        self.displayed = {}

        self.browser = None
        self.client = None
        self.ffmpeg = None
        self.synced = None

        # To check the loop seam
        self.first, self.last, self.steps = None, None, []

    # Simulated clock

    def frame_time(self, frame: int) -> float:
        if frame < 0:
            return self.settings.get("start", 0.0) + frame / self.fps
        return self.start + frame * self.settings["duration"] / self.frames

    def advance(self, t: float):
        """
        The scene reflects the state at self.now: renders the frames before t, then advances the clock to t
        """
        if self.start is None and self.now >= self.settings.get("start", 0.0):
            value = self.globals[self.settings["sync"]]
            if self.synced is not None and value != self.synced:
                self.start = self.now
                self.frame = max(self.frame, 0)
            self.synced = value

        # Pre-roll frames (frame < 0) are rendered before the start is known (sync)
        while self.frame < 0 or self.start is not None:
            if self.frame_time(self.frame) >= t:
                break
            self.capture(record=self.frame >= 0)
            self.frame += 1
            if self.frame >= self.frames:
                raise StopRecording()

        self.now = t
        self.move_sliders()

    def move_sliders(self):
        """
        Sets the sliders of the viewer panel from their keyframes ([time, value], linearly interpolated, repeated
        every duration from the start)
        """
        import placo_utils.visualization as viz

        for name, keyframes in self.settings.get("sliders", {}).items():
            if name in viz.sliders:
                times, values = np.array(keyframes, dtype=float).T
                t = (self.now - self.settings.get("start", 0.0)) % self.settings["duration"]
                value = float(np.interp(t, times, values))
                if viz.sliders[name].value != value:
                    viz.sliders[name].value = value

    # Rendering

    def capture(self, record: bool):
        if self.measure:
            if record:
                self.measure_points()
            return

        if self.client is None:
            self.connect()

        if self.settings.get("panel"):
            # Screenshot of the page, with the viewer panel (e.g. to see sliders moving). The render request makes sure
            # that the browser received the updates of the scene before
            self.client.get_render(16, 16, transport_format="jpeg")
            image = np.array(Image.open(io.BytesIO(self.page.screenshot(type="jpeg", quality=95))).convert("RGB"))
        else:
            image = self.client.get_render(self.height, self.width, transport_format="jpeg")
        if record:
            self.ffmpeg.stdin.write(np.ascontiguousarray(image).tobytes())
            image = image.astype(np.float32)
            if self.first is None:
                self.first = image
            else:
                self.steps.append(np.abs(image - self.last).mean())
            self.last = image

    def connect(self):
        from playwright.sync_api import sync_playwright
        import placo_utils.visualization as viz

        server = viz.get_viewer()
        camera = self.camera()
        server.initial_camera.up = (0.0, 0.0, 1.0)
        server.initial_camera.position = tuple(camera["position"])
        server.initial_camera.look_at = tuple(camera["look_at"])
        server.initial_camera.fov = np.deg2rad(self.settings.get("fov", 40.0))
        print(f"camera: position {np.round(camera['position'], 2).tolist()}, look_at {np.round(camera['look_at'], 2).tolist()}")

        if self.gpu:
            flags = ["--use-angle=vulkan", "--enable-features=Vulkan", "--enable-gpu", "--ignore-gpu-blocklist"]
        else:
            flags = ["--use-angle=swiftshader", "--enable-unsafe-swiftshader", "--ignore-gpu-blocklist"]
        self.playwright = sync_playwright().start()
        executable = os.environ.get("RECORD_CHROMIUM")
        self.browser = self.playwright.chromium.launch(executable_path=executable, args=flags)
        page = self.browser.new_page(viewport={"width": self.width, "height": self.height})
        self.page = page
        page.goto(f"http://localhost:{server.get_port()}")
        while len(server.get_clients()) == 0:
            time.sleep(0.1)
        self.client = list(server.get_clients().values())[0]
        time.sleep(3.0)  # Meshes loading

        # A few renders of the static scene (the viewer adapts its rendering during the first frames)
        for _ in range(30):
            self.client.get_render(self.height, self.width, transport_format="jpeg")

        output = os.path.join(repository, self.video)
        self.ffmpeg = subprocess.Popen(
            [imageio_ffmpeg.get_ffmpeg_exe(), "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
             "-s", f"{self.width}x{self.height}", "-r", str(self.fps), "-i", "-",
             "-c:v", "libx264", "-preset", "slow", "-crf", str(self.settings.get("crf", 25)), "-pix_fmt", "yuv420p",
             "-movflags", "+faststart", output],
            stdin=subprocess.PIPE,
        )

    def camera(self) -> dict:
        settings = self.settings["camera"]
        if "position" in settings:
            return settings

        # Automatic placement: from the given direction, the camera looks at the center of what is displayed during
        # the recording and is placed so that it fits in the image (with a margin)
        points = np.array(measure(self.video))
        azimuth, elevation = np.deg2rad(settings["view"])
        direction = np.array([np.cos(elevation) * np.cos(azimuth), np.cos(elevation) * np.sin(azimuth), np.sin(elevation)])
        forward = -direction
        right = np.cross(forward, [0.0, 0.0, 1.0])
        right /= np.linalg.norm(right)
        up = np.cross(right, forward)

        tan_v = np.tan(np.deg2rad(self.settings.get("fov", 40.0)) / 2) * (1 - 2 * settings.get("margin", 0.08))
        tan_h = tan_v * self.width / self.height
        center = (points.min(axis=0) + points.max(axis=0)) / 2
        for _ in range(10):
            # Distance at which every point is in the image, then centering the projected points
            def fits(distance):
                relative = points - (center + direction * distance)
                depth = relative @ forward
                return np.all(depth > 0.05) and np.all(np.abs(relative @ right) <= depth * tan_h) and np.all(
                    np.abs(relative @ up) <= depth * tan_v
                )

            low, high = 0.0, 100.0
            for _ in range(50):
                low, high = (low, (low + high) / 2) if fits((low + high) / 2) else ((low + high) / 2, high)
            distance = high
            relative = points - (center + direction * distance)
            depth = relative @ forward
            x, y = (relative @ right) / depth, (relative @ up) / depth
            mean_depth = np.mean(depth)
            center = center + right * (x.min() + x.max()) / 2 * mean_depth + up * (y.min() + y.max()) / 2 * mean_depth

        distance *= settings.get("zoom", 1.0)
        return {"position": (center + direction * distance).tolist(), "look_at": center.tolist()}

    # Measurement of what is displayed

    def measure_points(self):
        import placo

        for points in self.displayed.values():
            self.points.extend(points)

    def wrap_visualization(self):
        import placo_utils.visualization as viz

        def positions(*arrays):
            result = []
            for array in arrays:
                array = np.asarray(array, dtype=float)
                if array.shape == (4, 4):
                    result.append(array[:3, 3])
                elif array.ndim == 2:
                    result.extend(array[:, :3])
                elif array.shape[0] >= 3:
                    result.append(array[:3])
            return result

        def wrap(name, extract):
            original = getattr(viz, name)

            def wrapper(*args, **kwargs):
                if self.measure:
                    try:
                        self.displayed[(name, str(args[0]))] = extract(*args, **kwargs)
                    except Exception:
                        pass
                return original(*args, **kwargs)

            setattr(viz, name, wrapper)

        # Robots: corners of the bounding boxes of their visual geometries
        original_init, original_display = viz.RobotViz.__init__, viz.RobotViz.display

        def init(robot_viz, robot, *args, **kwargs):
            robot_viz.corners = []
            for geometry in robot.visual_model.objects:
                size = np.array(geometry.size, dtype=float)
                if geometry.type == "box":
                    lower, upper = -size / 2, size / 2
                elif geometry.type in ["sphere", "ellipsoid"]:
                    radius = size if geometry.type == "ellipsoid" else np.full(3, size[0])
                    lower, upper = -radius, radius
                elif geometry.type in ["cylinder", "capsule"]:
                    extent = np.array([size[0], size[0], size[1] / 2 + (size[0] if geometry.type == "capsule" else 0)])
                    lower, upper = -extent, extent
                elif geometry.type == "plane":
                    lower, upper = np.array([-size[0], -size[1], 0.0]), np.array([size[0], size[1], 0.0])
                else:
                    mesh = viz.load_mesh(geometry.mesh_path, False)
                    lower, upper = mesh.bounds * np.array(geometry.mesh_scale)
                corners = np.array([[x, y, z, 1.0] for x in [lower[0], upper[0]] for y in [lower[1], upper[1]] for z in [lower[2], upper[2]]])
                robot_viz.corners.append(corners)
            original_init(robot_viz, robot, *args, **kwargs)

        def display(robot_viz, q):
            if self.measure:
                self.displayed[("robot", id(robot_viz))] = [
                    (T @ corner)[:3] for T, corners in zip(robot_viz.robot.visual_placements(q), robot_viz.corners) for corner in corners
                ]
            return original_display(robot_viz, q)

        viz.RobotViz.__init__ = init
        viz.RobotViz.display = display

        def frame(T, scale=1.0):
            # Origin and axes tips (0.1 m long at scale 1)
            T = np.asarray(T, dtype=float)
            return [(T @ [*(0.1 * scale * axis), 1.0])[:3] for axis in np.vstack([np.zeros(3), np.eye(3)])]

        def spheres(points, radius):
            # Bounding box corners of spheres
            return [p + radius * np.array(corner) for p in positions(points) for corner in np.vstack([np.eye(3), -np.eye(3)])]

        def argument(args, kwargs, index, name, default):
            return kwargs[name] if name in kwargs else (args[index] if len(args) > index else default)

        wrap("frame_viz", lambda name, T, *a, **k: frame(T, argument(a, k, 1, "scale", 1.0)))
        wrap("point_viz", lambda name, p, *a, **k: spheres(p, argument(a, k, 0, "radius", 0.01)))
        wrap("points_viz", lambda name, p, *a, **k: spheres(p, argument(a, k, 0, "radius", 0.01)))
        wrap(
            "robot_frame_viz",
            lambda robot, name, *a, **k: frame(robot.get_T_world_frame(name), argument(a, k, 1, "scale", 1.0)),
        )
        wrap("arrow_viz", lambda name, a, b, *x, **k: positions(a, b))
        wrap("line_viz", lambda name, p, *a, **k: positions(p))

    # Running the example

    def run(self):
        example = os.path.join(repository, self.settings["example"])
        os.chdir(os.path.dirname(example))
        sys.path.insert(0, os.path.dirname(example))
        sys.argv = [example] + self.settings.get("args", [])
        random.seed(0)
        np.random.seed(0)

        self.wrap_visualization()
        recorder = self

        # Simulated time for the example (the viewer keeps the real time module)
        fake_time = types.ModuleType("time")
        fake_time.__dict__.update(time.__dict__)
        fake_time.time = lambda: 1e9 + recorder.now
        fake_time.sleep = lambda duration: recorder.advance(recorder.now + duration)

        # ischedule: the scheduled functions are called at their interval of simulated time
        scheduled = []

        def schedule(interval=None, **kwargs):
            def decorator(f):
                scheduled.append([interval, 0.0, f])
                return f

            return decorator

        def run_loop(*args, **kwargs):
            while True:
                entry = min(scheduled, key=lambda entry: entry[1])
                recorder.advance(entry[1])
                entry[2]()
                entry[1] += entry[0]

        real_time = sys.modules["time"]
        sys.modules["time"] = fake_time
        sys.modules["ischedule"] = types.SimpleNamespace(schedule=schedule, run_loop=run_loop)
        self.globals = {"__name__": "__main__", "__file__": example}
        code = compile(open(example).read(), example, "exec")
        try:
            exec(code, self.globals)
        except StopRecording:
            pass
        finally:
            sys.modules["time"] = real_time

        if not self.measure:
            self.finish()

    def finish(self):
        self.ffmpeg.stdin.close()
        self.ffmpeg.wait()
        self.browser.close()
        self.playwright.stop()

        # Loop seam: difference between the last and first frames, compared with consecutive frames
        seam = np.abs(self.first - self.last).mean()
        print(
            f"{self.video}: {self.frames} frames, loop seam {seam:.2f} "
            f"(between consecutive frames: {np.mean(self.steps):.2f} average, {np.max(self.steps):.2f} max)"
        )

        gif = self.settings.get("gif")
        if gif:
            width, fps = gif
            output = os.path.join(repository, self.video)
            subprocess.run(
                [imageio_ffmpeg.get_ffmpeg_exe(), "-v", "error", "-y", "-i", output, "-vf",
                 f"fps={fps},scale={width}:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=128[p];[b][p]paletteuse=dither=bayer",
                 output.replace(".mp4", ".gif")],
                check=True,
            )


def measure(video: str) -> list:
    # Runs the example in another process, without rendering, to know where things are during the recording
    with tempfile.NamedTemporaryFile(suffix=".json") as output:
        subprocess.run(
            [sys.executable, os.path.abspath(__file__), "--measure", output.name, video], capture_output=True, check=True
        )
        return json.load(open(output.name))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("videos", nargs="*", help="videos to record (default: all)")
    parser.add_argument("--cpu", action="store_true", help="software rendering (slower)")
    parser.add_argument("--measure", help=argparse.SUPPRESS)
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()

    videos = json.load(open(os.path.join(repository, "recording", "videos.json")))

    if args.measure:
        recorder = Recorder(args.videos[0], videos[args.videos[0]], True, False, args.width, args.height, args.fps)
        recorder.run()
        json.dump(np.array(recorder.points).tolist(), open(args.measure, "w"))
    else:
        for video in args.videos or list(videos):
            # Each video is recorded in its own process (examples have global state)
            if len(args.videos) == 1:
                recorder = Recorder(video, videos[video], False, not args.cpu, args.width, args.height, args.fps)
                recorder.run()
            else:
                subprocess.run([sys.executable, os.path.abspath(__file__), video] + (["--cpu"] if args.cpu else []))
