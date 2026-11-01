# PlaCo examples

You can find here examples of PlaCo usage.

* [PlaCo documentation](https://placo.readthedocs.io/)
* [PlaCo library](https://github.com/rhoban/placo)
## Videos

The videos shown in the documentation (`*/videos/`) are recorded from the viser viewer with:

```bash
pip install playwright imageio-ffmpeg
playwright install chromium
python recording/record.py                            # all the videos
python recording/record.py kinematics/videos/6axis_trajectory.mp4   # one video
```

Each example runs on a simulated clock and a headless browser renders one frame every 1/30 s of simulated time,
so the videos don't depend on the speed of the machine. The settings of each video (example, arguments, start time,
duration, camera) are in `recording/videos.json`. Videos of periodic motions last exactly one period, so that they
loop without a visible seam. Rendering uses the GPU (pass `--cpu` for software rendering, much slower).
