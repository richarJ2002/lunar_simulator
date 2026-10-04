# Post-processing

> Code is truth — this page describes; the scripts and run layout on `main` win on conflict.

Turn one captured test run's bag and logs into a portable, interactive
report site. Source tool and generated site share a name but live in
different places — do not mix them up.

Where: `post_processing/` source-tree tool (not a ROS executable)
([GitHub](https://github.com/richarJ2002/lunar_simulator/blob/main/post_processing/)).
Generated site: `test_runs/<run>/post_processing/` alongside that run's
other artifacts (never committed; never embed bag output here).

Core telemetry: every launch auto-records to
`test_runs/<run>/ros/bags/localisation` with `manifest.json` alongside
it. The launcher exits nonzero rather than completing a run whose
recorder failed to start or crashed immediately.

## Generate

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m pip install -r post_processing/requirements.txt
python3 post_processing/post_processing.py --test-run test_runs/<run>
```

Each subsystem script also renders its own page plus shared assets (run
any script with `-h` for `--output-dir`, `--bag`, and image-frame flags;
defaults point at the run's own `post_processing/` and
`ros/bags/localisation`).

## Images are opt-in

`./scripts/launch_simulator.sh --record-images` additionally records the
LocCam / annotated-feature image topics. Leave it off for normal runs:
stereo images at full camera rate add multiple gigabytes to a run.
Without it the visual-odometry page still renders every other section
and says how to recapture with images.
