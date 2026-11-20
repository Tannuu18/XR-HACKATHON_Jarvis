# Jarvis Path Finder

Samsung PRISM XR Hackathon 2026, **Track 3: trajectory estimation under sensor degradation**.

An IMU-only Python estimator that writes one synchronized 6-DoF TUM pose for every input row, including duplicate timestamps and invalid readings during the final blind window. The evaluator, benchmark tools, and examples are separate from the estimator.

**Status:** working research baseline with measured resource use and real EuRoC evaluation. This is not a claim of winning accuracy. Hidden-dataset performance and TUM-VI accuracy have not been established. The presentation and recorded walkthrough are deferred at the team's request.

## Run in 30 seconds

Python 3.10 or newer. **No pip packages are needed to run the estimator.**

```sh
python main.py examples/synthetic/imu.csv estimated_trajectory.txt --metadata metadata.json
python tools/validate_output.py examples/synthetic/imu.csv estimated_trajectory.txt
```

For organizer data:

```sh
python main.py path/to/imu0/data.csv estimated_trajectory.txt --metadata metadata.json
# Also accepts a sequence directory containing imu0/data.csv or mav0/imu0/data.csv.
```

Input is a CSV, not the dataset ZIP. Extract the organizer archive before running. Output is **space-separated** `timestamp tx ty tz qx qy qz qw`, with seconds, metres, and normalized body-to-world xyzw quaternions. Exact integer nanosecond timestamps are rendered as decimal seconds without converting them through a floating-point epoch first. Dropped input rows are not reinserted. No poses are appended after the final input timestamp.

Headers identify either EuRoC **gyro-first** columns or the organizer degradation script's **acceleration-first** columns. For a headerless CSV, explicitly select its layout:

```sh
python main.py input.csv --column-order gyro-accel
python main.py input.csv --column-order accel-gyro --ts-unit s
```

Defaults are ns, m/s², and rad/s. Options `--ts-unit s|ms|us|ns`, `--accel-unit g|ms2`, and `--gyro-deg` support other known units. Units are explicit because inferring them from motion can silently corrupt an estimate.

## How it works

1. Detect a quiet interval for gravity and gyro bias calibration. If the recording starts in motion, find a later quiet window and propagate its attitude back to the start using the recorded gyro. Without such a window, initialization remains uncertain.
2. Reject isolated large residuals with a nine-sample median history. Propagate body-to-world attitude on SO(3), rotate specific force, subtract gravity, and integrate at interval midpoints.
3. Use actual timestamp deltas through missing rows. Detect sustained quiet intervals, apply a zero-velocity prior, slowly update gyro bias, and correct gravity tilt.
4. On invalid/zero sensor rows, use damped acceleration and angular-rate prediction. Constant velocity remains the limiting model as predicted acceleration decays.
5. Refine position offline. With a quiet start, bridge velocity drift between inferred rests using temporary disk storage. With a moving start, use a regularized velocity solve to limit poorly observed low-frequency drift. Its tridiagonal elimination and backward substitution use constant-size RAM buffers.

See [the method and its assumptions](docs/METHOD.md). Neither the algorithm nor its calibration reads ground truth, waypoints, camera data, sensor calibration files, learned weights, or a sequence-specific route.

### Motion-prior controls

```sh
# Free inertial integration plus inferred-rest velocity bridges
python main.py input.csv --velocity-prior 0
# Explicit offline velocity prior; larger time scales preserve slower motion
python main.py input.csv --velocity-prior 5
# Disable inferred zero-velocity updates as well
python main.py input.csv --velocity-prior 0 --no-zupt
```

Automatic policy: a quiet initial interval selects `--velocity-prior 0`; a moving start selects `1` second. This policy depends on the IMU, not a dataset name. A zero-mean velocity prior **can shrink real sustained displacement**. Quiet IMU readings also cannot distinguish rest from constant-velocity travel. Disable or adjust these priors when their assumptions do not fit the mission.

## Validation and benchmarks

```sh
python -m unittest discover -s tests -v
python tools/benchmark.py examples/synthetic/imu.csv --name synthetic
python tools/validate_output.py examples/synthetic/imu.csv reports/synthetic.txt
```

The benchmark runs three uninstrumented timings and a **separate** `tracemalloc` heap profile. Profiling slows Python substantially; its wall time is never used to claim compliance with the 20 ms budget. Algorithm heap is compared against a conservative decimal 8,000,000 bytes. Python interpreter RSS is not this measurement, consistent with the official FAQ.

Measured results and machine details are in [`reports/`](reports/). See [the benchmark report](docs/RESULTS.md) for accuracy, comparisons, and gaps. These local results do not guarantee organizer-hardware timing or hidden-data accuracy. The estimator uses one Python thread and no numerical libraries. Temporary disk usage is O(N); algorithm RAM is bounded independently of sequence length.

### Reproduce the real EuRoC experiment

Install evaluation dependencies only:

```sh
python -m pip install -r requirements.txt
python tools/fetch_euroc.py MH_01_easy
python tools/prepare_imu.py data/euroc/MH_01_easy/mav0/imu0/data.csv examples/mh01_accel_first.csv
python resources/official/degrade_imu.py examples/mh01_accel_first.csv examples/mh01_degraded.csv --meta reports/mh01_degradation.json
python tools/benchmark.py examples/mh01_degraded.csv --name mh01_degraded
python tools/evaluate.py data/euroc/MH_01_easy/mav0/state_groundtruth_estimate0/data.csv reports/mh01_degraded.txt --output reports/mh01_degraded_accuracy.json
```

The fetcher retrieves only IMU and truth ZIP entries by HTTP ranges, not images. It refuses a server that ignores Range. If the official server is unavailable, use an existing EuRoC download; [data provenance](docs/DATA.md) records the public mirror used for this development run. Real raw data and ground truth are not republished here; generated results are included. The synthetic test fixture is provided in full and is clearly distinguished from real benchmark data.

**Important schema mismatch:** `degrade_imu.py` interprets columns 2–4 as acceleration. Do not run it directly on gyro-first EuRoC data. `prepare_imu.py` performs the required conversion. The supplied degradation script is preserved unchanged.

The local evaluator uses rigid SE(3) alignment without scale correction, 20 ms timestamp association, position ATE, and 1-second relative pose errors. The tail metric is a five-second tail over associated truth samples; it is not a hidden checkpoint score. For the official scorer, also run evo:

```sh
evo_ape euroc path/to/state_groundtruth_estimate0/data.csv reports/mh01_degraded.txt --align
evo_rpe euroc path/to/state_groundtruth_estimate0/data.csv reports/mh01_degraded.txt --delta 1 --delta_unit s
```

## Demo and submission package

Open `demo/index.html` locally in a browser. It works offline, loads the bundled synthetic example, and accepts a local TUM trajectory file. The viewer does not run the estimator or feed data to it.

```sh
python tools/package_submission.py examples/synthetic/imu.csv reports/synthetic.txt --metadata reports/synthetic_runtime.json
```

`submission/Jarvis.zip` contains the required root trajectory, optional metadata, `src/main.py`, `src/README.md`, and validation/hashes. The included ZIP is a **synthetic demonstration bundle**, not an organizer evaluation result. Rebuild it for the supplied evaluation CSV before final submission.

The repository is intended to be public. The submission form URL was not provided; no form submission has been made. See [the requirements checklist](docs/REQUIREMENTS.md) and [the remaining competition work](docs/NEXT_STEPS.md).

## Attribution

Original estimator and tooling: Jarvis team. Organizer material in `resources/official/` retains its original provenance and is excluded from the project's MIT license. EuRoC is by Burri et al., ETH Zurich ASL; TUM-VI is by Schubert et al., TUM. Dataset terms continue to apply. See [data sources](docs/DATA.md).
