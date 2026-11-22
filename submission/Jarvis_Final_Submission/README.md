# Jarvis — final Track 3 submission candidate

**One final estimator. Python 3.10+; no runtime pip packages.**

From the extracted folder run:

```sh
python src/main.py input.csv estimated_trajectory.txt --metadata metadata.json
```

This defaults to the improved physics-based baseline: centered spike repair,
interval integration, input-derived sensor-bias calibration, rest-aware drift
protection and offline smoothing. There are no older estimator options or model
weight files. Named EuRoC gyro-first and organizer accel-first headers are
detected. Headerless files need --column-order gyro-accel or accel-gyro. Input
defaults to nanoseconds, m/s² and rad/s; output is seconds, xyz, xyzw.

The included estimated_trajectory.txt and metadata.json are a labelled public
V2_03 development example. Rerun on the organizer's actual input if supplied.
There is exactly one normalized 6-DoF pose per provided input row.

## Preview and verification

For the preview with ZIP upload, install the optional evaluator dependency and
start the local server:

```sh
python -m pip install -r requirements-evaluation.txt
python tools/upload_server.py
```

Open http://127.0.0.1:8765/ . Upload one ZIP containing one degraded IMU CSV,
such as `imu0/data.csv`. An optional matching ground-truth CSV/TUM file enables
error scoring. Uploaded source code is never executed; only the packaged final
estimator runs. Files are processed locally and discarded after the request.
You can also open demo/index.html directly to inspect the included recordings;
ZIP upload requires the local server. Results and limitations are in
docs/VALIDATION.md and docs/current_results.json.

```sh
python -m unittest discover -s tests
python tools/validate_output.py input.csv estimated_trajectory.txt
python tools/benchmark.py input.csv
```

Optional offline evaluation: install NumPy, then run
`python tools/evaluate.py reference.csv estimated_trajectory.txt --imu input.csv`.
The estimator itself has no NumPy dependency and uses a single CPU thread.

The package contains only the final source, viewer, tests, resources and results.
The nine-slide presentation, built from the supplied hackathon template, is in
`presentation/Jarvis_Track3_Submission.pptx`.
AI use is disclosed in `docs/AI_DISCLOSURE.md`; APK/SDK is marked not applicable
in `APK_SDK_NOT_APPLICABLE.md`. The form has not been submitted because its URL
was not provided. No ranking, hidden-checkpoint result or accuracy percentage is
claimed.

Project: https://github.com/Tannuu18/XR-HACKATHON_Jarvis
