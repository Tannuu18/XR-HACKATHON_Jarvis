# Jarvis — Samsung PRISM XR Hackathon 2026, Track 3

The final submission package is [`Jarvis_Final_Submission.zip`](submission/Jarvis_Final_Submission.zip). The extracted version is in [`submission/Jarvis_Final_Submission`](submission/Jarvis_Final_Submission/README.md).

## Run the estimator

Extract the ZIP and run, using the supplied degraded IMU CSV:

```powershell
python src/main.py input.csv estimated_trajectory.txt --metadata metadata.json
```

The final estimator is Python 3.10+ and uses only the standard library at inference. It writes one TUM-format pose per IMU input row. See the package README and `docs/VALIDATION.md` for input details, results, limitations, and commands.

## Preview and ZIP evaluation

From the extracted package, run:

```powershell
python -m pip install -r requirements-evaluation.txt
python tools/upload_server.py
```

Open `http://127.0.0.1:8765/` and upload a ZIP with one degraded IMU CSV. An optional ground-truth file enables error scoring. This runs the packaged estimator locally; uploaded code is never executed. The public GitHub files do not provide a hosted Python upload service.

The bundled trajectories are labelled public EuRoC development examples, not organizer results. Scores use rigid SE(3) alignment without scale correction; remaining errors and limitations are reported in the validation document. No ranking or hidden-test result is claimed.

## Presentation and form

The filled presentation and form submission are not included. The team member and college details and the form URL were not provided in this workspace.

Project: <https://github.com/Tannuu18/XR-HACKATHON_Jarvis>
