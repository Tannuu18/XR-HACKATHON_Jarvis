# Track 3 requirements audit

Source: supplied consolidated track slides 13–17 and official FAQ. The FAQ overrides the older slide's C++ option and clarifies memory and synchronization.

| Requirement | Implementation / evidence |
|---|---|
| Standalone Python estimator | Root `main.py`, Python 3.10+, standard library only |
| Degraded IMU is the sole estimator input | No truth, waypoint, camera, external calibration, or network reads |
| EuRoC gyro-first input | Named-column parser; directory resolver |
| Organizer accel-first degradation schema | Named-column parser and explicit conversion tool |
| TUM text output, eight columns | `estimated_trajectory.txt` default, commented header |
| Unix timestamp seconds | Integer ns to exact decimal seconds |
| One pose per input frame | Streaming validator compares every timestamp and row |
| Five-second blind tail | Invalid sensor frames predicted without appending rows |
| Two-second missing-row dropout | Actual timestamp deltas, no reinserted rows; analytic fixture exercises 2 s |
| Normalized world quaternions | xyzw body-to-world, tested rotation convention |
| At most 8 MB algorithm heap | Separate tracemalloc run, benchmark JSON |
| At most 20 ms per sensor second | Separate uninstrumented wall timing; machine dependent |
| Single CPU thread, offline | No threads or network in estimator |
| ATE / RPE, SE(3) alignment | Local evaluator and reproducible evo commands |
| Source ZIP layout | Package tool validates output and hashes payload |
| Public GitHub resources | Source, docs, test fixture, output, demo, reports, and ZIP tracked |
| 2–3 minute video, max 3 min | Pending recording |
| Final submission PPT | Deferred per team request |
| Submission form link | Not supplied; not submitted |
| Deadline | FAQ says November 22, 2026; exact cutoff time not supplied |

No claim is made about hidden checkpoints, organizer ranking, or guaranteed qualification. The algorithm satisfies the input/output contract on tested files; accuracy remains a research risk.
