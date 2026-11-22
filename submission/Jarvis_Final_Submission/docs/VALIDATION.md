# Final estimator validation

This package contains one final physics-based estimator. It reads degraded IMU
only. Browser references and evaluation ground truth never enter the estimator.

Ten public EuRoC recordings were used for development and validation, with the
unchanged organizer degrader. Scores use rigid SE(3) alignment with no scale
fitting. They are not an untouched organizer test or an accuracy percentage.

| Recording | Trajectory RMSE | Sensor-outage RMSE |
|---|---:|---:|
| MH_01_easy | 3.946 m | 5.175 m |
| MH_02_easy | 3.716 m | 6.259 m |
| MH_03_medium | 3.301 m | 2.906 m |
| MH_04_difficult | 6.657 m | 4.350 m |
| MH_05_difficult | 6.686 m | 4.016 m |
| V1_02_medium | 1.520 m | 0.751 m |
| V1_03_difficult | 1.489 m | 1.258 m |
| V2_01_easy | 2.080 m | 3.181 m |
| V2_02_medium | 1.924 m | 2.318 m |
| V2_03_difficult | 1.990 m | 2.617 m |

Median per-recording trajectory error is 2.690 m; mean is 3.331 m. Some cases
retain substantial error, including MH_01 at 3.946 m and MH_04/MH_05 above 6 m.
Outage scores cover available matching reference timestamps, which may be
shorter than the full five-second outage. Full numerical results are in
current_results.json. These are local measurements, not organizer scores.

The final algorithm estimates gyro bias with a robust quiet-window fallback,
uses rest-aware drift protection and bounded-memory offline smoothing. Quiet
detection and the velocity prior are assumptions and can suppress actual motion.
Future motion during sensor failure cannot be uniquely reconstructed.

Run the packaged tests with `python -m unittest discover -s tests`.
Main execution needs only Python 3.10+. The optional evaluator needs NumPy;
the organizer degradation script also needs pandas. Full raw real data is not
redistributed. Dataset attribution: Michael Burri et al., ETH Zurich EuRoC MAV.
Author mirror and archive hashes are in data_provenance.json. Original dataset
and organizer terms still apply to their resources.
