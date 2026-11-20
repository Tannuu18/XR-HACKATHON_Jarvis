# Measured results

These are local development measurements, not an organizer score. SE(3) alignment uses no scale correction. No hidden waypoints are available. The estimator sees only the degraded IMU.

## Accuracy

| Data and mode | ATE RMSE, m | 1 s translation RPE, m | 1 s rotation RPE, degrees | Associated final 5 s RMSE, m |
|---|---:|---:|---:|---:|
| EuRoC MH_01_easy, organizer default degradation, auto prior | 3.988 | 0.450 | 0.572 | 5.809 |
| Same degraded data, free integration plus rest bridges | 1240.028 | 41.951 | 0.572 | 3795.832 |
| Same degraded timestamps, stationary identity-pose sanity baseline | 4.323 | 0.496 | 9.015 | 4.949 |
| Synthetic 3D fixture, auto rest-bridge mode | 0.387 | 0.091 | 0.207 | 0.376 |

The regularizer prevents catastrophic long-horizon position drift on MH_01, but the position improvement over a stationary sanity baseline is modest (about 7.7% in ATE), and its tail error is worse. Orientation RPE is substantially better than the identity baseline. These results do **not** justify a claim of competition-winning accuracy. The synthetic case is easier, particularly because it stops before the blind tail.

The real sequence was used during development and parameter experimentation. It is not an untouched holdout. TUM-VI, other EuRoC sequences, and hidden organizer data remain unvalidated. A static single-orientation interval cannot identify full accelerometer bias, which is a major remaining source of drift.

The evaluator associates to the nearest ground-truth timestamp within 20 ms, excluding times outside truth coverage. MH_01 has 35,190 associated degraded frames out of 35,629 input rows. Its five-second tail is measured over associated truth samples and therefore may not exactly equal the organizer's full failure window. The tool is independently implemented; evo commands are provided in the README but evo itself has not been run here.

## Efficiency

| Dataset | Sensor duration | Input/output rows | Median runtime, ms per sensor second | Peak traced algorithm heap |
|---|---:|---:|---:|---:|
| Degraded MH_01_easy | 184.095 s | 35,629 | 9.608 | 531,379 bytes |
| Synthetic fixture | 40.000 s | 7,603 | 9.758 | 521,153 bytes |

Limits are 20 ms per sensor second and 8 MB algorithm heap. Both tested files pass locally. All three uninstrumented timing runs pass; full run distributions, Python version, OS, and CPU identifier appear in `reports/*_benchmark.json`. CPU: Intel64 Family 6 Model 151 Stepping 2, Windows build 26200, Python 3.10.0. Wall time includes CSV parsing, initialization passes, trajectory estimation, temporary-file smoothing, and output serialization; excludes interpreter startup and metadata serialization. `tracemalloc` runs separately and is excluded from the speed claim.

The RAM claim concerns traced Python algorithm heap, as defined by the FAQ. It does not claim whole-process RSS below 8 MB. The standard-library-only estimator allocates no NumPy model or native numerical array. Operating-system disk cache and interpreter memory are excluded. Temporary files grow with recording length.

## Contract checks

All tested trajectories have exactly one pose per input row, including duplicates and invalid sensor rows. The validator compares timestamp decimals exactly and requires eight finite columns and unit quaternions. Maximum quaternion norm error is below 1e-9 on the measured runs. The synthetic fixture physically removes a two-second block and retains the final five-second failure timestamps. No output timestamps are inserted or appended.

The automated tests cover arbitrary gravity alignment, quaternion direction, known acceleration, units, duplicate/gap/blind handling, malformed and unsorted input, exact epoch-nanosecond rendering, all-invalid rejection, and the velocity prior's constant-velocity ambiguity.
