# Method and observability

The accelerometer measures specific force, not world acceleration. With a body-to-world rotation R, the propagation uses `a_world = R (a_measured - b_a) + (0, 0, -9.80665)`. Gyro increments right-multiply the body-to-world quaternion. Position and velocity are integrated with the midpoint attitude. Quaternion normalization is maintained at every rotation update.

## Initialization

The first second is examined with bounded storage (at most 1,000 samples). Low acceleration variance, plausible gravity magnitude, and low angular velocity support an initial rest assumption. Gyro mean is then a bias estimate. Only the radial component of accelerometer bias is inferred from gravity magnitude. Full accelerometer bias is not observable at a single stationary orientation.

For moving starts, the estimator scans fixed-size windows for a quieter later interval. A second gyro pass estimates the relative rotation to that interval, allowing the gravity-aligned attitude to be transferred back to the start. Absolute yaw remains unobservable. If no quiet window exists, the early acceleration direction is a weak fallback and may include translational acceleration.

## Motion and sensor faults

A nine-sample component median history identifies large residual spikes. This causal filter can reject genuine abrupt manoeuvres, so thresholds are explicit CLI arguments. It does not claim to recover arbitrary corruption. Repeated nonzero values from short stuck intervals are integrated as held measurements; this is a limitation, not a fully repaired signal.

Missing CSV rows manifest as timestamp gaps. Those intervals are integrated in steps no longer than 20 ms, using the previous force/rate over the gap and the new measurement at the final substep. No synthetic timestamps are emitted. Very long gaps have large unobservable motion uncertainty and potentially high compute cost; the official two-second case is the intended operating range.

An all-zero acceleration or any non-finite sensor component marks an invalid sensor row. During invalid intervals, recent world acceleration and angular velocity decay exponentially with a configurable 0.7-second time constant. Integrals of the decaying acceleration are evaluated analytically. Every invalid input row still receives a pose.

## Offline drift correction

Low-variance, low-rate sustained intervals supply **inferred**, not observed, zero-velocity constraints. Body motion at constant velocity is indistinguishable from rest, so these constraints can be wrong.

Without the velocity prior, residual velocity at successive inferred rest anchors is removed with a linear velocity bridge. The first bridge instead estimates a constant unknown initial velocity. Position corrections are continuous at anchor boundaries. Anchor records are streamed through a temporary file.

With prior time scale tau, let u be the propagated velocity. The offline solve minimizes:

`sum_i ||(v_i-v_(i-1)) - (u_i-u_(i-1))||² / dt_i + sum_i dt_i ||v_i||² / tau²`

Quiet intervals add a strong zero-velocity penalty. This produces a symmetric tridiagonal system for each velocity component. Forward elimination writes four doubles per sample to disk; backward substitution writes three doubles per sample. A final streaming pass integrates corrected velocities and retains the estimated orientation. Duplicate timestamps use a 1 µs numerical weight in the solve, while their position integration uses zero elapsed time. Output timestamps remain exact.

This is a motion prior that suppresses low-frequency velocity; it does not make position observable. A smaller tau gives more drift suppression and more displacement attenuation. It can also influence motion within the blind tail. Automatic selection disables this prior when initialization detected a quiet start, where a rest-based bridge is better justified; otherwise it selects tau=1 s to limit uncertain initial-velocity and bias drift. This choice has only limited real-data validation.

## Complexity and separation

CPU is O(N + total_gap_duration / 0.02 s); RAM is O(1) with fixed buffers; temporary disk space is O(N). No thread pools, network access, model downloads, external calibration, or truth data appear in the estimator. `tools/evaluate.py` alone reads truth. The estimator does not consume metadata emitted by the degradation script.

## Fundamental limits

An IMU alone cannot uniquely recover global translation, global yaw, unknown initial velocity, constant-velocity segments, arbitrary bias drift, or actual motion while sensors are dead. SE(3) alignment removes global rotation and translation but does not remove velocity error or scale error. A blind trajectory is a prediction under a motion prior, not a recovered measurement. Accuracy on unseen missions therefore requires broader validation and cannot be guaranteed.
