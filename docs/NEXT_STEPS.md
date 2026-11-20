# Before final competition submission

1. Validate on multiple unseen EuRoC and TUM-VI recordings, including long constant-velocity travel and a blind tail during a turn. The current single real sequence is insufficient evidence for a top-12 claim.
2. Compare the configurable free-integration and regularized modes on held-out data. Avoid selecting parameters using hidden evaluation truth. The zero-mean velocity prior can erase slow displacement, and stationary detection can mistake steady travel for rest.
3. Confirm resource limits on organizer-like hardware at the actual sample rate and maximum recording duration. Python heap profiling is separate from timing. Disk usage is linear in the recording length.
4. Rebuild the submission ZIP from the actual supplied evaluation IMU, not the included synthetic example. Validate exact row count and timestamps again.
5. Record a 2–3 minute walkthrough, with 3 minutes as the maximum. Show an actual CLI run, row validation, resource figures, trajectory viewer, and limitations. Do not label synthetic performance as real-data accuracy.
6. Complete the official PPT template with team/college/member details and verified results. Deferred by the team while the project is built.
7. Verify the GitHub repository is publicly accessible while signed out. The user authorized pushing to their supplied repository.
8. Obtain the submission form from the official announcements channel and submit the public repository URL. The FAQ gives November 22, 2026 as the deadline but no exact cutoff time.
