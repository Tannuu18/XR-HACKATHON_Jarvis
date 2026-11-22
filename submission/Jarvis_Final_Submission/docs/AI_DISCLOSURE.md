# AI use disclosure

The team used AI assistance during development and submission preparation. AI
tools helped draft and revise estimator code, debug implementation issues,
prepare documentation and validation summaries, and assemble presentation
content. Team members remain responsible for checking the code, data handling,
measurements, claims, and final submission.

The submitted estimator does not call an AI model or network service at
inference time. It uses deterministic, physics-based inertial integration,
sensor calibration, rest-aware drift control, and offline smoothing. Evaluation
ground truth is used only by separate scoring and visualization tools; it is
not an estimator input.
