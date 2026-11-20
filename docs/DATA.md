# Data provenance

## Organizer materials

The team supplied `degrade_imu.py`, `Track3_FAQ_Responses.txt`, the consolidated Track slides, and the final presentation template. The two text/code resources in `resources/official/` are unchanged copies. They retain organizer ownership and are not covered by the project MIT license.

The official degradation seed is 42. Default degradation includes progressive noise, bias random walks, scale random walks, spikes, repeated/stuck frames, removed rows, duplicates, and a zero-valued final five-second failure window. Its first seven columns must be timestamp, acceleration xyz, gyro xyz. EuRoC must be reordered before using it.

## Real benchmark: EuRoC MH_01_easy

- Dataset authors: Michael Burri et al., ETH Zurich Autonomous Systems Lab.
- Official dataset: https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets
- Official archive attempted: https://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip
- The official server reset connections during this session.
- Development mirror: https://github.com/Utkarsh-Sheel-A/Error-State-Kalman-Filter
- Only `data/MH_01_easy/imu0/data.csv` and `data/MH_01_easy/state_groundtruth_estimate0/data.csv` were used. Mirror algorithm code was not reused.
- IMU has 36,820 frames over 184.095 seconds. The unchanged organizer degradation script produces 35,629 frames with default settings in this run.
- Ground truth was used only by evaluation tools after the estimator generated output. No truth bias or velocity columns were used to initialize or tune the estimator.
- Results were iterated on this sequence, so it is a development benchmark, not an untouched holdout.

Dataset files are deliberately excluded from Git. Download them from the original source under its applicable terms. A mirror is not evidence that those terms have changed. Source hashes and mirror commit are in `reports/data_provenance.json`.

## Synthetic fixture

`tools/generate_sample.py` generates the checked-in 40-second fixture. It has a smooth analytic 3D path with changing yaw, initial/final rest, a two-second missing-row gap, isolated spikes, duplicate timestamp, low progressive white noise, and a final five-second all-zero sensor window. Analytic truth lives beside the IMU strictly for evaluation.

It is a designed test case, not EuRoC, TUM-VI, organizer evaluation data, or evidence of broad generalization. Its final rest makes blind prediction easier than continuing motion would be. Additional tests exercise unit conventions and known constant acceleration.
