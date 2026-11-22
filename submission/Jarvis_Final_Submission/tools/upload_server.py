"""Local-only ZIP runner for the final estimator demo.

Only a degraded IMU CSV is read from an upload. Uploaded Python/code is never
extracted or executed. Optional reference files are opened by the evaluator
after estimation to score and visualize the result.
"""
from __future__ import annotations

import io
import json
import math
import os
from pathlib import Path, PurePosixPath
import subprocess
import sys
import tempfile
import time
import zipfile
from http.server import HTTPServer, SimpleHTTPRequestHandler

WORKSPACE = Path(__file__).resolve().parents[1]
if (WORKSPACE / "src" / "main.py").is_file():
    PACKAGE = WORKSPACE
    STATIC = PACKAGE / "demo"
    WORKSPACE_MODE = False
elif (WORKSPACE / "submission" / "Jarvis_Final_Submission" / "src" / "main.py").is_file():
    PACKAGE = WORKSPACE / "submission" / "Jarvis_Final_Submission"
    STATIC = WORKSPACE / "demo"
    WORKSPACE_MODE = True
else:
    raise SystemExit("Could not locate the final submission's src/main.py")

MAX_UPLOAD = 96 * 1024 * 1024
MAX_EXPANDED = 512 * 1024 * 1024
MAX_MEMBERS = 10_000
MAX_CSV = 384 * 1024 * 1024
MAX_PLOT_POINTS = 1_800


def _classify_zip(archive: zipfile.ZipFile):
    entries = [info for info in archive.infolist() if not info.is_dir()]
    if len(entries) > MAX_MEMBERS:
        raise ValueError("ZIP contains too many files.")
    if sum(info.file_size for info in entries) > MAX_EXPANDED:
        raise ValueError("Expanded ZIP is too large (limit: 512 MB).")
    csvs = [i for i in entries if PurePosixPath(i.filename.replace("\\", "/")).suffix.lower() in (".csv", ".txt")]
    truth = [i for i in csvs if any(x in i.filename.lower() for x in
                                    ("groundtruth", "ground_truth", "ground-truth", "reference"))]
    excluded = {i.filename for i in truth}
    preferred = [i for i in csvs if i.filename not in excluded and (
        i.filename.replace("\\", "/").lower().endswith(("/imu0/data.csv", "/mav0/imu0/data.csv", "/imu/data.csv")) or
        PurePosixPath(i.filename.replace("\\", "/")).name.lower() in ("imu.csv", "degraded.csv", "input.csv"))]
    candidates = preferred or [i for i in csvs if i.filename not in excluded]
    if len(candidates) != 1:
        if not candidates:
            raise ValueError("ZIP must contain a degraded IMU CSV, such as imu0/data.csv.")
        raise ValueError("ZIP contains multiple possible IMU CSVs. Upload one recording per ZIP.")
    imu = candidates[0]
    if imu.file_size > MAX_CSV:
        raise ValueError("IMU CSV is too large (limit: 384 MB).")
    def family(info):
        parts = PurePosixPath(info.filename.replace("\\", "/")).parts
        return tuple(p.lower() for p in parts[:-2])
    same_sequence = [i for i in truth if family(i) == family(imu)]
    refs = same_sequence or (truth if len(truth) == 1 else [])
    reference = refs[0] if len(refs) == 1 else None
    if reference and reference.file_size > MAX_CSV:
        reference = None
    return imu, reference


def _read_member(archive: zipfile.ZipFile, info: zipfile.ZipInfo, destination: Path):
    total = 0
    with archive.open(info) as src, destination.open("wb") as out:
        while True:
            chunk = src.read(1024 * 1024)
            if not chunk:
                break
            total += len(chunk)
            if total > MAX_CSV:
                raise ValueError("A selected CSV exceeds the 384 MB safety limit.")
            out.write(chunk)


def _trajectory_model(estimate_path: Path, imu_path: Path, reference_path: Path | None,
                      metadata: dict, label: str):
    import numpy as np
    sys.path.insert(0, str(PACKAGE / "tools"))
    from evaluate import evaluate, load_truth

    est = np.loadtxt(estimate_path, comments="#", ndmin=2)
    times_all = est[:, 0]
    positions_all = est[:, 1:4]
    stride = max(1, math.ceil(len(est) / MAX_PLOT_POINTS))
    indices = np.unique(np.r_[np.arange(0, len(est), stride), len(est) - 1]).astype(int)
    score = None
    if reference_path is not None:
        score = evaluate(str(reference_path), str(estimate_path), str(imu_path))
        truth_times, truth_positions, _ = load_truth(str(reference_path))
        right = np.searchsorted(truth_times, times_all)
        right = np.clip(right, 1, len(truth_times) - 1)
        match = np.where(np.abs(truth_times[right] - times_all) < np.abs(truth_times[right - 1] - times_all), right, right - 1)
        valid = (np.abs(truth_times[match] - times_all) < .02) & (times_all >= truth_times[0]) & (times_all <= truth_times[-1])
        gt = truth_positions[match[valid]]
        pred = positions_all[valid]
        plot_times = times_all[valid]
        x = pred - pred.mean(axis=0)
        y = gt - gt.mean(axis=0)
        u, _, vt = np.linalg.svd(x.T @ y)
        d = np.eye(3)
        d[-1, -1] = np.linalg.det(vt.T @ u.T)
        rotation = vt.T @ d @ u.T
        aligned = x @ rotation.T + gt.mean(axis=0)
        errors = np.linalg.norm(aligned - gt, axis=1)
        plot_stride = max(1, math.ceil(len(plot_times) / MAX_PLOT_POINTS))
        plot_indices = np.unique(np.r_[np.arange(0, len(plot_times), plot_stride), len(plot_times) - 1]).astype(int)
        shown_estimate = aligned[plot_indices]
        shown_reference = gt[plot_indices]
        shown_times = plot_times[plot_indices]
        shown_errors = errors[plot_indices]
    else:
        shown_estimate = positions_all[indices]
        shown_reference = np.empty((0, 3))
        shown_times = times_all[indices]
        shown_errors = np.empty(0)

    return {
        "label": label,
        "uploaded": True,
        "real": False,
        "rows": int(metadata.get("rows", len(est))),
        "duration": float(metadata.get("duration_s", times_all[-1] - times_all[0])),
        "time": shown_times.tolist(),
        "estimate": shown_estimate.tolist(),
        "reference": shown_reference.tolist(),
        "referencePath": shown_reference.tolist(),
        "errors": shown_errors.tolist(),
        "score": score or {},
        "runtime_ms_per_sensor_second": float(metadata.get("ms_per_sensor_second", 0.0)),
    }


def evaluate_upload(payload: bytes, filename: str):
    if not zipfile.is_zipfile(io.BytesIO(payload)):
        raise ValueError("Please choose a valid ZIP containing one degraded IMU CSV.")
    with zipfile.ZipFile(io.BytesIO(payload)) as archive:
        imu_info, reference_info = _classify_zip(archive)
        with tempfile.TemporaryDirectory(prefix="jarvis-upload-") as tmp:
            temp = Path(tmp)
            imu_path = temp / "input.csv"
            estimate_path = temp / "estimated_trajectory.txt"
            metadata_path = temp / "metadata.json"
            truth_path = None
            _read_member(archive, imu_info, imu_path)
            if reference_info:
                truth_path = temp / ("reference" + PurePosixPath(reference_info.filename).suffix.lower())
                _read_member(archive, reference_info, truth_path)
            command = [sys.executable, str(PACKAGE / "src" / "main.py"), str(imu_path),
                       str(estimate_path), "--metadata", str(metadata_path)]
            try:
                result = subprocess.run(command, capture_output=True, text=True,
                                        timeout=180, cwd=PACKAGE)
            except subprocess.TimeoutExpired as exc:
                raise ValueError("The final estimator exceeded the 180-second processing limit.") from exc
            if result.returncode:
                message = result.stderr.strip().splitlines()
                raise ValueError(message[-1] if message else "The uploaded IMU CSV could not be parsed.")
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            safe_label = Path(filename or "uploaded-recording.zip").stem[:80]
            model = _trajectory_model(estimate_path, imu_path, truth_path, metadata, safe_label)
            has_reference = bool(model["score"])
            msg = (f"Processed {model['rows']:,} IMU poses locally. Scores are computed from the matching reference in this ZIP."
                   if has_reference else
                   f"Processed {model['rows']:,} IMU poses locally. No matching ground-truth file was found; deviation cannot be scored.")
            return {"model": model, "message": msg}


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(STATIC), **kwargs)

    def do_GET(self):
        if self.path == "/api/health":
            body = json.dumps({"ready": True, "local_only": True}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if WORKSPACE_MODE and self.path == "/":
            self.send_response(302)
            self.send_header("Location", "/final/")
            self.end_headers()
            return
        super().do_GET()

    def do_POST(self):
        if self.path != "/api/evaluate":
            self.send_error(404)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            length = 0
        if length <= 0 or length > MAX_UPLOAD:
            self._json(413, {"error": "Upload must be a non-empty ZIP no larger than 96 MB."})
            return
        if self.headers.get_content_type() != "application/zip":
            self._json(415, {"error": "Upload a ZIP file."})
            return
        try:
            payload = self.rfile.read(length)
            if len(payload) != length:
                raise ValueError("The upload ended before the ZIP was fully received.")
            result = evaluate_upload(payload, self.headers.get("X-Filename", "uploaded-recording.zip"))
            self._json(200, result)
        except (ValueError, zipfile.BadZipFile, OSError, RuntimeError) as exc:
            self._json(400, {"error": str(exc)[:500]})
        except Exception as exc:
            self._json(500, {"error": f"Could not evaluate this ZIP ({type(exc).__name__})."})

    def _json(self, status, value):
        body = json.dumps(value, separators=(",", ":"), allow_nan=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)


if __name__ == "__main__":
    host, port = "127.0.0.1", 8765
    server = HTTPServer((host, port), Handler)
    print(f"Jarvis local upload preview: http://{host}:{port}/")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
