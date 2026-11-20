"""Create the exact Track 3 ZIP structure, after validating synchronization."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import zipfile
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from tools.validate_output import validate


if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('imu'); p.add_argument('trajectory'); p.add_argument('--metadata'); p.add_argument('--output',default='submission/Jarvis.zip')
    a=p.parse_args(); validation=validate(a.imu,a.trajectory)
    root=Path(__file__).resolve().parents[1]; target=Path(a.output); target.parent.mkdir(parents=True,exist_ok=True)
    manifest={}
    with zipfile.ZipFile(target,'w',zipfile.ZIP_DEFLATED) as z:
        files=[(Path(a.trajectory),'estimated_trajectory.txt'),(root/'main.py','src/main.py')]
        if a.metadata: files.append((Path(a.metadata),'metadata.json'))
        for source,name in files:
            z.write(source,name); manifest[name]=hashlib.sha256(source.read_bytes()).hexdigest()
        z.writestr('src/README.md','# Jarvis Track 3 entrypoint\n\nRequires Python 3.10+ only; no pip packages.\n\nFrom this src directory run:\n\n```sh\npython main.py /path/to/imu0/data.csv ../estimated_trajectory.txt --metadata ../metadata.json\n```\n\nEuRoC headers are detected. Headerless files need `--column-order gyro-accel` or `accel-gyro`. Nanoseconds, m/s^2 and rad/s are defaults. Run `python main.py --help` for options.\n\nThe supplied trajectory and metadata are a synthetic demonstration, not an organizer evaluation. Replace them with a run on the supplied evaluation CSV. Full source, documentation and limitations: https://github.com/Tannuu18/XR-HACKATHON_Jarvis\n')
        z.writestr('validation.json',json.dumps(validation,indent=2))
        z.writestr('SHA256SUMS.json',json.dumps(manifest,indent=2))
    print(target)
