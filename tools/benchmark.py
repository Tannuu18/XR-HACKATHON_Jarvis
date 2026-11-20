"""Reproducible isolated timing, separate heap profile, and format validation."""
import argparse
import json
import os
import platform
from pathlib import Path
import statistics
import subprocess
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from tools.validate_output import validate


if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('imu'); p.add_argument('--name',default='benchmark'); p.add_argument('--runs',type=int,default=3)
    a=p.parse_args(); root=Path(__file__).resolve().parents[1]; dest=root/'reports'; dest.mkdir(exist_ok=True)
    trajectory=dest/f'{a.name}.txt'; meta=dest/f'{a.name}_runtime.json'
    command=[sys.executable,str(root/'main.py'),a.imu,str(trajectory),'--metadata',str(meta)]
    runs=[]
    for i in range(a.runs):
        subprocess.run(command,check=True,stdout=subprocess.DEVNULL); runs.append(json.loads(meta.read_text()))
    subprocess.run(command+['--profile-memory'],check=True,stdout=subprocess.DEVNULL)
    memory=json.loads(meta.read_text()); meta.write_text(json.dumps(runs[-1],indent=2)+'\n')
    (dest/f'{a.name}_memory.json').write_text(json.dumps(memory,indent=2)+'\n')
    summary={'timing_runs':runs,'median_ms_per_sensor_second':statistics.median(r['ms_per_sensor_second'] for r in runs),
             'peak_algorithm_python_heap_bytes':memory['peak_python_heap_bytes'],
             'memory_profile_wall_time_excluded_from_speed_claim':True,
             'runtime_budget_pass':all(r['ms_per_sensor_second']<=20 for r in runs),
             'memory_budget_pass':memory['peak_python_heap_bytes']<=8_000_000,
             'cpu':platform.processor(),'os':platform.platform(),'python':sys.version,
             'format':validate(a.imu,trajectory)}
    (dest/f'{a.name}_benchmark.json').write_text(json.dumps(summary,indent=2)+'\n'); print(json.dumps(summary,indent=2))
