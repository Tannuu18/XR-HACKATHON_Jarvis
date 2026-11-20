"""Strict organizer-format validation, streaming with constant memory."""
import argparse
from decimal import Decimal
import json
import math
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from main import rows,resolve_input


def validate(imu,trajectory,order='auto',unit='ns'):
    n=0; max_norm_error=0.
    with open(trajectory) as f:
        poses=(line.split() for line in f if line.strip() and not line.startswith('#'))
        for stamp,_,_,_ in rows(resolve_input(imu),order,unit,'ms2',False):
            pose=next(poses,None)
            if pose is None: raise ValueError('Missing output rows')
            if len(pose)!=8: raise ValueError(f'Row {n+1}: expected eight columns')
            if Decimal(pose[0])!=Decimal(stamp): raise ValueError(f'Row {n+1}: timestamp mismatch')
            values=list(map(float,pose))
            if not all(math.isfinite(x) for x in values): raise ValueError(f'Row {n+1}: non-finite output')
            err=abs(math.sqrt(sum(x*x for x in values[4:]))-1)
            if err>1e-6: raise ValueError(f'Row {n+1}: non-unit quaternion')
            max_norm_error=max(max_norm_error,err); n+=1
        if next(poses,None) is not None: raise ValueError('Extra output rows')
    if n==0: raise ValueError('No rows')
    return {'passed':True,'rows':n,'timestamps':'exact decimal match','max_quaternion_norm_error':max_norm_error}


if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('imu'); p.add_argument('trajectory'); p.add_argument('--column-order',default='auto'); p.add_argument('--ts-unit',default='ns'); p.add_argument('--output')
    a=p.parse_args(); text=json.dumps(validate(a.imu,a.trajectory,a.column_order,a.ts_unit),indent=2); print(text)
    if a.output: Path(a.output).write_text(text+'\n')
