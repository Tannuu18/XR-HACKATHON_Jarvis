"""Convert named EuRoC gyro-first input to organizer's accel-first schema."""
import argparse
import csv
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from main import rows

if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('input'); p.add_argument('output'); a=p.parse_args()
    with open(a.output,'w',newline='') as f:
        w=csv.writer(f); w.writerow(['timestamp','accel_x','accel_y','accel_z','gyro_x','gyro_y','gyro_z'])
        for stamp,t,acc,gyr in rows(a.input,'auto','ns','ms2',False):
            sec,ns=stamp.split('.'); w.writerow([int(sec)*10**9+int(ns),*acc,*gyr])
