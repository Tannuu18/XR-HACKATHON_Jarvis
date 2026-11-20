"""Analytic rotating, stop-and-go 3D sample. Truth is evaluation-only.

The final five seconds are invalid sensor frames, a two-second interval is
physically missing, and duplicate timestamps and isolated spikes are included.
This is a deterministic test fixture, not a EuRoC/TUM-VI benchmark result.
"""
import csv
import math
from pathlib import Path
import random


def generate(destination='examples/synthetic'):
    dest=Path(destination); dest.mkdir(parents=True,exist_ok=True); rng=random.Random(42)
    imu=dest/'imu.csv'; truth=dest/'groundtruth.txt'
    with imu.open('w',newline='') as fi,truth.open('w') as ft:
        writer=csv.writer(fi); writer.writerow(['timestamp','accel_x','accel_y','accel_z','gyro_x','gyro_y','gyro_z'])
        ft.write('# timestamp tx ty tz qx qy qz qw\n')
        for i in range(8001):
            t=i/200.; start=1403636579000000000; stamp=start+i*5000000
            if t<3 or t>33:
                u=0 if t<3 else 1.; du=ddu=0.
            else:
                s=(t-3)/30; u=10*s**3-15*s**4+6*s**5
                du=(30*s*s-60*s**3+30*s**4)/30
                ddu=(60*s-180*s*s+120*s**3)/900
            p=(8*u,3*math.sin(2*math.pi*u),1.5*(1-math.cos(2*math.pi*u)))
            aw=(8*ddu,3*(2*math.pi*math.cos(2*math.pi*u)*ddu-(2*math.pi*du)**2*math.sin(2*math.pi*u)),
                1.5*(2*math.pi*math.sin(2*math.pi*u)*ddu+(2*math.pi*du)**2*math.cos(2*math.pi*u)))
            yaw=math.pi*u; rate=math.pi*du; c,s=math.cos(yaw),math.sin(yaw)
            body=(c*aw[0]+s*aw[1],-s*aw[0]+c*aw[1],aw[2]+9.80665)
            ft.write(f'{stamp//10**9}.{stamp%10**9:09d} '+' '.join(map(str,(*p,0,0,math.sin(yaw/2),math.cos(yaw/2))))+'\n')
            if 18<t<20: continue
            severity=t/40
            a=[x+rng.gauss(0,.01)*severity for x in body]
            w=[rng.gauss(0,.0005)*severity,rng.gauss(0,.0005)*severity,rate+rng.gauss(0,.0005)*severity]
            if i in (1800,4200): a[0]+=6; w[1]+=1
            if t>=35: a=w=[0.,0.,0.]
            row=[stamp,*a,*w]; writer.writerow(row)
            if i==1000: writer.writerow(row)
    print(imu)


if __name__=='__main__': generate()
