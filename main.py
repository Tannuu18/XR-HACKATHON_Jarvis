#!/usr/bin/env python3
"""Jarvis Path Finder. Standard-library-only, bounded-memory IMU navigation.

Quaternion convention: body to world, xyzw. World gravity is (0,0,-g).
No ground truth, waypoint, camera, or network input is used by this program.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import statistics
import sys
import time
import tempfile
import struct
from collections import deque

G = 9.80665


def norm(v):
    return math.sqrt(sum(x*x for x in v))


def mul(a, b):
    x,y,z,w = a; X,Y,Z,W = b
    return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)


def unit(q):
    n = norm(q)
    return tuple(x/n for x in q)


def rotate(q, v):
    x,y,z,w = q; a,b,c = v
    tx,ty,tz = 2*(y*c-z*b),2*(z*a-x*c),2*(x*b-y*a)
    return (a+w*tx+y*tz-z*ty, b+w*ty+z*tx-x*tz, c+w*tz+x*ty-y*tx)


def advance(q, omega, dt):
    a = norm(omega)*dt
    s = math.sin(a/2)/norm(omega) if a > 1e-12 else dt/2
    return unit(mul(q, (*[w*s for w in omega], math.cos(a/2))))


def align_gravity(a):
    n = norm(a)
    if n < 1e-8:
        raise ValueError('No usable acceleration for gravity initialization')
    u = [x/n for x in a]
    return (1.,0.,0.,0.) if u[2] < -.999999 else unit((u[1],-u[0],0.,1+u[2]))


def timestamp_seconds(token, scale):
    # Integer subtraction before float conversion preserves epoch nanoseconds.
    if scale == 1e-9 and token.strip().lstrip('-+').isdigit():
        value = int(token); sec, ns = divmod(value, 1000000000)
        return sec + ns*1e-9
    return float(token)*scale


def timestamp_text(token, scale):
    if scale == 1e-9 and token.strip().lstrip('-+').isdigit():
        sec, ns = divmod(int(token), 1000000000)
        return f'{sec}.{ns:09d}'
    return f'{float(token)*scale:.9f}'


def resolve_input(path):
    p = Path(path)
    if p.is_dir():
        for rel in ('imu0/data.csv', 'mav0/imu0/data.csv', 'data.csv'):
            if (p/rel).is_file(): return p/rel
        raise ValueError('Directory must contain imu0/data.csv or mav0/imu0/data.csv')
    return p


def rows(path, order, ts_unit, accel_unit, gyro_deg):
    """Yield timestamp text, seconds, acceleration, angular velocity; O(1) space."""
    scale = {'s':1.,'ms':1e-3,'us':1e-6,'ns':1e-9}[ts_unit]
    ac = G if accel_unit == 'g' else 1.
    gc = math.pi/180 if gyro_deg else 1.
    indices = None
    with open(path, encoding='utf-8-sig', newline='') as f:
        for lineno, r in enumerate(csv.reader(f),1):
            if not r or not any(x.strip() for x in r): continue
            try: float(r[0])
            except ValueError:
                if indices is not None: raise ValueError(f'Unexpected header at line {lineno}')
                if len(r) != 7:
                    if r[0].startswith('#'): continue
                    raise ValueError(f'Expected seven columns at line {lineno}')
                names = [x.strip().lower() for x in r]
                ai = [i for i,n in enumerate(names) if n.startswith(('a_rs_s_', 'accel_', 'acc_', 'ax','ay','az'))]
                gi = [i for i,n in enumerate(names) if n.startswith(('w_rs_s_', 'gyro_', 'gx','gy','gz'))]
                if len(ai)==3 and len(gi)==3: indices = (ai,gi)
                elif order == 'auto': raise ValueError('Unrecognized header: specify --column-order accel-gyro or gyro-accel')
                continue
            if len(r)!=7: raise ValueError(f'Expected seven columns at line {lineno}')
            if indices is None:
                if order == 'auto': raise ValueError('Headerless CSV requires --column-order')
                indices = ([1,2,3],[4,5,6]) if order=='accel-gyro' else ([4,5,6],[1,2,3])
            try:
                t = timestamp_seconds(r[0],scale)
                a = tuple(float(r[i])*ac for i in indices[0])
                w = tuple(float(r[i])*gc for i in indices[1])
            except ValueError as e: raise ValueError(f'Invalid number at line {lineno}') from e
            if not math.isfinite(t): raise ValueError(f'Invalid timestamp at line {lineno}')
            yield timestamp_text(r[0],scale),t,a,w


def usable(a,w):
    return all(math.isfinite(x) for x in a+w) and norm(a)>1e-6


def initialize(reader):
    samples=[]; start=None
    for _,t,a,w in reader():
        if not usable(a,w): continue
        if start is None: start=t
        if t-start>1.0 or len(samples)>=1000: break
        samples.append((a,w))
    if not samples: raise ValueError('Input contains no usable IMU measurements')
    a=tuple(statistics.median(s[0][k] for s in samples) for k in range(3))
    w=tuple(statistics.median(s[1][k] for s in samples) for k in range(3))
    spread=max(statistics.pstdev(s[0][k] for s in samples) for k in range(3))
    stationary = spread<0.12 and norm(w)<0.08 and abs(norm(a)-G)<0.35
    # Bias is calibrated only when the initial interval is plausibly at rest.
    bias=w if stationary else (0.,0.,0.)
    ab=tuple(x*(1-G/norm(a)) for x in a) if stationary else (0.,0.,0.)
    if stationary: return align_gravity(a),bias,ab,True
    # Offline calibration: a moving start must not be mistaken for gravity.
    # Find the first sustained quiet window using only the supplied IMU.
    window=deque(maxlen=100)
    for index,(_,t,aa,ww) in enumerate(reader()):
        if not usable(aa,ww): window.clear(); continue
        window.append((t,aa,ww))
        if len(window)<100 or t-window[0][0]<.4 or index%20: continue
        if norm(ww)>.2 or abs(norm(aa)-G)>.3: continue
        means=tuple(sum(s[1][k] for s in window)/len(window) for k in range(3))
        if max(sum((s[1][k]-means[k])**2 for s in window)/len(window) for k in range(3))>.012: continue
        wm=tuple(sum(s[2][k] for s in window)/len(window) for k in range(3))
        if max(sum((s[2][k]-wm[k])**2 for s in window)/len(window) for k in range(3))>.0001: continue
        anchor=t; bias=wm; ab=tuple(x*(1-G/norm(means)) for x in means)
        relative=(0.,0.,0.,1.); previous=None
        for _,tt,aaa,www in reader():
            if tt>anchor: break
            if previous is not None and usable(aaa,www):
                relative=advance(relative,tuple(www[k]-bias[k] for k in range(3)),max(0,tt-previous))
            previous=tt
        inverse=(-relative[0],-relative[1],-relative[2],relative[3])
        return unit(mul(align_gravity(means),inverse)),bias,ab,False
    return align_gravity(a),bias,ab,False


def smooth_file(raw, anchors, output):
    """Disk-backed velocity bridge. Anchors are inferred rests, never waypoints."""
    anchors.seek(0)
    def get_anchor():
        data=anchors.read(32)
        return struct.unpack('<4d',data) if data else None
    nxt=get_anchor(); previous=None; offset=[0.,0.,0.]; first=None
    with open(raw,encoding='ascii') as src, open(output,'w',encoding='ascii',newline='') as dst:
        for line in src:
            if line.startswith('#'): dst.write(line); continue
            fields=line.split(); t=float(fields[0]); p=list(map(float,fields[1:4]))
            if first is None: first=t
            while nxt and t>nxt[0]:
                duration=nxt[0]-(first if previous is None else previous)
                for k in range(3): offset[k]-=nxt[k+1]*duration*(1 if previous is None else .5)
                previous=nxt[0]; nxt=get_anchor()
            for k in range(3):
                correction=offset[k]
                if nxt:
                    start=first if previous is None else previous
                    duration=nxt[0]-start; elapsed=t-start
                    if duration>0:
                        correction-=nxt[k+1]*(elapsed if previous is None else .5*elapsed*elapsed/duration)
                p[k]+=correction
            dst.write(fields[0]+' '+' '.join(f'{x:.9f}' for x in p)+' '+' '.join(fields[4:])+'\n')


def regularize_velocity(raw, output, tau):
    """Offline Gaussian velocity prior, solved by disk-backed Thomas elimination.

    Minimize sum ||(v_i-v_{i-1})-(u_i-u_{i-1})||^2/dt
    + sum dt*||v_i||^2/tau^2, where u is the inertial velocity.
    Quiet intervals add a strong v=0 constraint. No truth or route constraints.
    """
    record=struct.Struct('<4d'); vector=struct.Struct('<3d')
    with tempfile.TemporaryFile() as forward, tempfile.TemporaryFile() as solved, open(raw) as src:
        def read():
            for line in src:
                if not line.startswith('#'):
                    f=line.split(); yield float(f[0]),tuple(map(float,f[8:11])),bool(int(f[11])),bool(int(f[12]))
        iterator=iter(read()); current=next(iterator,None)
        if current is None: raise ValueError('Empty raw trajectory')
        previous=None; cprev=0.; dprev=(0.,0.,0.); n=0
        while current:
            following=next(iterator,None)
            left=0. if previous is None else 1/max(current[0]-previous[0],1e-6)
            right=0. if following is None else 1/max(following[0]-current[0],1e-6)
            width=((0 if previous is None else current[0]-previous[0])+(0 if following is None else following[0]-current[0]))/2
            diagonal=left+right+max(width,1e-6)/(tau*tau)+(1e4 if current[2] else 0)
            diagonal=max(diagonal,1e-12)
            rhs=tuple((left*(current[1][k]-previous[1][k]) if previous else 0)-
                      (right*(following[1][k]-current[1][k]) if following else 0) for k in range(3))
            denom=diagonal+left*cprev
            cp=-right/denom
            dp=tuple((rhs[k]+left*dprev[k])/denom for k in range(3))
            forward.write(record.pack(cp,*dp)); cprev,dprev=cp,dp; n+=1
            previous,current=current,following
        result=(0.,0.,0.)
        for i in range(n-1,-1,-1):
            forward.seek(i*record.size); cp,*dp=record.unpack(forward.read(record.size))
            result=tuple(dp[k]-cp*result[k] for k in range(3))
            solved.seek(i*vector.size); solved.write(vector.pack(*result))
        solved.seek(0); p=[0.,0.,0.]; last_t=None; last_v=None
        src.seek(0)
        with open(output,'w',encoding='ascii',newline='') as dst:
            dst.write('# timestamp tx ty tz qx qy qz qw\n')
            for line in src:
                if line.startswith('#'): continue
                f=line.split(); t=float(f[0]); v=vector.unpack(solved.read(vector.size))
                if last_t is not None:
                    for k in range(3): p[k]+=.5*(last_v[k]+v[k])*(t-last_t)
                dst.write(f[0]+' '+' '.join(f'{x:.9f}' for x in p)+' '+' '.join(f[4:8])+'\n')
                last_t,last_v=t,v


def estimate(args):
    path=resolve_input(args.input)
    if path.resolve()==Path(args.output).resolve(): raise ValueError('Input and output paths must differ')
    reader=lambda: rows(path,args.column_order,args.ts_unit,args.accel_unit,args.gyro_deg)
    q,bias,ab,initial_rest=initialize(reader)
    if args.velocity_prior is None: args.velocity_prior=0. if initial_rest else 1.
    p=[0.,0.,0.]; v=[0.,0.,0.]; prev=None; last_a=(0.,0.,0.); last_w=(0.,0.,0.)
    history=deque(maxlen=9); rest=deque(maxlen=60)
    stats=dict(rows=0,invalid_rows=0,duplicate_rows=0,gaps=0,spikes=0,stationary_updates=0,
               initial_stationary=initial_rest,gyro_bias=list(bias),initial_accel_bias=list(ab))
    first=last=None; blind_age=0.; motion=(0.,0.,0.); filtered_w=(0.,0.,0.)
    Path(args.output).parent.mkdir(parents=True,exist_ok=True)
    temp=Path(str(args.output)+'.partial')
    anchor_file=tempfile.TemporaryFile()
    last_anchor=None
    try:
        with open(temp,'w',encoding='ascii',newline='') as out:
            out.write('# timestamp tx ty tz qx qy qz qw\n')
            for stamp,t,a,w in reader():
                is_rest=False
                if first is None: first=t
                dt=0. if prev is None else t-prev
                if dt<0: raise ValueError('Timestamps must be nondecreasing')
                valid=usable(a,w)
                if dt==0 and prev is not None: stats['duplicate_rows']+=1
                if dt>args.gap_threshold: stats['gaps']+=1
                if not valid:
                    stats['invalid_rows']+=1; blind_age+=dt
                    # Damped acceleration and angular-rate prediction, with exact integrals.
                    tau=args.prediction_tau
                    e0=math.exp(-(blind_age-dt)/tau); e1=math.exp(-blind_age/tau)
                    dv=tau*(e0-e1); dp=tau*e0*dt-tau*tau*(e0-e1)
                    for k in range(3): p[k]+=v[k]*dt+motion[k]*dp; v[k]+=motion[k]*dv
                    q=advance(q,filtered_w,tau*(e0-e1))
                    rest.clear()
                else:
                    blind_age=0.
                    if history and len(history)>=5:
                        ma=tuple(statistics.median(s[0][k] for s in history) for k in range(3))
                        mw=tuple(statistics.median(s[1][k] for s in history) for k in range(3))
                        if norm(tuple(a[k]-ma[k] for k in range(3)))>args.accel_spike or norm(tuple(w[k]-mw[k] for k in range(3)))>args.gyro_spike:
                            a,w=ma,mw; stats['spikes']+=1
                    history.append((a,w))
                    ac=tuple(a[k]-ab[k] for k in range(3)); wc=tuple(w[k]-bias[k] for k in range(3))
                    # Missing rows: integrate held last force/rate over gap, current over last nominal interval.
                    n=max(1,math.ceil(dt/0.02)); h=dt/n
                    for j in range(n):
                        gap=dt>args.gap_threshold and j<n-1
                        aa=last_a if gap else ac; ww=last_w if gap else wc
                        mid=advance(q,ww,h/2); world=rotate(mid,aa)
                        lin=(world[0],world[1],world[2]-G)
                        for k in range(3): p[k]+=v[k]*h+0.5*lin[k]*h*h; v[k]+=lin[k]*h
                        q=advance(q,ww,h)
                    last_a,last_w=ac,wc
                    alpha=1-math.exp(-dt/0.3)
                    motion=tuple(motion[k]+alpha*(lin[k]-motion[k]) for k in range(3))
                    filtered_w=tuple(filtered_w[k]+alpha*(wc[k]-filtered_w[k]) for k in range(3))
                    rest.append((t,a,w))
                    is_rest=False
                    if len(rest)>=20 and rest[-1][0]-rest[0][0]>=0.20:
                        # A sustained low-variance, low-rate interval is a motion prior, not an observation of speed.
                        if norm(wc)<0.01 and abs(norm(ac)-G)<0.08:
                            means=[sum(s[1][k] for s in rest)/len(rest) for k in range(3)]
                            variance=sum(sum((s[1][k]-means[k])**2 for s in rest)/len(rest) for k in range(3))
                            is_rest=variance<0.012 and all(norm(tuple(s[2][k]-bias[k] for k in range(3)))<0.025 for s in rest)
                    if is_rest and not args.no_zupt:
                        if last_anchor is None or t-last_anchor>=.5:
                            anchor_file.write(struct.pack('<4d',t,*v)); last_anchor=t
                        v[:]=[0.,0.,0.]; motion=(0.,0.,0.)
                        rate=min(1.,dt/2)
                        bias=tuple(bias[k]+rate*(w[k]-bias[k]) for k in range(3))
                        # Gravity correction only within a confirmed quiet interval.
                        predicted=rotate(q,ac)
                        correction=align_gravity(predicted)
                        blend=unit(tuple(rate*x for x in correction[:3])+(1+rate*(correction[3]-1),))
                        q=unit(mul(blend,q)); stats['stationary_updates']+=1
                if args.no_zupt: is_rest=False
                if args.velocity_prior>0:
                    out.write(stamp+' '+' '.join(f'{x:.9f}' for x in (*p,*q,*v))+f' {int(is_rest)} {int(valid)}\n')
                else:
                    out.write(stamp+' '+' '.join(f'{x:.9f}' for x in (*p,*q))+'\n')
                stats['rows']+=1; prev=t; last=t
        if not stats['rows']: raise ValueError('Empty input')
        if args.velocity_prior>0:
            regularize_velocity(temp,args.output,args.velocity_prior); temp.unlink()
        elif args.no_zupt: os.replace(temp,args.output)
        else:
            smooth_file(temp,anchor_file,args.output); temp.unlink()
    except BaseException:
        temp.unlink(missing_ok=True)
        raise
    finally: anchor_file.close()
    stats['duration_s']=last-first
    stats['velocity_prior_tau_s']=args.velocity_prior
    stats['assumptions']=['quiet windows imply zero velocity','initial velocity unobservable without a rest anchor','zero-mean velocity prior suppresses sustained motion']
    return stats


def parser():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('input',help='IMU CSV or EuRoC sequence directory')
    p.add_argument('output',nargs='?',default='estimated_trajectory.txt')
    p.add_argument('--column-order',choices=['auto','accel-gyro','gyro-accel'],default='auto')
    p.add_argument('--ts-unit',choices=['ns','us','ms','s'],default='ns')
    p.add_argument('--accel-unit',choices=['ms2','g'],default='ms2')
    p.add_argument('--gyro-deg',action='store_true')
    p.add_argument('--metadata',default=None)
    p.add_argument('--profile-memory',action='store_true',help='Separate diagnostic run: tracemalloc slows execution')
    p.add_argument('--no-zupt',action='store_true',help='Disable the stationary zero-velocity prior')
    p.add_argument('--velocity-prior',type=float,default=None,metavar='SECONDS',help='Offline velocity prior: auto disabled for a quiet start, 1s otherwise; 0 disables')
    p.add_argument('--gap-threshold',type=float,default=0.05)
    p.add_argument('--prediction-tau',type=float,default=0.7)
    p.add_argument('--accel-spike',type=float,default=4.0)
    p.add_argument('--gyro-spike',type=float,default=0.8)
    return p


def main():
    args=parser().parse_args()
    positive=[args.prediction_tau,args.gap_threshold,args.accel_spike,args.gyro_spike]
    if any(not math.isfinite(x) or x<=0 for x in positive) or (args.velocity_prior is not None and (not math.isfinite(args.velocity_prior) or args.velocity_prior<0)):
        parser().error('Time constants and spike thresholds must be finite and positive; velocity prior may be zero')
    if args.profile_memory:
        import tracemalloc
        tracemalloc.start()
    start=time.perf_counter()
    try: stats=estimate(args)
    except (ValueError,OSError,csv.Error) as e:
        print(f'error: {e}',file=sys.stderr); return 2
    stats.update(wall_time_s=time.perf_counter()-start,python=sys.version.split()[0],
                 memory_profiled=args.profile_memory,estimator='jarvis-inertial-v1')
    stats['ms_per_sensor_second']=stats['wall_time_s']*1000/max(stats['duration_s'],1e-9)
    if args.profile_memory:
        stats['peak_python_heap_bytes']=tracemalloc.get_traced_memory()[1]; tracemalloc.stop()
    if args.metadata:
        Path(args.metadata).parent.mkdir(parents=True,exist_ok=True)
        Path(args.metadata).write_text(json.dumps(stats,indent=2)+'\n')
    print(json.dumps(stats,indent=2)); return 0


if __name__=='__main__': sys.exit(main())
