"""Evaluation only. SE(3) ATE and time-based RPE; never imported by main.py."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))


def load_truth(path):
    if str(path).endswith('.csv'):
        a=np.loadtxt(path,delimiter=',',comments='#')
        return a[:,0]*1e-9,a[:,1:4],a[:,[5,6,7,4]]
    a=np.loadtxt(path,comments='#'); return a[:,0],a[:,1:4],a[:,4:8]


def matrix(q):
    x,y,z,w=q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def evaluate(truth,estimate,imu=None):
    tt,tp,tq=load_truth(truth); est=np.loadtxt(estimate,comments='#'); et,ep,eq=est[:,0],est[:,1:4],est[:,4:8]
    right=np.searchsorted(tt,et); right=np.clip(right,1,len(tt)-1)
    ix=np.where(abs(tt[right]-et)<abs(tt[right-1]-et),right,right-1)
    valid=(abs(tt[ix]-et)<.02)&(et>=tt[0])&(et<=tt[-1])
    gt=tp[ix[valid]]; pred=ep[valid]; times=et[valid]
    if len(gt)<3: raise ValueError('Fewer than three timestamp associations')
    x=pred-pred.mean(0); y=gt-gt.mean(0)
    u,_,vt=np.linalg.svd(x.T@y); d=np.eye(3); d[-1,-1]=np.linalg.det(vt.T@u.T)
    r=vt.T@d@u.T
    aligned=x@r.T+gt.mean(0); errors=np.linalg.norm(aligned-gt,axis=1)
    out={'alignment':'SE(3), no scale correction','association_tolerance_s':.02,'matched_rows':len(gt),
         'ate_rmse_m':float(np.sqrt(np.mean(errors**2))),'ate_median_m':float(np.median(errors)),
         'ate_max_m':float(errors.max()),'tail_5s_rmse_m':float(np.sqrt(np.mean(errors[times>=times[-1]-5]**2)))}
    if imu:
        from main import rows,usable
        failure_start=None; first_time=None; previous_time=None; gap_end=None
        observed_times=[]; gap_adjacent=[]; usable_frames=[]
        for _,t,a,w in rows(imu,'auto','ns','ms2',False):
            if first_time is None: first_time=t
            ok=usable(a,w)
            if ok: failure_start=None
            elif failure_start is None: failure_start=t
            if previous_time is not None and t-previous_time>.05: gap_end=t+.5
            gap_adjacent.append(gap_end is not None and t<=gap_end)
            usable_frames.append(ok); observed_times.append(t); previous_time=t
        if len(observed_times)!=len(et) or np.max(np.abs(np.asarray(observed_times)-et))>1e-6:
            raise ValueError('IMU timestamps do not match the estimated trajectory')
        def regime(mask):
            associated=np.asarray(mask)[valid]
            return {'matched_rows':int(associated.sum()),'rmse_m':float(np.sqrt(np.mean(errors[associated]**2))) if associated.any() else None}
        out['regimes']={'valid_sensor':regime(usable_frames),'within_half_second_after_gap':regime(gap_adjacent)}
        if failure_start is not None:
            out['regimes']['final_invalid_sensor_window']=regime(np.asarray(observed_times)>=failure_start)
            out['regimes']['final_invalid_sensor_start_relative_s']=failure_start-first_time
        out['regime_note']='Same global SE(3) alignment; failure masks come from input validity. Gap metric covers supplied poses within 0.5 s after gaps over 0.05 s, not hidden dropout/checkpoint scores.'
    j=np.searchsorted(times,times+1.); keep=j<len(times); i=np.flatnonzero(keep); j=j[keep]
    good=abs(times[j]-times[i]-1)<.02; i,j=i[good],j[good]
    if len(i):
        # Relative transforms E = inv(Tgt_i^-1 Tgt_j) (Test_i^-1 Test_j).
        translations=[]; angles=[]; gq=tq[ix[valid]]; pq=eq[valid]
        for a,b in zip(i,j):
            ga,gb,pa,pb=matrix(gq[a]),matrix(gq[b]),matrix(pq[a]),matrix(pq[b])
            rg=ga.T@gb; re=pa.T@pb
            translations.append(np.linalg.norm(pa.T@(pred[b]-pred[a])-ga.T@(gt[b]-gt[a])))
            angles.append(np.arccos(np.clip((np.trace(rg.T@re)-1)/2,-1,1)))
        out['rpe_1s_translation_rmse_m']=float(np.sqrt(np.mean(np.square(translations))))
        out['rpe_1s_rotation_rmse_deg']=float(np.degrees(np.sqrt(np.mean(np.square(angles)))))
    return out


if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('truth'); p.add_argument('estimate'); p.add_argument('--output'); p.add_argument('--imu',help='Optional IMU for actual failure/gap regime metrics')
    a=p.parse_args(); result=evaluate(a.truth,a.estimate,a.imu)
    text=json.dumps(result,indent=2); print(text)
    if a.output: Path(a.output).write_text(text+'\n')
