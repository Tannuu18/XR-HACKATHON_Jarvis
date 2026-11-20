"""Evaluation only. SE(3) ATE and time-based RPE; never imported by main.py."""
import argparse
import json
from pathlib import Path
import numpy as np


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
    p=argparse.ArgumentParser(); p.add_argument('truth'); p.add_argument('estimate'); p.add_argument('--output')
    a=p.parse_args(); result=evaluate(a.truth,a.estimate)
    text=json.dumps(result,indent=2); print(text)
    if a.output: Path(a.output).write_text(text+'\n')
