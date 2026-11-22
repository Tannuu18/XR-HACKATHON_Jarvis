"""Local ZIP upload/run demo. The standalone estimator remains src/main.py.

Start: python serve_demo.py   then open http://127.0.0.1:8766/
Only the packaged estimator executes. Uploaded programs are never executed.
"""
import argparse
import csv
import hashlib
from http.server import SimpleHTTPRequestHandler,ThreadingHTTPServer
import io
import json
from pathlib import Path,PurePosixPath
import secrets
import shutil
import subprocess
import sys
import tempfile
import threading
from urllib.parse import urlsplit,parse_qs
import zipfile

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'src'))
import main as estimator
from tools.validate_output import validate

MAX_UPLOAD=64*1024*1024
MAX_ENTRY=64*1024*1024
TOKEN=secrets.token_hex(24)
LOCK=threading.Lock()
STORE=tempfile.TemporaryDirectory(prefix='jarvis-zip-demo-')
DOWNLOADS={}


def inventory(z):
    infos=z.infolist()
    if len(infos)>4000 or sum(i.file_size for i in infos)>256*1024*1024:
        raise ValueError('ZIP is too large after decompression. Include only the IMU and optional reference files.')
    seen=set();imus=[];refs=[]
    for info in infos:
        name=info.filename;path=PurePosixPath(name.replace('\\','/'))
        if path.is_absolute() or '..' in path.parts or (path.parts and ':' in path.parts[0]):raise ValueError('ZIP contains an unsafe path.')
        if name in seen:raise ValueError('ZIP contains duplicate file names.')
        seen.add(name)
        if info.flag_bits&1:raise ValueError('Encrypted ZIP files are not supported.')
        if info.is_dir() or path.suffix.lower() not in ('.csv','.txt'):continue
        if info.file_size>MAX_ENTRY:raise ValueError('A data file exceeds the 64 MB limit.')
        if info.compress_type not in (zipfile.ZIP_STORED,zipfile.ZIP_DEFLATED):raise ValueError('Use a normal stored/deflated ZIP.')
        with z.open(info) as src:sample=src.read(8192).decode('utf-8-sig',errors='replace')
        for line in sample.splitlines()[:20]:
            if not line.strip():continue
            fields=next(csv.reader([line])) if ',' in line else line.split()
            if line.lstrip().startswith('#') and len(fields) not in (7,17):continue
            if len(fields)==7 and path.suffix.lower()=='.csv':
                try:float(fields[0]);headerless=True
                except ValueError:headerless=False
                imus.append({'name':name,'headerless':headerless});break
            if len(fields)==17 and ',' in line:
                refs.append({'name':name,'format':'euroc17'});break
            if len(fields)==8 and ',' not in line:
                try:[float(v) for v in fields]
                except ValueError:continue
                refs.append({'name':name,'format':'tum8'});break
    return {'imu_files':imus,'reference_files':refs}


def copy_member(z,name,target):
    total=0
    with z.open(name) as src,target.open('wb') as dst:
        while True:
            block=src.read(1024*1024)
            if not block:break
            total+=len(block)
            if total>MAX_ENTRY:raise ValueError('Data file exceeds the decompression limit.')
            dst.write(block)


def build_plot(output,imu,reference,options,runtime):
    poses=[]
    for line in output.read_text(encoding='ascii').splitlines():
        if not line.startswith('#') and line.strip():poses.append([float(x) for x in line.split()])
    if len(poses)>300000:raise ValueError('Web preview supports up to 300,000 input frames.')
    start=poses[0][0];times=[p[0]-start for p in poses];points=[p[1:4] for p in poses]
    score={'ate_rmse_m':None,'rpe_1s_translation_rmse_m':None,'matched_rows':0,'regimes':{'final_invalid_sensor_window':{'rmse_m':None}}}
    ref_points=None;ref_path=[];errors=None;reference_error=None;failure_start=None
    for _,t,a,w in estimator.rows(imu,options['order'],options['unit'],options['accel'],options['gyro_deg']):
        if estimator.usable(a,w):failure_start=None
        elif failure_start is None:failure_start=t-start
    if reference:
        try:
            import numpy as np
            from tools.evaluate import evaluate,load_truth
            tt,tp,tq=load_truth(reference)
            if len(tt)<3 or not np.isfinite(tt).all() or not np.isfinite(tp).all() or not np.isfinite(tq).all() or (np.diff(tt)<0).any() or (np.linalg.norm(tq,axis=1)<1e-8).any():
                raise ValueError('Reference needs sorted finite timestamps, positions and nonzero quaternions.')
            est=np.asarray(poses);et=est[:,0];ix=np.clip(np.searchsorted(tt,et),1,len(tt)-1)
            ix=np.where(abs(tt[ix]-et)<abs(tt[ix-1]-et),ix,ix-1)
            keep=(abs(tt[ix]-et)<.02)&(et>=tt[0])&(et<=tt[-1])
            if keep.sum()<3:raise ValueError('Fewer than three timestamps match the reference within 20 ms.')
            gt=tp[ix[keep]];pred=est[keep,1:4];x=pred-pred.mean(0);y=gt-gt.mean(0)
            u,_,vt=np.linalg.svd(x.T@y);d=np.eye(3);d[-1,-1]=np.linalg.det(vt.T@u.T)
            aligned=x@(vt.T@d@u.T).T+gt.mean(0)
            err=np.linalg.norm(aligned-gt,axis=1);matched_times=et[keep]-start
            score=evaluate(reference,output)
            mask=matched_times>=failure_start if failure_start is not None else np.zeros(len(err),dtype=bool)
            score['regimes']={'final_invalid_sensor_window':{'matched_rows':int(mask.sum()),'rmse_m':float(np.sqrt(np.mean(err[mask]**2))) if mask.any() else None}}
            points=aligned.tolist();times=matched_times.tolist();ref_points=gt.tolist();errors=err.tolist()
            within=(tt>=est[0,0])&(tt<=est[-1,0]);full=tp[within];ref_path=full[::max(1,len(full)//4000)].tolist()
        except ImportError:
            reference_error='The estimate ran successfully. Install NumPy for optional reference scoring: python -m pip install numpy'
        except (ValueError,OSError) as exc:reference_error='The estimate ran successfully, but the reference could not be scored: '+str(exc)
    indices=list(range(0,len(points),max(1,len(points)//2000)))
    if indices[-1]!=len(points)-1:indices.append(len(points)-1)
    return {'label':'Uploaded run','custom':True,'real':True,'rows':len(poses),'duration':poses[-1][0]-start,'score':score,
            'time':[times[i] for i in indices],'estimate':[points[i] for i in indices],
            'reference':[ref_points[i] for i in indices] if ref_points is not None else None,'referencePath':ref_path,
            'errors':[errors[i] for i in indices] if errors is not None else None,'failureStart':failure_start},reference_error


def run_archive(blob,params):
    options={'order':params.get('order','auto'),'unit':params.get('unit','ns'),'accel':params.get('accel','ms2'),'gyro_deg':params.get('gyro_deg')=='1'}
    if options['order'] not in ('auto','accel-gyro','gyro-accel') or options['unit'] not in ('ns','us','ms','s') or options['accel'] not in ('ms2','g'):raise ValueError('Invalid CSV format settings.')
    with zipfile.ZipFile(io.BytesIO(blob)) as z,tempfile.TemporaryDirectory(prefix='jarvis-input-') as folder:
        contents=inventory(z);names={f['name'] for f in contents['imu_files']};imu_name=params.get('imu','')
        if imu_name not in names:raise ValueError('Choose one recognized seven-column IMU CSV from the ZIP.')
        reference_name=params.get('reference','');ref_info=next((f for f in contents['reference_files'] if f['name']==reference_name),None)
        if reference_name and ref_info is None:raise ValueError('Choose a recognized reference file or leave the reference empty.')
        folder=Path(folder);imu=folder/'imu.csv';output=folder/'estimated_trajectory.txt';meta=folder/'metadata.json'
        copy_member(z,imu_name,imu)
        reference=None
        if ref_info:
            reference=folder/('reference.csv' if ref_info['format']=='euroc17' else 'reference.txt');copy_member(z,reference_name,reference)
        command=[sys.executable,str(ROOT/'src/main.py'),str(imu),str(output),'--metadata',str(meta),'--column-order',options['order'],'--ts-unit',options['unit'],'--accel-unit',options['accel']]
        if options['gyro_deg']:command.append('--gyro-deg')
        run=subprocess.run(command,cwd=folder,capture_output=True,text=True,timeout=120)
        if run.returncode:raise ValueError((run.stderr or 'Estimator could not process the IMU.')[-1200:])
        validation=validate(imu,output,options['order'],options['unit']);runtime=json.loads(meta.read_text())
        if validation['rows']>300000:raise ValueError('Web demo supports up to 300,000 input frames.')
        heap=None
        if params.get('profile')=='1':
            profile_meta=folder/'memory.json';profile=command[:];profile[profile.index(str(meta))]=str(profile_meta)
            result=subprocess.run(profile+['--profile-memory'],cwd=folder,capture_output=True,text=True,timeout=180)
            if result.returncode:raise ValueError('The separate memory profile could not finish.')
            heap=json.loads(profile_meta.read_text())['peak_python_heap_bytes']
        plot,reference_error=build_plot(output,imu,reference,options,runtime)
        resource={'runtime':runtime['ms_per_sensor_second'],'heap':heap,'note':'One measured run on this uploaded IMU. Upload, plotting and optional reference scoring are excluded. Heap is measured only when the separate profile is selected.'}
        plot['resource']=resource
        runtime.update(input_sha256=hashlib.sha256(imu.read_bytes()).hexdigest(),estimator_sha256=hashlib.sha256((ROOT/'src/main.py').read_bytes()).hexdigest())
        run_id=secrets.token_hex(16);archive=Path(STORE.name)/(run_id+'.zip')
        with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as result:
            result.write(output,'estimated_trajectory.txt');result.writestr('metadata.json',json.dumps(runtime,indent=2));result.writestr('validation.json',json.dumps(validation,indent=2));result.writestr('evaluation.json',json.dumps({'reference_provided':bool(reference),'reference_error':reference_error,'score':plot['score']},indent=2))
        DOWNLOADS[run_id]=archive
        while len(DOWNLOADS)>8:
            name=next(iter(DOWNLOADS));old=DOWNLOADS.pop(name)
            try:old.unlink()
            except OSError:pass
        return {'plot':plot,'validation':validation,'resource':resource,'reference_error':reference_error,'download_url':'/api/download/'+run_id,'run_id':run_id}


class Handler(SimpleHTTPRequestHandler):
    def __init__(self,*args,**kwargs):super().__init__(*args,directory=str(ROOT/'demo'),**kwargs)

    def json(self,value,status=200):
        body=json.dumps(value,allow_nan=False).encode('utf-8');self.send_response(status);self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(body)));self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(body)

    def do_GET(self):
        path=urlsplit(self.path).path
        if path=='/api/health':return self.json({'ready':True,'token':TOKEN,'max_upload_bytes':MAX_UPLOAD,'estimator':'jarvis-final-v3'})
        if path.startswith('/api/download/'):
            name=path.rsplit('/',1)[-1];file=DOWNLOADS.get(name)
            if file is None:return self.json({'error':'Result expired; rerun the ZIP.'},404)
            try:
                with file.open('rb') as src:
                    self.send_response(200);self.send_header('Content-Type','application/zip');self.send_header('Content-Disposition','attachment; filename="Jarvis_Results.zip"');self.send_header('Content-Length',str(file.stat().st_size));self.end_headers();shutil.copyfileobj(src,self.wfile)
            except OSError:return
            return
        return super().do_GET()

    def do_POST(self):
        path=urlsplit(self.path).path
        if path not in ('/api/inspect','/api/run'):return self.json({'error':'Not found'},404)
        if self.headers.get('X-Jarvis-Token')!=TOKEN:return self.json({'error':'Reload the page before uploading.'},403)
        origin=self.headers.get('Origin')
        if origin:
            parsed=urlsplit(origin)
            if parsed.hostname not in ('localhost','127.0.0.1') or parsed.port!=self.server.server_port:return self.json({'error':'Origin is not allowed.'},403)
        if not LOCK.acquire(blocking=False):return self.json({'error':'Another run is in progress; try again when it finishes.'},409)
        try:
            size=int(self.headers.get('Content-Length','0'))
            if not 0<size<=MAX_UPLOAD:raise ValueError('Choose a ZIP no larger than 64 MB.')
            blob=self.rfile.read(size)
            if len(blob)!=size:raise ValueError('Upload was incomplete.')
            if path=='/api/inspect':
                with zipfile.ZipFile(io.BytesIO(blob)) as z:result=inventory(z)
                if not result['imu_files']:raise ValueError('No seven-column IMU CSV found. Include IMU data, not just code or a trajectory.')
            else:result=run_archive(blob,{k:v[-1] for k,v in parse_qs(urlsplit(self.path).query).items()})
            return self.json(result)
        except (ValueError,zipfile.BadZipFile,RuntimeError,subprocess.TimeoutExpired,OSError) as exc:return self.json({'error':str(exc)[:1200]},400)
        finally:LOCK.release()


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--port',type=int,default=8766);a=p.parse_args()
    server=ThreadingHTTPServer(('127.0.0.1',a.port),Handler)
    print(f'Jarvis ZIP runner: http://127.0.0.1:{a.port}/',flush=True)
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:server.server_close();STORE.cleanup()
