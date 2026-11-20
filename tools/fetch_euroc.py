"""Download only IMU and truth entries from an official EuRoC ZIP using ranges.

Never downloads images. Ground truth is reserved for tools/evaluate.py.
"""
import argparse
import json
from pathlib import Path
import struct
import urllib.request
import zlib

BASE='https://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/'


def fetch(sequence, destination):
    group='machine_hall' if sequence.startswith('MH') else 'vicon_room'+sequence[1]
    url=f'{BASE}{group}/{sequence}/{sequence}.zip'
    def get(start,end):
        req=urllib.request.Request(url,headers={'Range':f'bytes={start}-{end}'})
        with urllib.request.urlopen(req,timeout=60) as response:
            if response.status!=206: raise RuntimeError('Server must support HTTP Range; refusing full dataset download')
            return response.read()
    with urllib.request.urlopen(urllib.request.Request(url,method='HEAD'),timeout=60) as r:
        size=int(r.headers['Content-Length'])
    tail=get(max(0,size-65557),size-1)
    e=tail.rfind(b'PK\x05\x06')
    if e<0: raise RuntimeError('Missing ZIP directory')
    _,_,_,_,entries,cd_size,cd_offset,_=struct.unpack_from('<4s4H2IH',tail,e)
    directory=get(cd_offset,cd_offset+cd_size-1); pos=0; found=[]
    for _ in range(entries):
        fields=struct.unpack_from('<4s6H3I5H2I',directory,pos)
        method=fields[4]; crc,compressed=fields[7:9]; nl,el,cl=fields[10:13]; offset=fields[-1]
        name=directory[pos+46:pos+46+nl].decode(); pos+=46+nl+el+cl
        if not name.endswith(('imu0/data.csv','state_groundtruth_estimate0/data.csv')): continue
        header=get(offset,offset+29); local=struct.unpack('<4s5H3I2H',header)
        start=offset+30+local[-2]+local[-1]
        data=get(start,start+compressed-1)
        data=zlib.decompress(data,-15) if method==8 else data
        if zlib.crc32(data)&0xffffffff!=crc: raise RuntimeError('ZIP CRC mismatch')
        target=Path(destination)/sequence/name
        target.parent.mkdir(parents=True,exist_ok=True); target.write_bytes(data); found.append(str(target))
    if len(found)!=2: raise RuntimeError(f'Expected two files, found {found}')
    manifest={'source':url,'sequence':sequence,'files':found,'notes':'IMU is estimator input; truth is evaluation only. Original dataset terms apply.'}
    (Path(destination)/sequence/'provenance.json').write_text(json.dumps(manifest,indent=2))
    print(json.dumps(manifest,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('sequence',nargs='?',default='MH_01_easy'); p.add_argument('--destination',default='data/euroc')
    a=p.parse_args(); fetch(a.sequence,a.destination)
