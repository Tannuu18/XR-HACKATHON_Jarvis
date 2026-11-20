import csv
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from main import align_gravity,rotate,advance


class EstimatorTests(unittest.TestCase):
    def run_case(self, samples, header=None, flags=()):
        with tempfile.TemporaryDirectory() as td:
            src=Path(td)/'imu.csv'; out=Path(td)/'trajectory.txt'
            with src.open('w',newline='') as f:
                w=csv.writer(f)
                if header: w.writerow(header)
                w.writerows(samples)
            proc=subprocess.run([sys.executable,str(ROOT/'main.py'),str(src),str(out),*flags],capture_output=True,text=True)
            data=[list(map(float,l.split())) for l in out.read_text().splitlines() if not l.startswith('#')] if out.exists() else []
            return proc,data

    def test_stationary_duplicate_gap_and_blind(self):
        samples=[]
        for i in range(2000):
            if 600<i<1000: continue
            a=(0,0,9.80665) if i<1500 else (0,0,0)
            row=[1403636579000000000+i*5000000,*a,0,0,0]
            samples.append(row)
            if i==500: samples.append(row)
        proc,data=self.run_case(samples,['timestamp','ax','ay','az','gx','gy','gz'])
        self.assertEqual(proc.returncode,0,proc.stderr); self.assertEqual(len(data),len(samples))
        self.assertLess(max(abs(x) for r in data for x in r[1:4]),1e-7)
        for r in data: self.assertAlmostEqual(sum(x*x for x in r[4:]),1.,places=7)

    def test_gyro_first_and_constant_acceleration(self):
        samples=[[i*5000000,0,0,0,0.5 if i>=400 else 0,0,9.80665] for i in range(1001)]
        proc,data=self.run_case(samples,['#timestamp [ns]','w_RS_S_x','w_RS_S_y','w_RS_S_z','a_RS_S_x','a_RS_S_y','a_RS_S_z'],['--no-zupt','--velocity-prior','0'])
        self.assertEqual(proc.returncode,0,proc.stderr)
        self.assertAlmostEqual(data[-1][1],2.2575,delta=.02)

    def test_headerless_requires_order(self):
        proc,_=self.run_case([[0,0,0,9.80665,0,0,0]])
        self.assertEqual(proc.returncode,2); self.assertIn('requires --column-order',proc.stderr)

    def test_unsorted_rejected(self):
        proc,_=self.run_case([[10,0,0,9.80665,0,0,0],[0,0,0,9.80665,0,0,0]],flags=['--column-order','accel-gyro'])
        self.assertEqual(proc.returncode,2)

    def test_nan_failure_and_units(self):
        samples=[[i*.01,0,0,1 if i<200 else float('nan'),0,0,0] for i in range(400)]
        proc,data=self.run_case(samples,flags=['--column-order','accel-gyro','--ts-unit','s','--accel-unit','g'])
        self.assertEqual(proc.returncode,0,proc.stderr)
        self.assertTrue(all(math.isfinite(x) for r in data for x in r))

    def test_arbitrary_gravity_and_rotation(self):
        for a in [(9.80665,0,0),(0,0,-9.80665),(2,3,9)]:
            v=rotate(align_gravity(a),a)
            self.assertAlmostEqual(v[0],0,places=6); self.assertAlmostEqual(v[1],0,places=6)
            self.assertGreater(v[2],0)
        q=advance((0,0,0,1),(0,0,math.pi/2),1)
        v=rotate(q,(1,0,0)); self.assertAlmostEqual(v[1],1,places=6)

    def test_exact_epoch_nanoseconds(self):
        from main import timestamp_text
        self.assertEqual(timestamp_text('1403636579758555392',1e-9),'1403636579.758555392')

    def test_all_invalid_rejected(self):
        proc,data=self.run_case([[i,0,0,0,0,0,0] for i in range(10)],flags=['--column-order','accel-gyro'])
        self.assertEqual(proc.returncode,2); self.assertFalse(data)

    def test_regularizer_constant_velocity_has_zero_mean_prior(self):
        from main import regularize_velocity
        with tempfile.TemporaryDirectory() as td:
            raw=Path(td)/'raw'; out=Path(td)/'out'
            # Constant velocity has no IMU-observable acceleration. Under the
            # explicit zero-mean prior its best estimate is zero, not the truth.
            raw.write_text('\n'.join(f'{i*.01:.9f} 0 0 0 0 0 0 1 3 0 0 0 1' for i in range(100)))
            regularize_velocity(raw,out,1)
            values=[list(map(float,l.split())) for l in out.read_text().splitlines() if not l.startswith('#')]
            self.assertLess(max(abs(r[1]) for r in values),1e-8)


if __name__=='__main__': unittest.main()
