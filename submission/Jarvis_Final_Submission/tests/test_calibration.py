"""Known-signal checks for fallback gyro calibration and its rejection gate."""
import math
import csv
from pathlib import Path
import sys
import unittest
import tempfile
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
import main as m


class CalibrationTests(unittest.TestCase):
    def noisy_rest(self):
        bias=(.01,-.02,.09)
        for i in range(801):
            sign=1. if i%2 else -1.
            yield str(i*.005),i*.005,(.3*sign,.2*sign,m.G),tuple(v+.035*sign for v in bias)

    def test_known_offset_under_high_frequency_zero_mean_vibration(self):
        old={};new={}
        legacy=m.initialize(self.noisy_rest,old,False)
        q,bg,ba,initial_rest=m.initialize(self.noisy_rest,new)
        self.assertEqual(old['mode'],'uncalibrated')
        self.assertEqual(new['mode'],'robust_quiet_fallback')
        for actual,expected in zip(bg,(.01,-.02,.09)):self.assertAlmostEqual(actual,expected,places=9)
        self.assertAlmostEqual(m.norm(q),1.,places=12)
        self.assertFalse(initial_rest,'A later calibration must not invent an initial velocity anchor')
        self.assertEqual(legacy[1],(0.,0.,0.))

    def test_large_sustained_angular_rate_is_not_a_gyro_offset(self):
        def motion():
            for i in range(501):yield str(i*.005),i*.005,(0.,0.,m.G),(0.,0.,1.)
        diag={};result=m.initialize(motion,diag)
        self.assertEqual(diag['mode'],'uncalibrated')
        self.assertEqual(result[1],(0.,0.,0.))

    def test_existing_quiet_initialization_is_preserved(self):
        def rest():
            for i in range(501):yield str(i*.005),i*.005,(0.,0.,m.G),(0.,0.,0.)
        diag={};new=m.initialize(rest,diag);old=m.initialize(rest,robust_fallback=False)
        self.assertEqual(new,old)
        self.assertEqual(diag['mode'],'initial_quiet')

    def test_invalid_samples_do_not_poison_calibration(self):
        def interrupted():
            for i,row in enumerate(self.noisy_rest()):
                if i==220:yield row[0],row[1],(float('nan'),0.,0.),row[3]
                else:yield row
        diag={};q,bg,ba,_=m.initialize(interrupted,diag)
        self.assertEqual(diag['mode'],'robust_quiet_fallback')
        self.assertTrue(all(math.isfinite(v) for v in (*q,*bg,*ba)))

    def run_anchor_case(self,tail_rest=False,explicit=False):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);imu=root/'imu.csv';output=root/'out.txt'
            with imu.open('w',newline='') as f:
                w=csv.writer(f);w.writerow(['timestamp','ax','ay','az','gx','gy','gz'])
                for i in range(2401):
                    moving=i>=400 and not (tail_rest and i>1800)
                    w.writerow([i*5000000,.4 if moving else 0,0,m.G,0,0,.3 if moving else 0])
            flags=['--velocity-prior','0'] if explicit else []
            args=m.parser().parse_args([str(imu),str(output),*flags])
            stats=m.estimate(args)
            values=[line.split() for line in output.read_text().splitlines() if not line.startswith('#')]
            self.assertEqual(len(values),2401)
            self.assertTrue(all(len(row)==8 for row in values))
            return stats

    def test_initial_rest_does_not_leave_later_motion_unbounded(self):
        stats=self.run_anchor_case()
        self.assertTrue(stats['initial_stationary'])
        self.assertEqual(stats['velocity_prior_reason'],'unanchored_tail')
        self.assertGreater(stats['velocity_prior_tau_s'],0)

    def test_recent_rest_preserves_original_bridge(self):
        stats=self.run_anchor_case(tail_rest=True)
        self.assertEqual(stats['velocity_prior_reason'],'rest_anchored')
        self.assertEqual(stats['velocity_prior_tau_s'],0)

    def test_explicit_disabled_prior_is_respected(self):
        stats=self.run_anchor_case(explicit=True)
        self.assertEqual(stats['velocity_prior_reason'],'explicit')
        self.assertEqual(stats['velocity_prior_tau_s'],0)


if __name__=='__main__':unittest.main()
