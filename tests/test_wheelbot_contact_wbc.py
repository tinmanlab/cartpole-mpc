"""Native contract tests; no browser, server, or extra dependencies."""
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import wheelbot_contact_wbc as w
import numpy as np
import mujoco
from scipy.optimize import linprog

class ContactWBC(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.m = w.model()
        cls.x = w.settle(cls.m, 1)

    def test_dynamics_virtual_work(self):
        d, M, B, contacts = w.inspect(self.m, self.x)
        np.testing.assert_allclose(M, M.T, atol=1e-12)
        self.assertGreater(np.linalg.eigvalsh(M).min(), 0)
        self.assertFalse(B[:3].any())
        for c in contacts:
            f = np.array([2., .2, -.3]); generalized = np.zeros(6)
            for body, sign in [(c['b2'], 1), (c['b1'], -1)]:
                mujoco.mj_applyFT(self.m, d, sign * c['frame'].T @ f,
                                 np.zeros(3), c['pos'], body, generalized)
            np.testing.assert_allclose(c['J'].T @ f, generalized, atol=1e-12)

    def test_equilibrium_and_independent_highs(self):
        qp = w.build(self.m, self.x, 'hold', static=True)
        r = w.solve(qp)
        self.assertTrue(r['accepted'], r)
        A, lo, hi = qp['A'], qp['lo'], qp['hi']
        eq = np.isfinite(lo) & (lo == hi)
        upper = np.isfinite(hi) & ~eq; lower = np.isfinite(lo) & ~eq
        ref = linprog(np.zeros(A.shape[1]), A_ub=np.r_[A[upper], -A[lower]],
                      b_ub=np.r_[hi[upper], -lo[lower]], A_eq=A[eq], b_eq=hi[eq],
                      bounds=[(None, None)] * A.shape[1], method='highs')
        self.assertTrue(ref.success)
        bad = r['y'].copy(); bad[9] = -1
        self.assertFalse(w.check(qp, bad)['accepted'])
        bad = r['y'].copy(); bad[9:] = 0
        self.assertFalse(w.check(qp, bad)['accepted'])
        self.assertEqual(len(r['y'][6:9]), 3)

    def test_flight_and_release(self):
        x = self.x.copy(); x[1] += 2
        self.assertFalse(w.solve(w.build(self.m, x, 'hold', static=True))['accepted'])
        hold = w.build(self.m, self.x, 'hold')
        pivot = w.build(self.m, self.x, 'pivot')
        self.assertTrue(any(not c['support'] for c in pivot['contacts']))
        self.assertFalse(np.array_equal(hold['lo'], pivot['lo']) and np.array_equal(hold['hi'], pivot['hi']))
        for c in hold['contacts']:
            if c['wheel_floor']:
                self.assertAlmostEqual(c['Jn'][5], 0.)
        for delta in [-.001, .001]:
            x = self.x.copy(); x[2] += delta
            for mode in ['hold', 'pivot']:
                qp = w.build(self.m, x, mode)
                self.assertTrue(np.isfinite(qp['P']).all())

    def test_reject_no_advance_and_alignment(self):
        qp = w.build(self.m, self.x, 'hold'); result = w.solve(qp)
        for status in ['solved inaccurate', 'primal infeasible']:
            with self.assertRaises(ValueError):
                w.advance(self.m, self.x, qp, dict(result, accepted=False, status=status))
        if result['accepted']:
            out = w.advance(self.m, self.x, qp, result)
            self.assertEqual([r['time_s'] for r in out['substeps']], [0., .002, .004, .006, .008])
            self.assertGreater(sum(g[2] for g in out['substeps'][0]['forces'].values()), 0)
            d = mujoco.MjData(self.m); d.qpos[:] = self.x[:6]; d.qvel[:] = self.x[6:]
            d.ctrl[:] = result['y'][6:9]; mujoco.mj_step(self.m, d, nstep=5)
            np.testing.assert_allclose(out['state'], np.r_[d.qpos, d.qvel], atol=1e-12)
            self.assertFalse(np.asarray(out['external_forces']).any())

if __name__ == '__main__':
    unittest.main()
