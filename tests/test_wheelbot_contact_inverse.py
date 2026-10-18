import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import wheelbot_contact_wbc as w
import numpy as np
import mujoco

class ContactInverse(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.m = w.model()
        cls.x = w.settle(cls.m, 1)

    def test_inverse_jacobian_and_agreement(self):
        c = w.ContactInverseController(self.m)
        for target in [(0., 0.), (1., .25)]:
            r = c.solve(self.x, *target)
            self.assertTrue(r['accepted'], r)
            f, J, _ = w.inverse_eval(self.m, self.x, r['qacc'], True)
            eps = 1e-6
            fd = np.column_stack([(w.inverse_eval(self.m, self.x, r['qacc']+np.eye(6)[j]*eps)[0]-f)/eps for j in range(6)])
            np.testing.assert_allclose(J, fd, atol=1e-7)
            out = w.advance_inverse(self.m, self.x, r)
            self.assertEqual(len(out['substeps']), 5)
            for row in out['substeps']:
                self.assertTrue(all(p['local_force'][0] >= -1e-12 for p in row['contacts']))
                self.assertEqual(len(row['actuator_force']), 3)
            self.assertFalse(np.asarray(out['external_forces']).any())

    def test_rigid_contrast(self):
        q = w.build(self.m, self.x, 'pivot'); r = w.solve(q)
        self.assertTrue(r['accepted'])
        f, _, _ = w.inverse_eval(self.m, self.x, r['y'][:6])
        self.assertGreater(np.max(np.abs(f[:3])), 1e-7)

    def test_geometry_and_model(self):
        self.assertEqual(w.sha(w.XML), w.INVERSE_PROTOCOL['model_sha256'])
        self.assertEqual(int(self.m.opt.enableflags), 0)
        np.testing.assert_array_equal(self.m.actuator_ctrlrange[:, 1], [16,16,1.7])
        for sign in [-1, 1]:
            x = w.settle(self.m, sign); g = w.material_geometry(self.m, x)
            self.assertLess(abs(g['edge'][2]), .002)
            d, _, _, _ = w.inspect(self.m, x)
            b = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, 'torso')
            local = d.xmat[b].reshape(3,3).T @ (np.array(g['edge'])-d.xpos[b])
            np.testing.assert_allclose(local, [np.sign(x[2])*.0625,0,0], atol=1e-12)

    def test_reference_endpoints_and_trace_contract(self):
        for sign in [-1, 1]:
            initial = w.settle(self.m, sign)
            x = initial.copy(); x[2] += np.sign(initial[2])*.08
            side = np.sign(initial[2])*.0625
            offset = lambda a: side*np.sin(a)-.25*np.cos(a)
            x[1] += offset(x[2])-offset(initial[2]); x[6:] = 0
            np.testing.assert_allclose(w.inverse_reference(initial,x,.5), [0,0], atol=1e-12)
            self.assertTrue(w.ContactInverseController(self.m).solve(initial,0.,0.)['accepted'])

    def test_rejections(self):
        r = w.ContactInverseController(self.m).solve(self.x, 1., .25)
        self.assertTrue(r['accepted'])
        for key, value in [('accepted', False), ('success', False), ('qacc', np.full(6,np.nan)), ('tau', np.array([17.,0,0])), ('qacc', r['qacc']+1), ('status', 9), ('state', self.x+.001)]:
            before = self.x.copy()
            with self.assertRaises(ValueError):
                w.advance_inverse(self.m, self.x, dict(r, **{key:value}))
            np.testing.assert_array_equal(self.x, before)

if __name__ == '__main__': unittest.main()
