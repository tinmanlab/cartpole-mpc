"""Scratch reuse must preserve the official inverse and failed-command gates."""
from pathlib import Path
import sys, unittest
import numpy as np
import mujoco
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import wheelbot_contact_wbc as w

class WorkspaceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.m=w.model()
    def test_reused_official_inverse_matches_fresh(self):
        data=mujoco.MjData(self.m)
        for sign in [-1,1]:
            state=w.settle(self.m,sign)
            for a in [np.zeros(6),np.array([.05,.2,1,-2,1,.2]),np.array([0,0,-1,2,-1,-.2])]:
                force,jac,_=w.inverse_eval(self.m,state,a,True)
                reused,jac_reused,_=w.inverse_eval(self.m,state,a,True,scratch=data)
                np.testing.assert_array_equal(force,reused)
                np.testing.assert_array_equal(jac,jac_reused)
    def test_optimized_controller_preserves_command(self):
        state=w.settle(self.m,1)
        old=w.ContactInverseController(self.m).solve(state,1.,.25)
        new=w.ContactInverseController(self.m,reuse_workspace=True).solve(state,1.,.25)
        self.assertTrue(old['accepted'] and new['accepted'])
        np.testing.assert_allclose(old['tau'],new['tau'],atol=1e-10,rtol=0)
        np.testing.assert_allclose(old['qacc'],new['qacc'],atol=1e-10,rtol=0)
        with self.assertRaises(ValueError):
            w.advance_inverse(self.m,state,dict(new,accepted=False))
    def test_iteration_budget_is_explicit(self):
        with self.assertRaises(ValueError):w.ContactInverseController(self.m,max_iterations=0)
        with self.assertRaises(ValueError):w.ContactInverseController(self.m,max_iterations=1.5)
        default=w.ContactInverseController(self.m)
        self.assertEqual(default.max_iterations,w.INVERSE_PROTOCOL['maxiter'])

if __name__=='__main__':unittest.main()
