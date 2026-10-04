"""Optional native NMPC contracts; no hardware, network server or browser dependency."""
import sys, unittest
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from native_nmpc import NativeNMPC, read_model, discrete_model, audit_plan

class NativeContracts(unittest.TestCase):
    def test_asset_parameters_and_model_equilibrium(self):
        p=read_model(ROOT/'assets/cartpole.xml')
        self.assertAlmostEqual(p['l'],.5)
        self.assertAlmostEqual(p['inertia'],p['mp']*p['l']**2/3)
        f,_,_=discrete_model(p)
        np.testing.assert_allclose(np.asarray(f(np.zeros(4),0)).ravel(),np.zeros(4),atol=1e-14)
    def test_invalid_asset_rejected(self):
        import tempfile
        s=(ROOT/'assets/cartpole.xml').read_text().replace('axis="0 1 0"','axis="1 0 0"')
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'bad.xml';path.write_text(s)
            with self.assertRaises(ValueError):read_model(path)
    def test_status_does_not_make_invalid_plan_admissible(self):
        f,_,_=discrete_model(read_model(ROOT/'assets/cartpole.xml'))
        X=np.zeros((31,4));U=np.zeros((30,1))
        good=audit_plan(X,U,np.zeros(4),0,f)
        self.assertTrue(good['accepted'])
        X[5,0]=2.5
        self.assertFalse(audit_plan(X,U,np.zeros(4),0,f)['accepted'])
        X[:]=0;U[2]=np.nan
        self.assertFalse(audit_plan(X,U,np.zeros(4),0,f)['accepted'])
    def test_native_solver_cost_and_rejection(self):
        solver=NativeNMPC('SQP')
        r=solver.solve(np.array([.3,.1,.08,-.05]),0)
        self.assertTrue(r['accepted'],r)
        self.assertLess(r['cost_error'],1e-6)
        self.assertLess(r['max_dynamics_defect'],1e-5)
        self.assertLessEqual(abs(r['action']),10+1e-8)
        for x in [[np.nan,0,0,0],[2.5,0,0,0],[0,0]]:
            bad=solver.solve(x,0)
            self.assertFalse(bad['accepted'])
            self.assertIsNone(bad['action'])

    def test_dare_terminal_retains_cross_terms_and_matches_cost(self):
        solver=NativeNMPC('SQP',terminal='dare')
        self.assertGreater(np.max(np.abs(solver.terminal-np.diag(np.diag(solver.terminal)))),1)
        self.assertTrue(np.linalg.eigvalsh(solver.terminal).min()>0)
        result=solver.solve([.1,0,.02,0],.5)
        self.assertTrue(result['accepted'],result)
        self.assertLess(result['cost_error'],1e-6)
    def test_rti_recovery_never_applies_rejected_primary(self):
        rti=NativeNMPC('SQP_RTI');backup=NativeNMPC('SQP')
        result=rti.solve_with_recovery([.01762466701618987,0,.041833939640195196,0],.5,backup)
        self.assertTrue(result['accepted'],result)
        self.assertTrue(result['fallbackUsed'])
        self.assertEqual(result['solverAlgorithmApplied'],'SQP')
        self.assertGreater(result['primaryDefect'],1e-5)

if __name__=='__main__':unittest.main()
