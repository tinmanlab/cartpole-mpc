"""Runtime rejection contracts; equations/replay checked against offline evidence."""
import copy
import importlib.util
import json
from pathlib import Path
import sys
import unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))

class TrackingTests(unittest.TestCase):
    def test_selected_derivative_mapping(self):
        from wheelbot_contact_tracking import selected_derivatives_validated as gate
        branch={'contacts':True,'constraints':True}
        checks=[{'sign':1,'step':0,'kind':'x','column':0,'relative_refinement_error':.01,'branches':[branch.copy(),branch.copy()]}]
        expected={(1,0,'x',0)}
        self.assertTrue(gate(checks,[],1.,expected))
        checks[0]['branches'][0]['contacts']=False
        refinement=[{'sign':1,'step':0,'kind':'x','column':0,'same_branch':[True,True],'relative_error':1e-8}]
        self.assertTrue(gate(checks,refinement,1.,expected))
        for bad in [[],refinement*2,[dict(refinement[0],column=1)],[dict(refinement[0],same_branch=[True,False])]]:
            self.assertFalse(gate(checks,bad,1.,expected))
        for error in [float('nan'),float('inf'),.1,.2,-1.]:
            self.assertFalse(gate(checks,[dict(refinement[0],relative_error=error)],1.,expected))
        for condition in [float('nan'),float('inf'),1e10+1]:
            self.assertFalse(gate(checks,refinement,condition,expected))
        for error in [float('nan'),float('inf')]:
            self.assertFalse(gate([dict(checks[0],relative_refinement_error=error)],refinement,1.,expected))
        self.assertFalse(gate(checks*2,refinement,1.,expected))
        self.assertFalse(gate([],[],1.,expected))
        checks[0]['branches'][1]['constraints']=False
        self.assertFalse(gate(checks,refinement,1.,expected))
        checks[0]['branches']=[branch.copy(),branch.copy()]
        self.assertFalse(gate(checks,refinement,1.,expected))  # unmatched, unnecessary refinement
        checks[0]['relative_refinement_error']=.2
        self.assertTrue(gate(checks,refinement,1.,expected))  # excessive coarse step

    def test_derivative_approval_required(self):
        import wheelbot_contact_tracking as t
        p=json.loads((ROOT/'assets/wheelbot/contact_tracking_profile.json').read_text())
        for flag in [False,None,1,'true']:
            bad=copy.deepcopy(p);bad['branchDiagnostics']['selected_derivatives_validated']=flag
            with self.assertRaises(ValueError):t.TrackingController(bad,p['identities'])
        bad=copy.deepcopy(p);bad['branchDiagnostics'].pop('selected_derivatives_validated',None)
        with self.assertRaises(ValueError):t.TrackingController(bad,p['identities'])

    def test_module_exists(self):
        self.assertIsNotNone(importlib.util.find_spec('wheelbot_contact_tracking'))

    def test_runtime_contracts(self):
        import wheelbot_contact_tracking as t
        path=ROOT/'assets/wheelbot/contact_tracking_profile.json'
        p=json.loads(path.read_text())
        ids=p['identities']
        c=t.TrackingController(p,ids)
        x=p['trajectories']['1']['ref'][0]
        np.testing.assert_allclose(c.command(x,0,1),p['trajectories']['1']['u'][0],atol=1e-14)
        for state,k,sign,stage in [(x,-1,1,'tracking'),(x,250,1,'tracking'),(x,0,0,'tracking'),(x,0,1,'standing'),([float('nan')]*12,0,1,'tracking'),(x[:11],0,1,'tracking'),(x,.5,1,'tracking')]:
            with self.assertRaises(ValueError):c.command(state,k,sign,stage)
        for key in ['model_sha256','cost_sha256','trace_sha256','protocol_sha256']:
            wrong=dict(ids);wrong[key]='0'*64
            with self.assertRaises(ValueError):t.TrackingController(p,wrong)
        for mutation in ['indices','shape','nan','limits','schema']:
            bad=copy.deepcopy(p)
            if mutation=='indices':bad['controlledIndices'][0]=12
            elif mutation=='shape':bad['trajectories']['1']['K'].pop()
            elif mutation=='nan':bad['trajectories']['1']['K'][0][0][0]=float('nan')
            elif mutation=='limits':bad['limitsNm'][0]=160
            else:bad['schema']='wrong'
            with self.assertRaises(ValueError):t.TrackingController(bad,ids)
        with self.assertRaises((ValueError,FileNotFoundError)):t.load_controller('/tmp/contact-tracking-missing-profile.json',ids,active_xml=ROOT/'assets/wheelbot/contact_model.xml')
        with self.assertRaises(ValueError):t.load_controller(path,ids,active_xml=ROOT/'tests/fixtures/wheelbot_contact_tracking.json')
        with self.assertRaises(ValueError):t.TrackingController({},ids)
        y=np.array(x);y[3]+=.005
        expected=np.array(p['trajectories']['1']['u'][0])-np.array(p['trajectories']['1']['K'][0])@(y-np.array(x))[t.INDICES]
        np.testing.assert_allclose(c.command(y,0,1),np.clip(expected,-c.limits,c.limits))

    def test_native_replay(self):
        import hashlib
        import mujoco
        import wheelbot_contact_tracking as t
        p=json.loads((ROOT/'assets/wheelbot/contact_tracking_profile.json').read_text())
        xml=ROOT/'assets/wheelbot/contact_model.xml'
        self.assertEqual(hashlib.sha256(xml.read_bytes()).hexdigest(),p['identities']['model_sha256'])
        m=mujoco.MjModel.from_xml_path(str(xml));d=mujoco.MjData(m)
        c=t.load_controller(ROOT/'assets/wheelbot/contact_tracking_profile.json',p['identities'],active_xml=xml)
        for sign in [-1,1]:
            ref=np.array(p['trajectories'][str(sign)]['ref']);x=ref[0].copy()
            for k in range(250):
                u=c.command(x,k,sign)
                mujoco.mj_resetData(m,d);d.qpos[:]=x[:6];d.qvel[:]=x[6:];d.ctrl[:]=u
                mujoco.mj_step(m,d);x=np.r_[d.qpos,d.qvel]
                np.testing.assert_allclose(x,ref[k+1],atol=1e-10,rtol=0)
                self.assertFalse(d.qfrc_applied.any() or d.xfrc_applied.any())

    def test_offline_equations_and_replay(self):
        p=json.loads((ROOT/'assets/wheelbot/contact_tracking_profile.json').read_text())
        raw=json.loads((ROOT/'test-results/contact-tracking-design.json').read_text())
        Q=np.array(p['Q']);R=np.array(p['R'])
        for sign in ['-1','1']:
            P=Q.copy();row=raw['trajectories'][sign]
            for A,B,K in reversed(list(zip(row['A'],row['B'],p['trajectories'][sign]['K']))):
                A=np.array(A);B=np.array(B);K=np.array(K)
                np.testing.assert_allclose((R+B.T@P@B)@K,B.T@P@A,rtol=1e-9,atol=1e-9)
                P=Q+A.T@P@A-A.T@P@B@K;P=(P+P.T)/2
            self.assertLess(row['max_replay_error'],1e-10)

if __name__=='__main__':unittest.main()
