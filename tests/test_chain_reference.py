"""Native MuJoCo replay, mathematical checks, and independent OSQP problem probes."""
import json,sys,hashlib
from pathlib import Path
import numpy as np,mujoco
from scipy import linalg as la
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from chain_native import ChainQP
profiles=json.loads((ROOT/'assets/chains/profiles.json').read_text())['profiles']
wasm=json.loads((ROOT/'test-results/chain_wasm.json').read_text())['rows'];rows=[]
for p,w in zip(profiles,wasm):
    assert p['poles']==w['poles'];m=mujoco.MjModel.from_xml_path(str(ROOT/p['asset']));d=mujoco.MjData(m);dof=m.nv
    d.qpos[:]=w['x0'][:dof];d.qvel[:]=w['x0'][dof:];errors=[]
    for u,expected in zip(w['controls'],w['states']):
        d.ctrl[0]=u
        for _ in range(4):mujoco.mj_step(m,d)
        actual=np.r_[d.qpos,d.qvel];diff=actual-np.asarray(expected);diff[1:dof]=np.arctan2(np.sin(diff[1:dof]),np.cos(diff[1:dof]));errors.append(float(np.max(np.abs(diff))))
    assert max(errors)<1e-8,(p['poles'],max(errors))
    A,B,Q,P=[np.asarray(p[k]) for k in ['A','B','Q','P']];K=np.asarray(p['K']);H=np.asarray(p['H']);L=np.asarray(p['kalmanGain'])
    assert p['pbhControlSigmaMin']>1e-8 and p['pbhAllEncoderObservableSigmaMin']>1e-8
    computedK=la.solve(np.array([[p['R']]])+B.T@P@B,B.T@P@A)
    np.testing.assert_allclose(K,computedK,rtol=1e-9,atol=1e-9)
    residual=la.norm(A.T@P@A-P-A.T@P@B@K+Q,np.inf)/max(1,la.norm(P,np.inf))
    assert abs(residual-p['dareNormalizedResidual'])<1e-12
    assert bool(p['designAvailable'])==bool(residual<=1e-9 and la.eigvalsh(P)[0]>0)
    row={'poles':p['poles'],'nativeReplayMaxError':max(errors),'controlPBHMinimum':p['pbhControlSigmaMin'],'allEncoderObservationPBHMinimum':p['pbhAllEncoderObservableSigmaMin'],'oldTwoSensorObservationPBHMinimum':p['pbhOldTwoSensorSigmaMin'],'designAvailable':p['designAvailable'],'dareCondition':p['dareCondition'],'dareNormalizedResidual':p['dareNormalizedResidual']}
    if p['designAvailable']:
        assert max(abs(la.eigvals(A-B@K)))<1
        assert max(abs(la.eigvals((np.eye(p['nx'])-L@H)@A)))<1
        s=ChainQP(p,.1);x=np.zeros(p['nx']);x[1]=.003;r=s.solve(x);row['osqpFrozenProbe']={k:v for k,v in r.items() if k!='inputSequence'}
        browser=w['qpProbe'];row['matchingBrowserProbe']=browser
        if r['accepted'] and browser['accepted']:
            action_error=abs(r['action']-browser['action']);cost_error=abs(r['cost']-browser['cost'])/max(1,abs(r['cost']))
            assert action_error<.001 and cost_error<1e-4
            row['pairedQpParity']={'firstActionError':action_error,'relativeCostError':cost_error,'passed':True}
        else:row['pairedQpParity']={'passed':None,'scope':'Not both admitted; no numerical-equivalence claim'}
        assert s.solve([np.nan]*p['nx'])['action'] is None
        x[0]=2.5;assert s.solve(x)['action'] is None
    else:
        assert p['dareNormalizedResidual']>1e-9
    # Refinement against smaller-step native RK4 is separate from WASM/native parity.
    # Unit scales: 1 m, 1 rad per hinge, 1 m/s, 1 rad/s per hinge.
    def replay(substeps,integrator):
        mm=mujoco.MjModel.from_xml_path(str(ROOT/p['asset']));dd=mujoco.MjData(mm)
        mm.opt.timestep=.02/substeps;mm.opt.integrator=integrator
        dd.qpos[:]=w['x0'][:dof];dd.qvel[:]=w['x0'][dof:];xx=[]
        for k in range(10):
            dd.ctrl[0]=.02*np.sin(.2*k)
            for _ in range(substeps):mujoco.mj_step(mm,dd)
            xx.append(np.r_[dd.qpos,dd.qvel])
        return np.array(xx)
    reference=replay(64,mujoco.mjtIntegrator.mjINT_RK4)
    refinement=[]
    for substeps in [1,4,16]:
        difference=replay(substeps,mujoco.mjtIntegrator.mjINT_EULER)-reference
        difference[:,1:dof]=np.arctan2(np.sin(difference[:,1:dof]),np.cos(difference[:,1:dof]))
        refinement.append({'substeps':substeps,'normalizedRmsError':float(np.sqrt(np.mean(difference**2)))})
    assert refinement[2]['normalizedRmsError']<refinement[1]['normalizedRmsError']<refinement[0]['normalizedRmsError']
    row['shortReplayRefinement']=refinement
    rows.append(row)
report={'schema':'cartpole-chain-reference/v1','rows':rows,'passed':True,'mujoco':mujoco.__version__,
 'limits':{'nativeWasmReplay':1e-8,'osqpPrimalViolation':1e-7,'osqpNormalizedStationarity':1e-7},
 'interpretation':'PBH at one equilibrium and nominal parameters is numerical local evidence. Native normalized stationarity differs from the browser absolute KKT gate; neither rejected numeric attempt proves physical impossibility.'}
(ROOT/'evidence/chain_reference.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n');print(json.dumps(rows,indent=2))
