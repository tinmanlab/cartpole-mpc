"""F2 diagnostic verification; implementation correctness is not a required winner.
Uses the existing independent condensed OSQP transcription. An extra constant
coordinate represents the nonzero reference exactly, without changing that solver.
"""
from pathlib import Path
import hashlib,json,subprocess,sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tests'))
from osqp_condensed_reference import solve_active_rail
script=ROOT/'scripts/run_constraint_diagnostic.mjs'
assert script.exists(),'Missing separate constraint-active diagnostic'
subprocess.run(['node',str(script)],cwd=ROOT,check=True,timeout=120)
r=json.loads((ROOT/'test-results/constraint_diagnostic_full.json').read_text())
protocol=json.loads((ROOT/'tests/fixtures/constraint_diagnostic.json').read_text())
m=json.loads((ROOT/protocol['baseTask']).read_text())
assert r['experimentValid'] and r['defaultsChanged'] is False
assert r['protocolSha256']==hashlib.sha256((ROOT/'tests/fixtures/constraint_diagnostic.json').read_bytes()).hexdigest()
assert len(r['runs'])==len(protocol['cases'])*4==20
for path,h in r['sourceSha256'].items():assert hashlib.sha256((ROOT/path).read_bytes()).hexdigest()==h
assert [(c['initialState'],c['goal']) for c in protocol['cases'][1:3]]==[([2.35,.3,.12,-.05],2.05),([-2.35,-.3,-.12,.05],-2.05)]
probes=[]
for row in r['probes']:
    d=row['design'];A=np.asarray(d['A']);B=np.asarray(d['B']);Q=np.diag(d['Qc']);P=np.asarray(d['Pf']);x=np.asarray(row['initialState']);g=row['goal']
    # z=[world-state,1], error=T*z. Constant coordinate is algebra, not another plant.
    target=np.array([g,0,0,0.]);T=np.column_stack([np.eye(4),-target]);AA=np.eye(5);AA[:4,:4]=A;AA[:4,4]=(np.eye(4)-A)@target
    BB=np.vstack([B,np.zeros((1,1))]);xx=np.r_[x,1.]
    native=solve_active_rail({'A':AA.tolist(),'B':BB.tolist(),'Q':(T.T@Q@T).tolist(),'Qf':(T.T@P@T).tolist(),'R':d['Rc'],'N':d['horizon'],'limit':10,'x':xx.tolist()},2.4)
    js=row['mpc'];check={'caseId':row['caseId'],'browserAccepted':js['accepted'],'nativeAccepted':native['accepted'],'nativeStatus':native['status'],'sameProblemCompared':False}
    if js['accepted']:
        assert js['kktResidual']<=1e-7 and js['primalResidual']<=1e-8
        X=np.array(js['X']);U=np.array(js['U']);E=X-target
        assert np.max(np.abs(U))<=10+1e-8 and np.max(np.abs(X[:,0]))<=2.4+1e-8
        np.testing.assert_allclose(E[1:],E[:-1]@A.T+U[:,None]*B[:,0],atol=1e-8,rtol=0)
        J=sum(e@Q@e+d['Rc']*u*u for e,u in zip(E[:-1],U))+E[-1]@P@E[-1]
        assert abs(J-js['J'])<=1e-7*max(1,abs(J))
        assert js['railActive']==bool(np.any(np.abs(X[:,0])>=2.4-1e-6))
    if js['accepted'] and native['accepted']:
        delta=abs(js['U'][0]-native['action']);cost=abs(js['J']-native['J'])/max(1,abs(native['J']))
        assert delta<=.001 and cost<=1e-4
        check.update(sameProblemCompared=True,firstActionError=float(delta),relativeCostError=float(cost))
    probes.append(check)
assert any(p['sameProblemCompared'] for p in probes),'No common admitted probe; numerical comparison unavailable'
for run in r['runs']:
    case=next(c for c in protocol['cases'] if c['id']==run['caseId']);trace=run['trace']
    assert run['information']['controller']=='observer-output only'
    assert run['candidateId']=='r1-q1' and run['requestedSteps']==600
    assert run['appliedSteps']==len(trace) and run['outcome'] in ['completed','solver-rejected','envelope-failure']
    assert run['initialEstimate'][1]==run['initialEstimate'][3]==0
    np.testing.assert_array_equal(run['initialEstimate'],[run['initialMeasurement'][0],0,run['initialMeasurement'][1],0])
    if not trace:assert run['fullScore'] is None and not run['taskPassed'];continue
    assert all(abs(t['command'])<=10+1e-8 for t in trace)
    assert run['finalTruth']==trace[-1]['nextTruth']
    if run['outcome']=='completed':assert len(trace)==600 and run['fullScore'] is not None
    else:assert run['fullScore'] is None
    window=trace[-m['task']['finalWindowSteps']:]
    ep=np.sqrt(np.mean([(t['nextTruth'][0]-case['goal'])**2 for t in window]));ea=np.sqrt(np.mean([t['nextTruth'][2]**2 for t in window]))
    task=run['outcome']=='completed' and len(window)==100 and ep<=.15 and ea<=.06
    assert bool(task)==run['taskPassed']
    # Each accepted row's estimate is based on this/past measurement; next measurement is later.
    assert all(t['nextTime']>t['measurementTime'] for t in trace)
summary={**r['summary'],'nativeProbeChecks':probes,'numericalAndInformationChecksPassed':True,'interpretation':'A failed or rejected physical trial is a valid diagnostic result. No algorithm win is required; exact-state planning is not sensor-only closed-loop evidence.'}
(ROOT/'evidence/constraint_diagnostic_summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
print('CONSTRAINT_DIAGNOSTIC_RESULT '+json.dumps(summary,allow_nan=False))
