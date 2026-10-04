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
# Task-scoped regressions: collect all missing reporting contracts on the base.
missing=[]
if not all('snapshots' in run for run in r['runs']): missing.append('missing initial/rejected same-time snapshots')
if not all(run.get('predictionResidualSamples')==len(run['trace']) and (bool(run['trace']) or run['maxOneStepPredictionResidual'] is None) for run in r['runs']): missing.append('zero-sample residual must be null with sample count')
if not (ROOT/'scripts/constraint_diagnostic_report.py').exists(): missing.append('missing source/count-bound report renderer')
assert not missing, '; '.join(missing)
protocol=json.loads((ROOT/'tests/fixtures/constraint_diagnostic.json').read_text())
m=json.loads((ROOT/protocol['baseTask']).read_text())
assert r['experimentValid'] and r['defaultsChanged'] is False
assert r['protocolSha256']==hashlib.sha256((ROOT/'tests/fixtures/constraint_diagnostic.json').read_bytes()).hexdigest()
assert len(r['runs'])==len(protocol['cases'])*4==20
for path,h in r['sourceSha256'].items():assert hashlib.sha256((ROOT/path).read_bytes()).hexdigest()==h
assert [(c['initialState'],c['goal']) for c in protocol['cases'][1:3]]==[([2.35,.3,.12,-.05],2.05),([-2.35,-.3,-.12,.05],-2.05)]
def compare(row):
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
    check['nativeCategory'] = ('initial-state-outside' if abs(x[0])>2.4 else 'accepted' if native['accepted'] else 'solver-declared infeasible' if 'infeasible' in native['status'] else 'numerically unaccepted/iteration limit')
    check['browserReason']=js.get('reason')
    check.update(browserFirstAction=js['U'][0] if js['accepted'] else None,browserCost=js.get('J'),nativeFirstAction=native.get('action'),nativeCost=native.get('J'),nativeIterations=native.get('iterations'))
    return check
probes=[compare(row) for row in r['probes']]
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
    d=run['design'];target=np.array([case['goal'],0,0,0])
    residuals=[]
    for row in trace:
        predicted=np.asarray(d['A'])@(np.array(row['truth'])-target)+np.asarray(d['B'])[:,0]*row['command']+target
        error=np.array(row['nextTruth'])-predicted
        error[2]=np.arctan2(np.sin(error[2]),np.cos(error[2]))
        residuals.append(np.abs(error))
    np.testing.assert_allclose(run['maxOneStepPredictionResidual'],np.max(residuals,axis=0),atol=1e-14,rtol=0)
    if run['outcome']=='completed':assert len(trace)==600 and run['fullScore'] is not None
    else:assert run['fullScore'] is None
    window=trace[-m['task']['finalWindowSteps']:]
    ep=np.sqrt(np.mean([(t['nextTruth'][0]-case['goal'])**2 for t in window]));ea=np.sqrt(np.mean([t['nextTruth'][2]**2 for t in window]))
    task=run['outcome']=='completed' and len(window)==100 and ep<=.15 and ea<=.06
    assert bool(task)==run['taskPassed']
    # Each accepted row's estimate is based on this/past measurement; next measurement is later.
    assert all(t['nextTime']>t['measurementTime'] for t in trace)
initialization=[]
for run in r['runs']:
    snaps=run['snapshots'];assert snaps[0]['kind']=='initial' and snaps[0]['appliedSteps']==0
    assert len(snaps)==(2 if run['outcome']=='solver-rejected' else 1)
    seen=set()
    for snap in snaps:
        k=snap['appliedSteps'];trace=run['trace']
        assert snap['time']==k*.02 and snap['goal']==next(c['goal'] for c in protocol['cases'] if c['id']==run['caseId'])
        assert snap['design']==run['design'] and snap['pairId']==run['pairId'] and snap['candidateId']==run['candidateId']
        np.testing.assert_array_equal(snap['estimate'],snap['replay']['estimate'])
        np.testing.assert_array_equal(snap['P'],snap['replay']['P'])
        np.testing.assert_array_equal(snap['cold']['estimate'],[snap['measurement'][0],0,snap['measurement'][1],0])
        np.testing.assert_array_equal(snap['cold']['P'],np.diag(run['design']['P0']))
        if k:
            np.testing.assert_array_equal(snap['measurement'],trace[k-1]['nextMeasurement'])
            np.testing.assert_array_equal(snap['truth'],trace[k-1]['nextTruth'])
            np.testing.assert_array_equal(snap['P'],trace[k-1]['nextP'])
            np.testing.assert_array_equal(snap['estimate'],trace[k-1]['nextEstimate'])
            assert snap['fresh']==run['measurementFreshness'][k-1]
        else:
            np.testing.assert_array_equal(snap['measurement'],run['initialMeasurement'])
            np.testing.assert_array_equal(snap['truth'],next(c['initialState'] for c in protocol['cases'] if c['id']==run['caseId']))
        pd=snap['positionDiagnostic'];radius=2*np.sqrt(snap['P'][0][0])
        assert pd['mean']==snap['estimate'][0] and pd['assumedTwoSigma']==radius
        assert pd['error']==snap['estimate'][0]-snap['truth'][0]
        np.testing.assert_array_equal(pd['interval'],[pd['mean']-radius,pd['mean']+radius])
        if snap['kind']=='first-rejected':
            assert k==run['appliedSteps'] and snap['rejectionReason']==run['reason']
            np.testing.assert_array_equal(snap['truth'],run['finalTruth'])
        if 'plans' not in snap or k in seen:continue
        seen.add(k)
        checks={}
        for kind,x in [('continuous',snap['estimate']),('cold',snap['cold']['estimate']),('truth',snap['truth'])]:
            checks[kind]=compare({'caseId':run['caseId'],'design':run['design'],'initialState':x,'goal':snap['goal'],'mpc':snap['plans'][kind]})
        same_time=[s for s in snaps if s['appliedSteps']==k]
        initialization.append({k:v for k,v in snap.items() if k not in ['plans','design']} | {
            'kinds':[s['kind'] for s in same_time], 'rejectionReason':next((s['rejectionReason'] for s in same_time if s['rejectionReason']),None),
            'designSha256':hashlib.sha256(json.dumps(snap['design'],sort_keys=True,separators=(',',':')).encode()).hexdigest(),
            'checks':checks})
summary={**r['summary'],'sourceSha256':r['sourceSha256'],'nativeProbeChecks':probes,'initializationChecks':initialization,'numericalAndInformationChecksPassed':True,'interpretation':'Offline initialization ablation, not executed warm-up. Truth is evaluator-only; solver rejection is not plant impossibility.'}
sys.path.insert(0,str(ROOT/'scripts'))
from constraint_diagnostic_report import digest,render
summary['receiptSha256']=digest(summary)
report=render(summary)
# Both accidental edits and re-sealed inconsistent counts / stale sources fail closed.
import copy
for mutation in ['content','counts','source','protocol']:
    bad=copy.deepcopy(summary)
    if mutation in ['content','counts']:bad['counts']['taskPassed']+=1
    elif mutation=='source':bad['sourceSha256']['src/engine.js']='0'*64
    else:bad['protocolSha256']='0'*64
    if mutation!='content':bad['receiptSha256']=digest(bad)
    try:render(bad)
    except AssertionError:pass
    else:raise AssertionError('Renderer accepted '+mutation)
(ROOT/'evidence/constraint_diagnostic_summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
(ROOT/'evidence/constraint_diagnostic_report.md').write_text(report)
print('CONSTRAINT_DIAGNOSTIC_RESULT '+json.dumps(summary['counts']))
print('INITIALIZATION_CHECKS',len(initialization),'REPORT_BINDING_NEGATIVE_TESTS',4)
