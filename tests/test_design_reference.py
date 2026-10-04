"""Independent SciPy/OSQP design, objective, ranking and admission reconstruction.
Actual task outcomes are evidence, never a forced passing-score expectation.
"""
from pathlib import Path
import hashlib,json,sys
import numpy as np
from scipy.linalg import solve_discrete_are
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'tests'))
from test_native_reference import solve_qp
m=json.loads((ROOT/'tests/fixtures/design_study.json').read_text())
r=json.loads((ROOT/'evidence/design_study.json').read_text());full=json.loads((ROOT/'test-results/design_study_full.json').read_text())
assert r['experimentValid'] and r['selectionLockedBeforeTest'] and not r['selection']['testUsedForSelection']
assert r['manifestSha256']==hashlib.sha256((ROOT/'tests/fixtures/design_study.json').read_bytes()).hexdigest()
assert all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in r['sourceSha256'].items())
measurements=np.array(full['capture']['measurements']);anchor=measurements[0,1];measurements[:,1]=anchor+np.arctan2(np.sin(measurements[:,1]-anchor),np.cos(measurements[:,1]-anchor))
cov=np.cov(measurements,rowvar=False,ddof=1);Rerror=float(np.max(np.abs(cov-np.array(r['fit']['covariance']))));assert Rerror<1e-12
params={};maxP=0.;maxK=0.;frames=0;maxScore=0.;allkeys=set()
scales=m['task']['objective'];norm=np.array([scales[k]['weight']/scales[k]['scale']**2 for k in ['position','angle','force','deltaForce']])
for run in full['auditRuns']:
    key=(run['phase'],run['pairId'],run['candidateId'],run['caseId']);assert key not in allkeys,key;allkeys.add(key)
    design=run['design'];Q=np.diag(design['Qc']);Rc=design['Rc'];A=np.array(design['A']);B=np.array(design['B'])
    expectedQ=1/np.square(m['theoryDesign']['stateScales']);np.testing.assert_allclose(np.diag(Q),expectedQ,rtol=0,atol=1e-14)
    assert Rc==design['candidate']['effortMultiplier']/m['theoryDesign']['forceScale_N']**2
    np.testing.assert_allclose(design['Qe'],np.array(m['theoryDesign']['QeBase'])*design['candidate']['processMultiplier'],rtol=0,atol=1e-16)
    np.testing.assert_array_equal(design['Re'],r['fit']['Rdiag']);np.testing.assert_array_equal(design['P0'],m['theoryDesign']['P0diag'])
    if Rc not in params:
        P=solve_discrete_are(A,B,Q,np.array([[Rc]]));K=np.linalg.solve(np.array([[Rc]])+B.T@P@B,B.T@P@A)
        params[Rc]=(P,K)
    P,K=params[Rc];maxP=max(maxP,float(np.max(np.abs(P-design['Pf']))));maxK=max(maxK,float(np.max(np.abs(K.ravel()-design['K']))))
    assert maxP<1e-6 and maxK<1e-7
    a=np.array(run['auditSeries'],float).reshape(-1,11);frames+=len(a);assert len(a)==run['appliedSteps']
    assert run['information']['controller']=='observer-output only'
    if not len(a):assert run['fullScore'] is None and not run['taskPassed'];continue
    assert np.max(np.abs(a[:,2]))<=m['task']['forceLimit_N']+1e-8
    np.testing.assert_allclose(a[:,3],np.r_[a[0,2],np.diff(a[:,2])],rtol=0,atol=1e-12)
    part=np.mean(a[:,:4]**2*norm,axis=0);score=float(np.sum(part))
    for value,name in zip(part,['position','angle','force','deltaForce']):assert abs(value-run['scoreComponents'][name])<1e-9
    assert abs(score-run['partialScore'])<1e-9
    if run['outcome']=='completed':maxScore=max(maxScore,abs(score-run['fullScore']));assert len(a)==m['steps']
    else:assert run['fullScore'] is None
    np.testing.assert_allclose(np.sqrt(np.mean(a[:,4:8]**2,axis=0)),run['estimationRmseByState'],atol=1e-10)
    cases=m['training']+m['validation']+m['test'];case=next(t for t in cases if t['id']==run['caseId']);w=a[-m['task']['finalWindowSteps']:]
    tailp=float(np.sqrt(np.mean((w[:,8]-case['goal'])**2)));tailt=float(np.sqrt(np.mean(w[:,9]**2)))
    assert abs(tailp-run['tailPositionRms'])<1e-10 and abs(tailt-run['tailAngleRms'])<1e-10
    task=run['outcome']=='completed' and len(w)==m['task']['finalWindowSteps'] and tailp<=m['task']['finalPositionRms_m'] and tailt<=m['task']['finalAngleRms_rad']
    assert bool(task)==run['taskPassed'];assert sum(abs(u)>=10-1e-8 for u in a[:,2])==run['saturatedSamples']
assert maxScore<1e-9
# Independently recompute every candidate aggregate, shortlist and selected pair.
def stat(rows,c):
    return {'candidate':c,'hardFailures':sum(x['outcome']!='completed' for x in rows),'taskFailures':sum(not x['taskPassed'] for x in rows),'fullScore':float(np.mean([x['fullScore'] for x in rows])) if all(x['outcome']=='completed' for x in rows) else None}
def rank(x):return (x['hardFailures'],x['taskFailures'],x['fullScore'] if x['fullScore'] is not None else float('inf'),0 if x['candidate']['baseline'] else 1,x['candidate']['id'])
selected=[]
for pair in r['selection']['pairs']:
    training=[]
    for stored in pair['training']:
        c=stored['candidate'];rr=[q for q in full['auditRuns'] if q['phase']=='training' and q['pairId']==pair['pairId'] and q['candidateId']==c['id']]
        assert [q['caseId'] for q in rr]==[t['id'] for t in m['training']]
        s=stat(rr,c);assert s['hardFailures']==stored['hardFailures'] and s['taskFailures']==stored['taskFailures'];training.append(s)
    assert len(training)==9
    shortlist=[s['candidate']['id'] for s in sorted(training,key=rank)[:m['candidateGrid']['shortlist']]]
    baseline=next(s['candidate']['id'] for s in training if s['candidate']['baseline'])
    if baseline not in shortlist:shortlist.append(baseline)
    assert shortlist==[s['candidate']['id'] for s in pair['validation']]
    val=[]
    for stored in pair['validation']:
        c=stored['candidate'];rr=[q for q in full['auditRuns'] if q['phase']=='validation' and q['pairId']==pair['pairId'] and q['candidateId']==c['id']]
        assert [q['caseId'] for q in rr]==[t['id'] for t in m['validation']]
        val.append(stat(rr,c))
    best=sorted(val,key=rank)[0];assert best['candidate']['id']==pair['candidate']['id']
    ok=best['hardFailures']==0 and best['taskFailures']==0;assert ok==pair['eligible']
    if ok:selected.append({'pairId':pair['pairId'],'score':best['fullScore']})
if selected:
    low=min(p['score'] for p in selected);near=[p for p in selected if p['score']<=low*(1+m['selection']['crossPairTieRelative'])]
    chosen=min(near,key=lambda p:next(q['simplicityOrder'] for q in m['pairs'] if q['id']==p['pairId']))['pairId']
else:chosen=None
assert chosen==r['selection']['recommendedPairId']==r['admission']['recommendedPairId']
rr=[p for p in r['tests'] if p['pairId']==chosen and p['group']=='primary'];both=all(p['baseline']['outcome']==p['candidate']['outcome']=='completed' for p in rr)
accepted=bool(chosen and len(rr)==4 and all(p['candidate']['taskPassed'] for p in rr) and not any(p['baseline']['taskPassed'] and not p['candidate']['taskPassed'] for p in rr) and both and np.mean([p['candidate']['fullScore'] for p in rr])<=1.02*np.mean([p['baseline']['fullScore'] for p in rr]))
assert accepted==r['admission']['accepted']
qpRows=[]
for row in full['tests']:
 if not row['pairId'].startswith('mpc-'):continue
 for arm in ['baseline','candidate']:
    run=row[arm]
    if not run['trace']:continue
    d=run['design'];f=run['trace'][0];x=np.array(f['estimate']);x[0]-=f['goal']
    q=solve_qp({'A':d['A'],'B':d['B'],'Q':np.diag(d['Qc']).tolist(),'Qf':d['Pf'],'R':d['Rc'],'N':d['horizon'],'limit':10,'x':x.tolist()})
    # Independent box-only solution agrees when the actual rail isn't active; verify that condition first.
    if q['accepted'] and q.get('max_position',0)+abs(f['goal'])<2.4:
        delta=abs(q['action']-f['command']);assert delta<.001
        qpRows.append({'caseId':row['caseId'],'pairId':row['pairId'],'arm':arm,'firstActionError':delta})
assert len(qpRows)>=8
out={'schema':'cartpole-task-design-reference/v1','passed':True,'measurementCovarianceError':Rerror,'riccatiMatrixMaxError':maxP,'gainMaxError':maxK,'scoredDynamicFrames':frames,'maximumScoreError':maxScore,'candidateRankAndAdmissionReconstructed':True,'lockedRecommendation':chosen,'accepted':accepted,'independentQpChecks':qpRows,'evidenceSha256':hashlib.sha256((ROOT/'evidence/design_study.json').read_bytes()).hexdigest(),'sourceSha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'scope':'Computational and selection-contract verification, not performance dominance, rare-failure certification or deployment authority'}
(ROOT/'evidence/design_reference.json').write_text(json.dumps(out,indent=2,allow_nan=False)+'\n');print(json.dumps({k:v for k,v in out.items() if k!='independentQpChecks'},indent=2))
