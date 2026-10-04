"""Independent reconstruction from real raw trajectories, not tuner self-report.
Rechecks task/loss, phase coverage, budgets, selected recipes and statistical units.
"""
from pathlib import Path
from collections import defaultdict,Counter
import hashlib,gzip,json,math,sys
import numpy as np
from scipy.stats import bootstrap
ROOT=Path(__file__).resolve().parents[1]
p=json.loads((ROOT/'tests/fixtures/sequential_tuning.json').read_text());m=json.loads((ROOT/p['taskManifest']).read_text())
r=json.loads((ROOT/'evidence/sequential_tuning.json').read_text());folder=ROOT/'test-results/sequential-tuning'
assert r['mode']=='frozen-full' and r['experimentValid'] and r['sourcesStable']
assert r['protocolSha256']==hashlib.sha256((ROOT/'tests/fixtures/sequential_tuning.json').read_bytes()).hexdigest()
assert all(hashlib.sha256((ROOT/f).read_bytes()).hexdigest()==h for f,h in r['sourceSha256'].items())
lock=json.loads((folder/'recommendations-locked.json').read_text());assert lock['testEvaluations']==0
locked={x['campaign']:x['configuration'] for x in lock['recommendations']};groups=defaultdict(list);frames=0;max_error=0.;test_started=False;raw_index=0
weights=np.array([m['task']['objective'][k]['weight']/m['task']['objective'][k]['scale']**2 for k in ['position','angle','force','deltaForce']])
keys=lambda c:json.dumps(c,sort_keys=True,separators=(',',':'))
with gzip.open(folder/'raw-evaluations.jsonl.gz','rt') as f:
 for line in f:
    q=json.loads(line);assert q['recordIndex']==raw_index;raw_index+=1
    if q['phase']=='test':test_started=True;assert q['configuration']==locked[q['campaign']]
    else:assert not test_started,'Test leaked before all optimization/validation locks'
    config=q['configuration'];assert ('horizon' in config)==(config['controller']=='hard_mpc');assert .5<=config['effort']<=2 and .1<=config['process']<=10
    if 'horizon' in config:assert config['horizon'] in [20,30,40]
    d=q['design'];assert d['Rc']==config['effort']/9;np.testing.assert_allclose(d['Qe'],np.array(m['theoryDesign']['QeBase'])*config['process'],rtol=0,atol=1e-16)
    np.testing.assert_array_equal(d['P0'],m['theoryDesign']['P0diag']);np.testing.assert_array_equal(d['Re'],r['fit']['Rdiag'])
    assert d['horizon']==config.get('horizon',30)
    assert q['information']['controller']=='observer-output only'
    a=np.asarray(q['auditSeries'],float).reshape(-1,11);frames+=len(a);assert len(a)==q['appliedSteps']
    if len(a):
        assert np.max(np.abs(a[:,2]))<=10+1e-8
        np.testing.assert_allclose(a[:,3],np.r_[a[0,2],np.diff(a[:,2])],rtol=0,atol=1e-12)
        score=float(np.sum(np.mean(a[:,:4]**2*weights,axis=0)));max_error=max(max_error,abs(score-q['partialScore']));assert max_error<1e-8
        if q['outcome']=='completed':assert len(a)==600 and abs(score-q['fullScore'])<1e-8
    if q['outcome']!='completed':assert q['fullScore'] is None and not q['taskPassed']
    if q['phase']=='test':
        template=int(q['caseId'].split('-')[1]);case=m['test'][template];assert q['seed'] in p['freshTestSeedRows'][template]
    else:case=next(c for c in m[q['phase']] if c['id']==q['caseId']);assert q['seed']==case['seed']
    if len(a):
        tail=a[-m['task']['finalWindowSteps']:]
        tailp=float(np.sqrt(np.mean((tail[:,8]-case['goal'])**2)));tailt=float(np.sqrt(np.mean(tail[:,9]**2)))
        expected=q['outcome']=='completed' and len(tail)==100 and tailp<=.15 and tailt<=.06
        assert bool(expected)==q['taskPassed']
    loss=16*(q['outcome']!='completed')+4*(not q['taskPassed'])+(q['fullScore']/512 if q['outcome']=='completed' else 0)
    assert abs(loss-q['loss'])<1e-12
    groups[q['campaign']].append({k:v for k,v in q.items() if k not in ['auditSeries','design']})
assert raw_index==r['actualEvaluations']

def certificate(rows,phase):
    gg=defaultdict(list)
    for q in rows:
        if q['phase']==phase:gg[keys(q['configuration'])].append(q)
    expected={t['id'] for t in m[phase]};out=[]
    for key,rr in gg.items():
        if len(rr)!=3 or {x['caseId'] for x in rr}!=expected:continue
        hard=sum(x['outcome']!='completed' for x in rr);bad=sum(not x['taskPassed'] for x in rr);score=float(np.mean([x['fullScore'] for x in rr])) if not hard else None
        out.append({'config':rr[0]['configuration'],'hard':hard,'task':bad,'score':score,'key':key})
    return out

def base(c):return c['effort']==c['process']==1 and c.get('horizon',30)==30

def rank(x):return (x['hard'],x['task'],x['score'] if x['score'] is not None else float('inf'),0 if base(x['config']) else 1,x['key'])

summaries=[]
for campaign in r['campaigns']:
    name=campaign['campaign'];allrows=groups[name];train=[x for x in allrows if x['phase']=='training'];val=[x for x in allrows if x['phase']=='validation'];test=[x for x in allrows if x['phase']=='test']
    assert len(train)==campaign['actualSearchRollouts']==(216 if campaign['method']=='grid_exhaustive' else 108)
    assert len({(keys(x['configuration']),x['caseId']) for x in train})==len(train)
    assert sum(x['appliedSteps'] for x in train)==campaign['actualSearchSteps']
    first=train[:12];assert all(base(x['configuration']) for x in first);assert len({(x['configuration']['controller'],x['configuration']['observer']) for x in first})==4
    cc=sorted(certificate(train,'training'),key=rank)
    assert len(cc)==campaign['fullyEvaluatedConfigurations']
    alloc=Counter(keys(x['configuration']) for x in train);assert dict(Counter(alloc.values()))=={int(k):v for k,v in campaign['allocationCounts'].items()}
    if campaign['method'] in ['smac_racing','random_racing']:
        assert campaign['engine']['intensifier']=='Intensifier';assert campaign['engine']['runhistoryFinished']==len(train)
        history=json.loads((folder/name/'runhistory.json').read_text());assert len(history['data'])==len(train)
    else:assert all(v==3 for v in alloc.values())
    for checkpoint in campaign['checkpoints']:
        cert=certificate(train[:checkpoint['rollouts']],'training');good=[x['score'] for x in cert if not x['hard'] and not x['task']]
        expected=min(good) if good else None
        assert expected is None and checkpoint['bestCertifiedScore'] is None or abs(expected-checkpoint['bestCertifiedScore'])<1e-10
    shortlist=[x['config'] for x in cc[:2]];default={'controller':'lqr','observer':'kf','effort':1.,'process':1.}
    if keys(default) not in {keys(c) for c in shortlist}:shortlist.append(default)
    assert shortlist==campaign['shortlist']
    assert len(val)==3*len(shortlist)==campaign['validationRollouts']
    vg=[x for x in certificate(val,'validation') if not x['hard'] and not x['task']]
    if vg:
        minimum=min(x['score'] for x in vg);near=[x for x in vg if x['score']<=minimum*1.01]
        def policy(x):
            c=x['config'];order=next(z['simplicityOrder'] for z in m['pairs'] if z['controller']==c['controller'] and z['observer']==c['observer'])
            return order,x['score'],0 if base(c) else 1,x['key']
        selected=min(near,key=policy)['config'];assert selected==campaign['selection']['configuration']==locked[name]
        assert len(test)==18
    else:assert not test and campaign['selection']['configuration'] is None
    assert sum(x['taskPassed'] for x in test if int(x['caseId'].split('-')[1])<4)==campaign['primaryTaskSuccesses']
    summaries.append({'campaign':name,'training':len(train),'validation':len(val),'test':len(test),'valid':True})
# Recompute paired seed-level effect; serial frames are never replicates.
for stat in r['pairedStatistics']:
    values=[]
    for seed in p['optimizerSeeds']:
        ca=[x for x in groups[f'{stat["method"]}-{seed}'] if x['phase']=='test' and int(x['caseId'].split('-')[1])<4]
        cb=[x for x in groups[f'grid_budget-{seed}'] if x['phase']=='test' and int(x['caseId'].split('-')[1])<4]
        if len(ca)==len(cb)==12 and all(x['outcome']=='completed' for x in ca+cb):values.append(float(np.mean([x['fullScore'] for x in ca])/np.mean([x['fullScore'] for x in cb])-1))
    assert len(values)==len(stat['pairs'])
    if values:assert abs(np.mean(values)-stat['meanRelativeTaskDifference'])<1e-12
    if len(values)==5:
        if np.ptp(values)==0:ci=[values[0],values[0]]
        else:
            b=bootstrap((np.array(values),),np.mean,n_resamples=10000,confidence_level=.95,method='percentile',rng=np.random.default_rng(19001));ci=[b.confidence_interval.low,b.confidence_interval.high]
        np.testing.assert_allclose(ci,stat['conditionalPairedBootstrap95'],rtol=0,atol=1e-12)
report={'schema':'cartpole-sequential-tuning-reference/v1','passed':True,'independentlyCheckedRollouts':raw_index,'independentlyScoredFrames':frames,'maximumTaskScoreError':max_error,'campaignBudgetsAndLocks':summaries,'conditionalBootstrapReconstructed':True,'evidenceSha256':hashlib.sha256((ROOT/'evidence/sequential_tuning.json').read_bytes()).hexdigest(),'rawLogSha256':hashlib.sha256((folder/'raw-evaluations.jsonl.gz').read_bytes()).hexdigest(),'scope':'Independent arithmetic/data-split/rank/budget validation. Statistical intervals are conditional pilot estimates, not proof of global superiority or deployment safety.'}
(ROOT/'evidence/sequential_tuning_reference.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k!='campaignBudgetsAndLocks'},indent=2))
