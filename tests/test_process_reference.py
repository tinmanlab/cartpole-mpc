"""Independent likelihood, split/selection and test-outcome arithmetic; no performance-forcing gate."""
from pathlib import Path
import json,hashlib
import numpy as np
from scipy.stats import multivariate_normal
ROOT=Path(__file__).resolve().parents[1]
full=json.loads((ROOT/'test-results/process_selection_full.json').read_text())
r=json.loads((ROOT/'evidence/process_selection.json').read_text())
study=json.loads((ROOT/'tests/fixtures/process_selection.json').read_text())
m=json.loads((ROOT/'tests/fixtures/calibration_lab.json').read_text())
assert r['experimentValid'] and r['selectionLockedBeforeTest']
assert r['manifestSha256']==hashlib.sha256((ROOT/'tests/fixtures/process_selection.json').read_bytes()).hexdigest()
assert not full['selection']['testUsedForSelection']
expected_fields={'id','seed','controlDt','measurements','commands','information'}
for record in full['trainingRecords']+full['validationRecords']:
    assert set(record)==expected_fields
    assert len(record['measurements'])==len(record['commands'])+1==study['steps']+1
seeds=[c['seed'] for k in ['training','validation','test'] for c in study[k]]+[m['calibration']['seed']]
assert len(set(seeds))==len(seeds)
max_nll_error=0.;count=0;phase_stats={}
for phase in ['training','validation']:
    for scale in study['scales']:
        evaluations=[e for e in full['selection']['evaluations'] if e['phase']==phase and e['scale']==scale]
        if not evaluations:continue
        values=[]
        assert [e['recordId'] for e in evaluations]==[c['id'] for c in study[phase]]
        for e in evaluations:
            assert e['samples']==study['steps']-study['burnInSteps']==len(e['rows'])
            vv=[]
            for row in e['rows']:
                nu=np.asarray(row['innovation']);S=np.asarray(row['S']);sign,logdet=np.linalg.slogdet(S)
                assert sign==1 and np.linalg.eigvalsh(S).min()>0
                value=float(-multivariate_normal.logpdf(nu,mean=np.zeros(2),cov=S))
                max_nll_error=max(max_nll_error,abs(value-row['nll']))
                assert abs(value-.5*(nu@np.linalg.solve(S,nu)+logdet+2*np.log(2*np.pi)))<1e-8
                vv.append(value);count+=1
            assert abs(np.mean(vv)-e['meanNll'])<1e-8
            values.extend(vv)
        phase_stats[(phase,scale)]=float(np.mean(values))
        stored=next(q for q in full['selection'][phase] if q['scale']==scale)
        assert abs(stored['meanNll']-np.mean(values))<1e-8
assert max_nll_error<1e-8
rank=lambda pair:(pair[1],-1 if pair[0]==1 else 0,pair[0])
train_rank=sorted([(s,phase_stats[('training',s)]) for s in study['scales']],key=rank)
shortlist=[q[0] for q in train_rank[:study['trainingShortlist']]]
if study['includeBaselineInValidation'] and 1 not in shortlist:shortlist.append(1)
assert shortlist==full['selection']['shortlist']
winner=sorted([(s,phase_stats[('validation',s)]) for s in shortlist],key=rank)[0][0]
assert winner==full['selection']['scale']
assert full['selection']['atGridBoundary']==(winner in [min(study['scales']),max(study['scales'])])
frames=0
for pair in full['pairs']:
    test=next(c for c in study['test'] if c['id']==pair['id'])
    assert pair['baseline']['seed']==pair['candidate']['seed']==test['seed']
    np.testing.assert_array_equal(pair['baseline']['K'],pair['candidate']['K'])
    np.testing.assert_array_equal(pair['baseline']['Rdiag'],pair['candidate']['Rdiag'])
    np.testing.assert_array_equal(pair['baseline']['trace'][0]['P'],pair['candidate']['trace'][0]['P'])
    np.testing.assert_allclose(pair['candidate']['Qe'],np.array(m['observer']['Q'])*winner,rtol=0,atol=1e-20)
    for key in ['baseline','candidate']:
        run=pair[key];trace=run['trace'];frames+=len(trace)
        assert run['appliedSteps']==len(trace)
        x=np.array([q['truth'] for q in trace]);xh=np.array([q['estimate'] for q in trace]);err=xh-x;err[:,2]=np.arctan2(np.sin(err[:,2]),np.cos(err[:,2]))
        np.testing.assert_allclose(np.sqrt(np.mean(err**2,axis=0)),run['estimationRmseByState'],atol=1e-10)
        assert abs(np.sqrt(np.mean((x[:,0]-test['goal'])**2))-run['positionTrackingRmse_m'])<1e-10
        assert sum(abs(q['requestedForce'])>m['controller']['forceLimit'] for q in trace)==run['saturatedSamples']
        expected_task=run['outcome']=='completed' and len(trace)>=m['evaluation']['goalWindowSteps'] and all(abs(q['nextTruth'][0]-test['goal'])<=m['evaluation']['positionTolerance_m'] and abs(q['nextTruth'][2])<=m['evaluation']['angleTolerance_rad'] for q in trace[-m['evaluation']['goalWindowSteps']:])
        assert bool(expected_task)==run['taskPassed']
    assert pair['sameDataReplay'][0]['matchedArmMaximumDifference']<1e-10
for group in full['groups']:
    pp=[p for p in full['pairs'] if p['group']==group['group']];both=[p for p in pp if p['paired']['bothCompleted']]
    assert len(both)==group['bothCompletedPairs']
    for key in ['baseline','candidate']:
        saved=next(a for a in group['arms'] if a['arm']==key)
        assert saved['taskPassed']==sum(p[key]['taskPassed'] for p in pp)
        if both:
            mse=np.mean([p[key]['positionTrackingRmse_m']**2 for p in both]);assert abs(np.sqrt(mse)-saved['bothCompletedTrackingRmse_m'])<1e-10
out={'schema':'cartpole-process-reference/v1','passed':True,'scipyLogpdfSamples':count,'maximumNllError':max_nll_error,'testFramesChecked':frames,'selectedScale':winner,'selectionUsesTest':False,'independentRankReconstructed':True,'scoreIncludesLogdet':True,'frozenBeforeTest':r['selectionLockedBeforeTest'],'recordedEvidenceSha256':hashlib.sha256((ROOT/'evidence/process_selection.json').read_bytes()).hexdigest(),'sourceSha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'scope':'Independent likelihood/ranking/data-split and outcome arithmetic. No global optimum, physical process-noise identification, nonlinear posterior or hardware claim.'}
(ROOT/'evidence/process_reference.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
