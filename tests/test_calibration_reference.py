"""Independent NumPy/statistics audit of data-only calibration and causal logs.
Does not fit using injection truth or alter the chosen R/controllers after test results.
"""
from pathlib import Path
import json,hashlib
import numpy as np
from scipy.stats import chi2
ROOT=Path(__file__).resolve().parents[1]
report=json.loads((ROOT/'evidence/calibration_lesson.json').read_text())
full=json.loads((ROOT/'test-results/calibration_lesson_full.json').read_text())
manifest=json.loads((ROOT/'tests/fixtures/calibration_lab.json').read_text())
y=np.asarray(full['capture']['measurements']);cov=np.cov(y,rowvar=False,ddof=1)
error=float(np.max(np.abs(cov-np.asarray(full['fit']['covariance']))));assert error<1e-12
np.testing.assert_allclose(np.mean(y,axis=0),full['fit']['mean'],atol=1e-12)
count=len(y);ci={}
for i,label in enumerate(['position_m2','angle_rad2']):
    ci[label]=[(count-1)*cov[i,i]/chi2.ppf(.975,count-1),(count-1)*cov[i,i]/chi2.ppf(.025,count-1)]
    yy=y[:,i]-y[:,i].mean();lag=float(yy[1:]@yy[:-1]/(yy@yy));assert abs(lag-full['fit']['lagOne'][i])<1e-12
maximum_nis_error=0.;maximum_force_identity_error=0.;checked=0
for case in full['cases']:
    stored=next(c for c in report['cases'] if c['id']==case['id']);K0=None
    for r in case['runs']:
        t=r['trace'];assert t;K=np.asarray(r['K']);K0=K if K0 is None else K0;np.testing.assert_array_equal(K,K0)
        assert r['Qe']==manifest['observer']['Q']
        X=np.array([q['truth'] for q in t]);E=np.array([q['estimate'] for q in t]);U=np.array([q['command'] for q in t])
        d=E-X;d[:,2]=np.arctan2(np.sin(d[:,2]),np.cos(d[:,2]));tracking=X[:,0]-np.array([q['goal'] for q in t])
        np.testing.assert_allclose(np.sqrt(np.mean(d*d,axis=0)),r['estimationRmseByState'],rtol=1e-10,atol=1e-12)
        assert abs(np.sqrt(np.mean(tracking**2))-r['positionTrackingRmse_m'])<1e-12
        saturated=0
        for q in t:
            ep=np.asarray(q['estimate']).copy();ep[0]-=q['goal'];requested=float(-K@ep);actual=float(np.clip(requested,-10,10))
            assert abs(actual-q['command'])<1e-12
            force_error=abs(requested-q['oracleForce']-sum(q['estimationForceContributions']));maximum_force_identity_error=max(maximum_force_identity_error,force_error)
            assert force_error<1e-9
            assert q['measurementTime']==q['commandTime'] and q['nextTime']>q['commandTime']
            saturated+=abs(requested)>10
            if q['S'] is not None:
                innovation=np.array(q['innovation']);score=float(innovation@np.linalg.solve(np.asarray(q['S']),innovation));maximum_nis_error=max(maximum_nis_error,abs(score-q['nis']));assert abs(score-q['nis'])<1e-7
            else:assert q['nis'] is None
        assert int(saturated)==r['saturatedSamples'];checked+=len(t)
        s=next(v for v in stored['runs'] if v['armId']==r['armId']);assert s['outcome']==r['outcome'] and s['positionTrackingRmse_m']==r['positionTrackingRmse_m']
    replay=stored['sameDataReplay'];matched=next(q for q in replay if q['armId']=='measured')
    assert matched['matchedArmMaximumDifference']<1e-10 and all(q['controllerApplied'] is False for q in replay)
assert all(c['seed']!=manifest['calibration']['seed'] for c in manifest['cases'])
out={'schema':'cartpole-calibration-reference/v1','passed':True,'measurementRows':count,'covarianceMaxAbsoluteError':error,'checkedDynamicFrames':checked,
 'maximumForceIdentityError_N':maximum_force_identity_error,'maximumNisDifference':maximum_nis_error,
 'varianceConfidenceIntervals95':ci,'intervalAssumption':'Independent Gaussian errors, stationary unknown constant mean; interval is not valid for arbitrary correlated/nonstationary data.',
 'sameDataReplayMatchedRecordedFilter':True,'scenarioOutcomesNotForcedToImprove':True,'referenceSourceSha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
 'recordedEvidenceSha256':hashlib.sha256((ROOT/'evidence/calibration_lesson.json').read_bytes()).hexdigest(),
 'scope':'Independent sample covariance/NIS/force decomposition and summary arithmetic. Existing EKF/MuJoCo/controller numerical authorities are reused; this is not proof of optimal Q_e or hardware calibration.'}
(ROOT/'evidence/calibration_reference.json').write_text(json.dumps(out,indent=2,allow_nan=False)+'\n');print(json.dumps(out,indent=2))
