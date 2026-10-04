"""Fast committed-receipt/source contract, NOT a rerun of native SMAC benchmarks."""
from pathlib import Path
import json,hashlib
ROOT=Path(__file__).resolve().parents[1]
r=json.loads((ROOT/'evidence/sequential_tuning.json').read_text());p=json.loads((ROOT/'tests/fixtures/sequential_tuning.json').read_text());v=json.loads((ROOT/'evidence/sequential_tuning_reference.json').read_text())
sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
assert r['experimentValid'] and r['sourcesStable'] and r['mode']=='frozen-full'
assert r['protocolSha256']==sha(ROOT/'tests/fixtures/sequential_tuning.json')
assert all(sha(ROOT/f)==h for f,h in r['sourceSha256'].items())
assert r['versions']['smac']==p['smac']['version'] and r['versions']['ConfigSpace']==p['smac']['configspaceVersion']
assert v['passed'] and v['evidenceSha256']==sha(ROOT/'evidence/sequential_tuning.json')
assert len(r['campaigns'])==21
actual=0;seen=set()
for c in r['campaigns']:
    assert c['campaign'] not in seen;seen.add(c['campaign'])
    budget=216 if c['method']=='grid_exhaustive' else 108
    assert c['actualSearchRollouts']==c['searchBudget']==budget
    assert sum(int(k)*count for k,count in c['allocationCounts'].items())==budget
    assert c['fullyEvaluatedConfigurations']==c['allocationCounts'].get('3',0)
    assert c['duplicateAsks']==0 and c['lockedBeforeTest']
    assert c['searchWallSeconds']>=c['searchEvaluationSeconds'] and c['searchWallSeconds']>=c['tunerSeconds']
    cfg=c['selection']['configuration'];n=18 if cfg else 0
    actual+=budget+c['validationRollouts']+n
    if cfg:assert ('horizon' in cfg)==(cfg['controller']=='hard_mpc')
    if c['method']=='smac_racing':
        assert c['engine']['model']=='RandomForest' and c['engine']['intensifier']=='Intensifier'
        assert any(('local' in key.lower() or 'sorted' in key.lower()) and count>0 for key,count in c['engine']['originCounts'].items()),'No model-based proposal actually exercised'
assert actual==r['actualEvaluations']==v['independentlyCheckedRollouts']
assert r['freshTestLock']['testEvaluations']==0 and len(r['freshTestLock']['recommendations'])==21
assert r['defaultPromoted'] is False and r['hardware']=='NOT_EVALUATED'
print('Stored tuning receipt/source/budget/conditional-space contracts PASS; actual optimizer execution is documented in the offline reference.')
