"""Verify the historical benchmark at its recorded source, not a fresh rerun.
F1 changes finalization only. Numeric/search definitions must remain AST-identical;
all other benchmark inputs must still match current files. No evidence is rehashed
to imply the old 2,943 runs were performed on new code.
"""
from pathlib import Path
import ast,json,hashlib,subprocess
ROOT=Path(__file__).resolve().parents[1]
RECORDED_COMMIT='15df4437667e3497f13ec28345ddc722dcadcfc4'
FINALIZATION_PATHS={'scripts/sequential_tuning.py','scripts/run_tuning_campaign.py','scripts/tuning_bridge.mjs'}
r=json.loads((ROOT/'evidence/sequential_tuning.json').read_text());p=json.loads((ROOT/'tests/fixtures/sequential_tuning.json').read_text());v=json.loads((ROOT/'evidence/sequential_tuning_reference.json').read_text())
sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
assert r['experimentValid'] and r['sourcesStable'] and r['mode']=='frozen-full'
assert r['protocolSha256']==sha(ROOT/'tests/fixtures/sequential_tuning.json')
for path,digest in r['sourceSha256'].items():
    if sha(ROOT/path)==digest:continue
    assert path in FINALIZATION_PATHS,'Unreviewed change to historical benchmark input: '+path
    original=subprocess.check_output(['git','show',RECORDED_COMMIT+':'+path],cwd=ROOT)
    assert hashlib.sha256(original).hexdigest()==digest,'Wrong historical source: '+path
    if path=='scripts/sequential_tuning.py':
        def definitions(text):
            return {n.name:ast.dump(n,include_attributes=False) for n in ast.parse(text).body if isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name!='main'}
        assert definitions(original)==definitions((ROOT/path).read_text()),'Search/scoring/selection changed, not only finalization'
# Recorded outcomes are immutable; CI does not rewrite this result for the fix.
assert (ROOT/'evidence/sequential_tuning.json').read_bytes()==subprocess.check_output(['git','show',RECORDED_COMMIT+':evidence/sequential_tuning.json'],cwd=ROOT)
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
print('Historical tuning receipt/budget/search-identity contracts PASS; F1 current execution is checked separately.')
