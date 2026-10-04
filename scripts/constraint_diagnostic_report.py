"""Render only a validated, source-bound CartPole diagnostic receipt."""
from pathlib import Path
import hashlib
import json

ROOT = Path(__file__).resolve().parents[1]

def digest(receipt):
    return hashlib.sha256(json.dumps({k:v for k,v in receipt.items() if k != 'receiptSha256'}, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()

def render(receipt):
    assert receipt['receiptSha256'] == digest(receipt), 'Receipt content changed'
    assert receipt['numericalAndInformationChecksPassed'] is True
    sources=receipt['sourceSha256']
    required={'scripts/run_constraint_diagnostic.mjs','scripts/constraint_diagnostic_report.py','tests/test_constraint_diagnostic.py','tests/osqp_condensed_reference.py','tests/fixtures/constraint_diagnostic.json','tests/fixtures/design_study.json','src/design_study.js','src/calibration_lab.js','src/engine.js','src/qp.js','src/plant.js','src/mujoco_backend.mjs','assets/cartpole.xml'}
    assert set(sources)==required, 'Incomplete source binding'
    for path, expected in sources.items():
        assert hashlib.sha256((ROOT/path).read_bytes()).hexdigest()==expected, 'Stale source: '+path
    assert receipt['protocolSha256']==sources['tests/fixtures/constraint_diagnostic.json']
    runs=receipt['runs'];probes=receipt['probes']
    counts={'cases':len(probes),'sensorBasedRuns':len(runs),'completed':sum(r['outcome']=='completed' for r in runs),'taskPassed':sum(r['taskPassed'] for r in runs),'exactStatePlansAccepted':sum(p['mpcAccepted'] for p in probes),'exactStatePlansRailActive':sum(p['mpcAccepted'] and p['mpcRailActive'] for p in probes),'sensorMpcRailActiveSamples':sum(r['predictedRailActiveSamples'] for r in runs if r['pairId'].startswith('mpc'))}
    assert counts==receipt['counts'], 'Outcome counts disagree'
    lines=['# Constraint diagnostic receipt','',f"Receipt SHA256: `{receipt['receiptSha256']}`",'',f"Protocol SHA256: `{receipt['protocolSha256']}`",'',f"Physical results: {counts['taskPassed']}/{counts['sensorBasedRuns']} task passes; {counts['completed']} completed; {counts['sensorMpcRailActiveSamples']} MPC rail-active samples.",'','| Case | Pair | Outcome | Applied steps | Task passed |','|---|---|---|---:|---|']
    lines += [f"| {r['caseId']} | {r['pairId']} | {r['outcome']} | {r['steps']} | {r['taskPassed']} |" for r in runs]
    lines += ['','Offline initialization ablation; each row compares the same nominal QP at one physical time. Truth and its plans are evaluator-only. Step-zero initial/rejected duplicates are combined.','','| Case | Pair | Step | Continuous (native) | Cold reset (native) | Truth (native) |','|---|---|---:|---|---|---|']
    lines += [f"| {s['caseId']} | {s['pairId']} | {s['appliedSteps']} | "+' | '.join(s['checks'][k]['nativeCategory'] for k in ['continuous','cold','truth'])+' |' for s in receipt['initializationChecks']]
    lines += ['','Solver-declared infeasibility is a solver status, not a mathematical certificate or proof of plant impossibility. Numerical nonacceptance is separate. Jointly accepted JS/native plans are checked for first action and original cost. No measured successful warm-up is claimed.','', 'Position intervals in the receipt are mean +/- 2 sqrt(Ppp) under assumed covariance, with same-time replay covariance identity and evaluator-only error. They are not calibrated risk, robust margins, safety permission or simultaneous coverage. Zero-transition residuals are null with zero samples. No online use or default changes.','','Source SHA256 bindings:','']
    lines += [f'- `{p}`: `{h}`' for p,h in sorted(sources.items())]
    return '\n'.join(lines)+'\n'

if __name__=='__main__':
    receipt=json.loads((ROOT/'evidence/constraint_diagnostic_summary.json').read_text())
    (ROOT/'evidence/constraint_diagnostic_report.md').write_text(render(receipt))
