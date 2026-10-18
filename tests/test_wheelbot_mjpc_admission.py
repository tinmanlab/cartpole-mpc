"""Admission regression checks and independent 2ms full-contact LQR positive control."""
import csv, json, subprocess, sys
from pathlib import Path
import mujoco
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts/mjpc_wheelbot'))
import run
OUT=ROOT/'test-results/mjpc-admission'; OUT.mkdir(parents=True,exist_ok=True)
run.RUN_OUT=OUT
path,profile,_=run.model('safe_compact')
m=mujoco.MjModel.from_xml_path(str(path)); K=np.array(profile['K']); q=np.array(profile['qref'])
results=[]
for sign in (1,-1):
    d=mujoco.MjData(m); d.qpos[:]=q; d.qpos[2]+=sign*.02; mujoco.mj_forward(m,d)
    rows=[]
    for i in range(3001):
        d.ctrl[:]=np.clip(np.array(profile['uref'])-K@np.r_[d.qpos[:5]-q[:5],d.qvel],-np.array(profile['limitsNm']),profile['limitsNm'])
        check=mujoco.MjData(m); mujoco.mj_copyData(check,m,d); mujoco.mj_forward(m,check)
        row={'time':d.time,**{f'q{j}':d.qpos[j] for j in range(6)},**{f'v{j}':d.qvel[j] for j in range(6)},**{f'u{j}':d.ctrl[j] for j in range(3)},**{f'force{j}':check.actuator_force[j] for j in range(3)},'contacts':check.ncon,'penetration':max([0]+[-c.dist for c in check.contact]),'limit':max([0]+[max(m.jnt_range[j,0]-d.qpos[j],d.qpos[j]-m.jnt_range[j,1]) for j in (3,4)]),'cost':0,'iteration':i}
        rows.append(row)
        if i<3000: mujoco.mj_step(m,d)
    trace=OUT/f'lqr-baseline-{sign}.csv'
    with trace.open('w') as f:
        w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
    result=run.evaluate(trace,profile);assert result['getup_success'],result
    results.append({'sign':sign,'trace':str(trace.relative_to(ROOT)),**result})
(OUT/'baseline.json').write_text(json.dumps(results,indent=2)+'\n')
result=subprocess.run([str(ROOT/'test-results/mjpc-adopt/wheelbot'),str(path),str(OUT/'selfcheck.csv'),'check-admission','.002','0','reproducible'],capture_output=True,text=True,timeout=30)
assert result.returncode==0,(result.returncode,result.stderr)
assert json.loads(result.stdout)['late_unsafe_rejected']
print('PASS: independent ±.02rad LQR 6s; native late unsafe candidate rejected despite finite first torque')
# When supplied, validate actual run receipts in addition to the independent baseline.
for name in sys.argv[1:]:
    receipt=json.loads(Path(name).read_text()); planner=receipt['planner']
    with (ROOT/receipt['trace']).open() as f: rows=list(csv.DictReader(f))
    if receipt['mode'] in ('perturbed','perturbed-left'):
        assert receipt['returncode']==0 and all(receipt['gates'].values())
        assert planner['verified_plans']==600 and planner['executed_steps']==3000
        assert planner['consistency_pass'] and planner['rejected_plans']==0
        assert planner['sample_selected']+planner['backup_selected']==600
        assert all(r['selectedpolicy'] in ('officialsample','LQRbackup') for r in rows)
        assert all(float(r['nonlinear_horizon_feasibility_max'])<=1 for r in rows)
    elif receipt['mode']=='right':
        assert receipt['returncode']==2 and planner['rejection']=='NO_ADMISSIBLE_PLAN'
        assert planner['executed_steps']==0 and len(rows)==1
        assert rows[0]['selectedpolicy']=='none' and not receipt['getup_success']
print('PASS: supplied local/full-horizon and no-admissible-plan receipts')
