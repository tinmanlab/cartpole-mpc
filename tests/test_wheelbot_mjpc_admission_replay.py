"""Independent execution evidence: policy provenance, actual constraints and settling.

A successful standing trial is NEVER labelled a successful get-up. No fallback
selection is counted as an improvement produced by the upstream sampler.
"""
from pathlib import Path
import argparse,csv,hashlib,json,sys
import mujoco
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts/mjpc_wheelbot'))
import run

def verify(path):
    r=json.loads(path.read_text());assert r['returncode']==0
    trace=ROOT/r['trace'];trace_hash=r.get('trace_sha256')
    if trace_hash is None:
        # The first run predates inline trace digests; its existing run index
        # separately binds both receipt and trace. Do not fabricate a past hash.
        index=json.loads((ROOT/'evidence/wheelbot_mjpc_current.json').read_text())
        matches=[x for x in index['runs'] if x['receipt']==str(path.relative_to(ROOT)) and x['receipt_sha256']==hashlib.sha256(path.read_bytes()).hexdigest()]
        assert len(matches)==1,'Legacy receipt lacks an unambiguous indexed trace binding'
        trace_hash=matches[0]['trace_sha256']
    assert hashlib.sha256(trace.read_bytes()).hexdigest()==trace_hash
    p=json.loads((ROOT/'assets/wheelbot/contact_profile.json').read_text())
    assert hashlib.sha256((ROOT/'assets/wheelbot/contact_profile.json').read_bytes()).hexdigest()==r['profile_sha256']
    m=mujoco.MjModel.from_xml_path(str(ROOT/r['task_xml']));d=mujoco.MjData(m)
    assert run.assert_plant_equal(mujoco.MjModel.from_xml_path(str(ROOT/'assets/wheelbot/contact_model.xml')),m,r['numerics_profile'])==['disableflags: mjDSBL_WARMSTART']
    K=np.array(p['K']);qref=np.array(p['qref']);u0=np.array(p['uref']);limits=np.array(p['limitsNm']);rows=list(csv.DictReader(trace.open()))
    assert len(rows)==3001 and r['planner']['executed_steps']==3000
    types={};max_torque=np.zeros(3);max_joint=0.;max_pen=0.;action_error=0.;counted={'officialsample':0,'LQRbackup':0}
    for k,row in enumerate(rows):
        x=np.array([float(row[f'q{i}'])for i in range(6)]+[float(row[f'v{i}'])for i in range(6)])
        u=np.array([float(row[f'u{i}'])for i in range(3)])
        assert np.isfinite(x).all()and np.isfinite(u).all()and np.all(np.abs(u)<=limits+1e-12)
        assert float(row['nonlinear_horizon_feasibility_max'])<=1+1e-12
        source=row['selectedpolicy'];assert source in counted
        if k<3000:
            types[source]=types.get(source,0)+1
            if k%5==0:counted[source]+=1
            if source=='LQRbackup':
                expected=np.clip(u0-K@np.r_[x[:5]-qref[:5],x[6:]],-limits,limits)
                action_error=max(action_error,float(np.max(np.abs(expected-u))))
            assert int(row['rejectedcandidatecount'])+int(row['feasiblecandidates'])==r['planner']['rollouts_per_iteration']
        mujoco.mj_resetData(m,d);d.qpos[:]=x[:6];d.qvel[:]=x[6:];d.ctrl[:]=u;mujoco.mj_forward(m,d)
        assert np.max(np.abs(d.actuator_force-u))<1e-10
        pen=max([0.]+[-float(d.contact[j].dist)for j in range(d.ncon)])
        excursion=max(0.,m.jnt_range[3,0]-x[3],x[3]-m.jnt_range[3,1],m.jnt_range[4,0]-x[4],x[4]-m.jnt_range[4,1])
        assert abs(pen-float(row['penetration']))<1e-9 and abs(excursion-float(row['limit']))<1e-12
        max_pen=max(max_pen,pen);max_joint=max(max_joint,float(excursion));max_torque=np.maximum(max_torque,np.abs(u))
    assert action_error<1e-8 and max_pen<.02 and max_joint<.05
    assert counted['officialsample']==r['planner']['sample_selected'] and counted['LQRbackup']==r['planner']['backup_selected']
    evaluated=run.evaluate(trace,p);assert all(evaluated['gates'].values())
    actual_modes=('standing','perturbed','perturbed-left');assert r['mode'] in actual_modes,'This test verifies local standing, not fallen-state recovery'
    return {'receipt':str(path.relative_to(ROOT)),'receiptSha256':hashlib.sha256(path.read_bytes()).hexdigest(),'traceSha256':trace_hash,'initialPitchErrorRad':float(rows[0]['q2'])-qref[2],'executedSteps':3000,'localStandingPassed':True,'getUpFromFallenDemonstrated':False,'planningSelections':counted,'actualControlStepsByPolicy':types,'maxExistingLqrCommandErrorNm':action_error,'maximumTorquesNm':max_torque.tolist(),'maxJointExcursionRad':max_joint,'maxPenetrationM':max_pen,'metrics':evaluated['metrics'],'scope':'Observed local exact-state controller-plus-verified-backup behavior, not sampler-only success, sensor robustness, general safety or hardware certification.'}

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('receipts',nargs='+');parser.add_argument('--output',default='test-results/mjpc-admission/independent-admission.json');a=parser.parse_args()
    out={'schema':'wheelbot-mjpc-admission-replay/v1','passed':True,'scriptSha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'rows':[verify(ROOT/p)for p in a.receipts]}
    (ROOT/a.output).write_text(json.dumps(out,indent=2,allow_nan=False)+'\n');print(json.dumps(out,indent=2))
if __name__=='__main__':main()
