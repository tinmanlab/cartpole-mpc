"""Offline native 2ms map + finite-horizon Riccati, following design_wheelbot_jump.py.
The online controller imports neither this module nor the inverse controller.
"""
from pathlib import Path
from time import perf_counter_ns, perf_counter
import hashlib
import json
import numpy as np
import mujoco
from scipy.linalg import solve
from wheelbot_contact_tracking import TrackingController, INDICES, selected_derivatives_validated
from validate_wheelbot_contact_braking import assess, summary_ns
ROOT=Path(__file__).resolve().parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
PROTOCOL=ROOT/'tests/fixtures/wheelbot_contact_tracking.json'
TRACE_DIGEST='c84199c0cb7624a0bf88a082d788560038286d8971915ac1fc5466d150eab3da'

def write(path,value):
    (ROOT/path).write_text(json.dumps(value,separators=(',',':'),allow_nan=False)+'\n')

def main():
    before=json.loads((ROOT/'test-results/contact-tracking-pre-admission-profile.json').read_text())
    before_design=json.loads((ROOT/'test-results/contact-tracking-pre-admission-design.json').read_text())
    started=perf_counter();protocol=json.loads(PROTOCOL.read_text())
    source=json.loads((ROOT/'evidence/wheelbot_contact_braking.json').read_text())
    for path,digest in source['hashes'].items():
        if sha(ROOT/path)!=digest:raise ValueError('Historical source mismatch: '+path)
    trace=ROOT/source['trace_path']
    if sha(trace)!=source['trace_sha256'] or sha(trace)!=TRACE_DIGEST:raise ValueError('Trace mismatch')
    cost_path=ROOT/'assets/wheelbot/contact_profile.json';cost=json.loads(cost_path.read_text())
    xml=ROOT/'assets/wheelbot/contact_model.xml'
    if sha(xml)!=protocol['model_sha256'] or sha(xml)!=cost['assetSha256']:raise ValueError('Model mismatch')
    m=mujoco.MjModel.from_xml_path(str(xml));d=mujoco.MjData(m);geo=mujoco.MjData(m)
    assert mujoco.__version__=='3.7.0' and m.opt.timestep==.002 and (m.nq,m.nv,m.nu)==(6,6,3)
    assert int(m.opt.enableflags)==int(m.opt.disableflags)==0
    assert cost['controlledIndices']==INDICES
    Q=np.array(cost['Q']);R=np.array(cost['R']);torso=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'torso')
    identities={'model_sha256':sha(xml),'cost_sha256':sha(cost_path),'trace_sha256':sha(trace),'protocol_sha256':sha(PROTOCOL)}
    profile={'schema':'wheelbot-contact-tracking/v1','identities':identities,'source_hashes':source['hashes'],'source_report_sha256':sha(ROOT/'evidence/wheelbot_contact_braking.json'),'controlledIndices':INDICES,'dt':.002,'steps':250,'limitsNm':[16,16,1.7],'Q':Q.tolist(),'R':R.tolist(),'terminal':'Q','protocol':protocol,'trajectories':{},'method':'Nonlinear reference from contact inverse + time-varying trajectory-error feedback; exact state diagnostic, no observer'}
    raw={'trajectories':{}};checks=[]
    def set_data(data,x,u=None):
        mujoco.mj_resetData(m,data);data.qpos[:]=x[:6];data.qvel[:]=x[6:]
        if u is not None:data.ctrl[:]=u
    def step(x,u):
        set_data(d,x,u);mujoco.mj_step(m,d)
        signature=(tuple((int(c.geom1),int(c.geom2)) for c in d.contact),tuple(int(v) for v in d.efc_state))
        return np.r_[d.qpos,d.qvel].copy(),signature
    rows=[r for r in json.loads(trace.read_text()) if r['name']=='substep_feedback_2ms']
    assert sorted(r['sign'] for r in rows)==[-1,1]
    for row in rows:
        sign=str(row['sign']);h=row['history'];assert len(h)==len(row['commands'])==row['accepted_commands']==250
        assert row['case']['feedback_dt_s']==.002 and row['metrics']['tracking_success']
        xs=np.array([row['initial_state']]+[v['after'] for v in h]);us=np.array([v['tau'] for v in h])
        Aall=[];Ball=[];max_replay=0.
        for k in range(250):
            assert np.array_equal(h[k]['before'],xs[k]) and not np.any(h[k]['warmstart_before']) and h[k]['external_forces_zero']
            assert np.array_equal(row['commands'][k]['tau'],us[k])
            assert abs(h[k]['time_s']-(k+1)*.002)<1e-12 and h[k]['command']==k and h[k]['substep']==0
            assert np.isfinite(xs[k]).all() and np.isfinite(us[k]).all() and np.all(np.abs(us[k])<=[16,16,1.7])
            y,base=step(xs[k],us[k]);max_replay=max(max_replay,float(np.max(np.abs(y-xs[k+1]))))
            A=np.zeros((11,11));B=np.zeros((11,3))
            for kind,count,out in [('x',11,A),('u',3,B)]:
                for j in range(count):
                    estimates=[];branches=[]
                    for eps in protocol['design']['epsilons']:
                        dx=np.zeros(12);du=np.zeros(3)
                        if kind=='x':dx[INDICES[j]]=eps
                        else:du[j]=eps
                        yp,sp=step(xs[k]+dx,us[k]+du);ym,sm=step(xs[k]-dx,us[k]-du)
                        estimates.append((yp-ym)[INDICES]/(2*eps));branches.append({'contacts':sp[0]==sm[0]==base[0],'constraints':sp[1]==sm[1]==base[1]})
                    error=float(np.max(np.abs(estimates[0]-estimates[1]))/max(1.,float(np.max(np.abs(estimates[1])))))
                    checks.append({'sign':int(sign),'step':k,'kind':kind,'column':j,'relative_refinement_error':error,'branches':branches})
                    out[:,j]=estimates[-1]
            if not np.isfinite(A).all() or not np.isfinite(B).all():raise ValueError('Nonfinite derivative')
            Aall.append(A);Ball.append(B)
        assert max_replay<1e-10
        P=Q.copy();Ks=[None]*250;conditions=[];residuals=[]
        for k in range(249,-1,-1):
            A,B=Aall[k],Ball[k];H=R+B.T@P@B;G=B.T@P@A
            K=solve(H,G,assume_a='pos');conditions.append(float(np.linalg.cond(H)));residuals.append(float(np.max(np.abs(H@K-G))))
            F=A-B@K;P=Q+K.T@R@K+F.T@P@F;P=(P+P.T)/2;Ks[k]=K.tolist()
        profile['trajectories'][sign]={'ref':xs.tolist(),'u':us.tolist(),'K':Ks}
        raw['trajectories'][sign]={'A':[a.tolist() for a in Aall],'B':[b.tolist() for b in Ball],'max_replay_error':max_replay,'max_solve_condition':max(conditions),'max_gain_residual':max(residuals)}
    diag={'derivative_columns':len(checks),'native_map_evaluations':500+len(checks)*4,'epsilons':protocol['design']['epsilons'],'contact_branch_mismatches':[sum(not c['branches'][i]['contacts'] for c in checks) for i in [0,1]],'constraint_branch_mismatches':[sum(not c['branches'][i]['constraints'] for c in checks) for i in [0,1]],'max_relative_refinement_error':max(c['relative_refinement_error'] for c in checks),'largest_abs_gain':max(float(np.max(np.abs(r['K']))) for r in profile['trajectories'].values()),'max_solve_condition':max(r['max_solve_condition'] for r in raw['trajectories'].values()),'poor_condition_threshold':1e10}
    diag['linearization_validated']=not diag['contact_branch_mismatches'][-1] and not diag['constraint_branch_mismatches'][-1] and diag['max_relative_refinement_error']<.1 and diag['max_solve_condition']<1e10
    # One bounded diagnostic refinement; original derivative gate and gains remain unchanged.
    refinement=[]
    for check in checks:
        if check['relative_refinement_error']<.1 and all(v['contacts'] and v['constraints'] for v in check['branches']):continue
        row=profile['trajectories'][str(check['sign'])];k=check['step'];x=np.array(row['ref'][k]);u=np.array(row['u'][k]);est=[];branches=[]
        _,base=step(x,u)
        for eps in [1e-7,1e-8]:
            dx=np.zeros(12);du=np.zeros(3)
            if check['kind']=='x':dx[INDICES[check['column']]]=eps
            else:du[check['column']]=eps
            yp,sp=step(x+dx,u+du);ym,sm=step(x-dx,u-du)
            est.append((yp-ym)[INDICES]/(2*eps));branches.append(sp==sm==base)
        refinement.append({'sign':check['sign'],'step':k,'kind':check['kind'],'column':check['column'],'same_branch':branches,'relative_error':float(np.max(np.abs(est[0]-est[1]))/max(1.,float(np.max(np.abs(est[1])))))})
    diag['bounded_refinement']={'epsilons':[1e-7,1e-8],'rows':refinement,'gains_changed':False,'original_gate_retained':True,'extra_map_evaluations':len(refinement)*5}
    diag['selected_derivatives_validated']=selected_derivatives_validated(checks,refinement,diag['max_solve_condition'])
    diag['selected_derivative_scope']='Finite local consistency for chosen nominal branch, not global differentiability or stability under contact changes.'
    diag['coarse_invalid_classification']=[dict(sign=c['sign'],step=c['step'],kind=c['kind'],column=c['column'],reason='contactbranch-crossing' if not all(b['contacts'] and b['constraints'] for b in c['branches']) else 'excessive-coarse-step') for c in checks if not all(b['contacts'] and b['constraints'] for b in c['branches']) or c['relative_refinement_error']>=.1]
    for key in ['Q','R','limitsNm','controlledIndices']:
        np.testing.assert_array_equal(profile[key],before[key])
    for sign in ['-1','1']:
        for key in ['ref','u','K']:np.testing.assert_array_equal(profile['trajectories'][sign][key],before['trajectories'][sign][key])
        for key in ['A','B']:np.testing.assert_array_equal(raw['trajectories'][sign][key],before_design['trajectories'][sign][key])
    old_protocol=json.loads((ROOT/'test-results/contact-tracking-pre-admission-protocol.json').read_text())
    for key,value in old_protocol.items():
        if key not in ['scope','changed_numerical_budget']:assert protocol[key]==value,key
    assert not diag['linearization_validated'] and diag['selected_derivatives_validated']
    diag['numeric_arrays_exactly_unchanged']=True
    assert abs(float(m.body_mass.sum())-cost['modelMetadata']['totalMassKg'])<1e-12
    np.testing.assert_array_equal(m.actuator_ctrlrange[:,1],[16,16,1.7])
    profile['branchDiagnostics']=diag;profile['generator_sha256']=sha(Path(__file__));profile['runtime_sha256']=sha(ROOT/'scripts/wheelbot_contact_tracking.py')
    log_start=perf_counter()
    write('assets/wheelbot/contact_tracking_profile.json',profile);write('test-results/contact-tracking-design.json',raw);write('test-results/contact-tracking-derivatives.json',checks)
    design_log_seconds=perf_counter()-log_start
    build_seconds=perf_counter()-started
    controller=TrackingController(profile,identities)
    def geometry(x):
        set_data(geo,x);mujoco.mj_forward(m,geo)
        edge=float((geo.xpos[torso]+geo.xmat[torso].reshape(3,3)@np.array([np.sign(x[2])*.0625,0,0]))[2])
        pen=max([0.]+[-float(c.dist) for c in geo.contact])
        excursion=float(max(0.,m.jnt_range[3,0]-x[3],x[3]-m.jnt_range[3,1],m.jnt_range[4,0]-x[4],x[4]-m.jnt_range[4,1]))
        return edge,pen,excursion
    timing=[];sample=profile['trajectories']['1']['ref'][0]
    for k in range(200):controller.command(sample,0,1)
    for k in range(2000):
        t=perf_counter_ns();controller.command(sample,0,1);timing.append(perf_counter_ns()-t)
    trials=[]
    for sign in [-1,1]:
        ref=np.array(profile['trajectories'][str(sign)]['ref'])
        for variant in protocol['variants']:
            for feedback in [False,True]:
                initial=ref[0].copy();initial[variant['index']]+=variant['offset'];x=initial.copy()
                edge0,pen0,joint0=geometry(x);initial_contacts=[(int(c.geom1),int(c.geom2)) for c in geo.contact];admissible=pen0<=protocol['maximum_penetration_m'] and joint0<=protocol['maximum_joint_excursion_rad']
                hist=[];timers={key:[] for key in ['command','physics','diagnostics','loop']};saturation=0;finite=True;torque_violation=0.;prediction=0.;prediction_components=np.zeros(11);start=perf_counter()
                for k in range(250 if admissible else 0):
                    begin=perf_counter_ns();t=perf_counter_ns();u=controller.command(x,k,sign,feedback=feedback);timers['command'].append(perf_counter_ns()-t)
                    t=perf_counter_ns();set_data(d,x,u);mujoco.mj_step(m,d);y=np.r_[d.qpos,d.qvel].copy();timers['physics'].append(perf_counter_ns()-t)
                    t=perf_counter_ns();edge,pen,joint=geometry(y);pen=max(pen,max([0.]+[-float(c.dist) for c in d.contact]))
                    assert not d.qfrc_applied.any() and not d.xfrc_applied.any()
                    finite=finite and bool(np.isfinite(y).all());torque_violation=max(torque_violation,float(np.max(np.abs(u)-controller.limits)),0.)
                    saturation+=int(np.any(np.abs(u)>=controller.limits))
                    design=raw['trajectories'][str(sign)];a=np.array(design['A'][k]);b=np.array(design['B'][k]);pred=ref[k+1,INDICES]+a@(x-ref[k])[INDICES]+b@(u-np.array(profile['trajectories'][str(sign)]['u'][k]));prediction=max(prediction,float(np.max(np.abs(y[INDICES]-pred))));prediction_components=np.maximum(prediction_components,np.abs(y[INDICES]-pred))
                    hist.append({'time_s':(k+1)*.002,'after':y.tolist(),'tau':u.tolist(),'edge_rise_m':edge-edge0,'penetration_m':pen,'joint_excursion_rad':joint,'external_forces_zero':True});x=y
                    timers['diagnostics'].append(perf_counter_ns()-t);timers['loop'].append(perf_counter_ns()-begin)
                    if not finite or pen>protocol['maximum_penetration_m'] or joint>protocol['maximum_joint_excursion_rad']:break
                elapsed=perf_counter()-start;metrics=assess(protocol,initial,hist,len(hist)==250)
                trials.append({'sign':sign,'variant':variant,'controller':'tvlqr' if feedback else 'feedforward','initial_state':initial.tolist(),'initial_contact_geom_pairs':initial_contacts,'initial_admissible':bool(admissible),'initial_penetration_m':pen0,'initial_joint_excursion_rad':joint0,'metrics':metrics,'finite':finite,'hard_input_violation_nm':torque_violation,'saturated_commands':saturation,'max_abs_torque_nm':np.max(np.abs([h['tau'] for h in hist]),axis=0).tolist() if hist else None,'max_state_prediction_error_by_controlled_index':prediction_components.tolist(),'max_state_prediction_error_inf':prediction,'max_reference_error_by_state':np.max(np.abs(np.array([h['after'] for h in hist])-ref[1:len(hist)+1]),axis=0).tolist() if hist else None,'timing':{k:summary_ns(v,.002) for k,v in timers.items()},'wall_seconds':elapsed,'simulated_seconds':len(hist)*.002,'simtime_over_wall':len(hist)*.002/elapsed,'history':hist,'timing_ns':timers})
    log_start=perf_counter();write('test-results/contact-tracking-trials.json',trials);trial_log_seconds=perf_counter()-log_start
    report={'schema':'wheelbot-contact-tracking-evidence/v1','identities':identities,'profile_sha256':sha(ROOT/'assets/wheelbot/contact_tracking_profile.json'),'profile_bytes':(ROOT/'assets/wheelbot/contact_tracking_profile.json').stat().st_size,'source_hashes':{p:sha(ROOT/p) for p in ['scripts/design_wheelbot_contact_tracking.py','scripts/wheelbot_contact_tracking.py','tests/test_wheelbot_contact_tracking.py','tests/fixtures/wheelbot_contact_tracking.json']},'native_version':mujoco.__version__,'offline_build_seconds':build_seconds,'artifact_logging_seconds':{'design_included_in_build':design_log_seconds,'trial_raw_write':trial_log_seconds,'excluded_from_control_loop':True,'final_receipt_write_measured':False},'derivatives':diag,'native_replay_max_error':max(r['max_replay_error'] for r in raw['trajectories'].values()),'max_gain_residual':max(r['max_gain_residual'] for r in raw['trajectories'].values()),'command_benchmark':{'warmup':200,**summary_ns(timing,.002)},'aggregate_timing':{mode:{key:summary_ns([v for row in trials if row['controller']==mode for v in row['timing_ns'][key]],.002) for key in ['command','physics','diagnostics','loop']} for mode in ['feedforward','tvlqr']},'trials':[{k:v for k,v in t.items() if k not in ['history','timing_ns']} for t in trials],'raw_hashes':{p:sha(ROOT/p) for p in ['test-results/contact-tracking-design.json','test-results/contact-tracking-derivatives.json','test-results/contact-tracking-trials.json']},'scope':'Locked deterministic assessment, unchanged Q/R, no tuning or statistical generalization. Exact simulation state; no KF, hardware, full getup, hard real-time or safety guarantee. Prior 9-10ms inverse full cycles included different logging and are not like-for-like.','linearization_workaround':None if diag['linearization_validated'] else 'Prototype only: the recorded 1e-7/1e-8 diagnostic is the single bounded refinement; original failed gate retained, no gain tuning or physics rewrite.'}
    write('evidence/wheelbot_contact_tracking.json',report)
    print(json.dumps({k:v for k,v in report.items() if k not in ['trials','source_hashes','raw_hashes']},indent=2))
    for t in trials:print(t['sign'],t['variant']['name'],t['controller'],t['metrics']['tracking_success'],t['metrics'].get('final_window_angle_error_rad'),t['metrics'].get('final_window_pitch_speed_rad_s'))

if __name__=='__main__':main()
