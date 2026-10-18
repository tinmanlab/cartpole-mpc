"""Actual command-rate comparison using the existing matched-contact inverse.

A2msfeedback candidate is not re-labelled as a10mshold controller. Exact state
is a diagnostic input; simulation time does not include computation latency.
"""
from pathlib import Path
from time import perf_counter_ns
import json
import numpy as np
import mujoco
import wheelbot_contact_wbc as w
ROOT=Path(__file__).resolve().parents[1]
PROTOCOL=ROOT/'tests/fixtures/wheelbot_contact_braking.json'

def plain(x):
    if isinstance(x,np.ndarray):return x.tolist()
    if isinstance(x,np.generic):return x.item()
    raise TypeError(type(x).__name__)

def summary_ns(values,deadline):
    x=np.array(values,dtype=float)
    return None if not len(x) else {'samples':len(x),'median_ms':float(np.median(x)/1e6),'p95_ms':float(np.quantile(x,.95)/1e6),'max_ms':float(x.max()/1e6),'deadline_misses':int(np.sum(x>deadline*1e9))}

def assess(protocol,initial,history,complete):
    sign=np.sign(initial[2]);n=len(history)
    if not n:return {'tracking_success':False,'completed':False,'physics_steps':0}
    rotations=np.array([sign*(r['after'][2]-initial[2])for r in history])
    velocities=np.array([r['after'][8]for r in history])
    tail=np.array([r['time_s']>protocol['duration_s']-protocol['final_window_s']+1e-12 for r in history])
    edge=max(r['edge_rise_m']for r in history);pen=max(r['penetration_m']for r in history);joint=max(r['joint_excursion_rad']for r in history)
    final_error=float(np.max(np.abs(rotations[tail]-protocol['reference_rotation_rad']))) if tail.any() else None
    final_speed=float(np.max(np.abs(velocities[tail]))) if tail.any() else None
    overshoot=max(0.,float(rotations.max()-protocol['reference_rotation_rad']))
    physical=pen<=protocol['maximum_penetration_m'] and joint<=protocol['maximum_joint_excursion_rad']
    passed=bool(complete and physical and edge>=protocol['minimum_hip_edge_rise_m'] and final_error is not None and final_error<=protocol['maximum_final_angle_error_rad'] and final_speed<=protocol['maximum_final_pitch_speed_rad_s'] and overshoot<=protocol['maximum_rotation_overshoot_rad'])
    return {'tracking_success':passed,'completed':bool(complete),'physics_steps':n,'max_edge_rise_m':edge,'max_rotation_rad':float(rotations.max()),'final_rotation_rad':float(rotations[-1]),'final_window_angle_error_rad':final_error,'final_window_pitch_speed_rad_s':final_speed,'final_pitch_velocity_rad_s':float(velocities[-1]),'rotation_overshoot_rad':overshoot,'max_penetration_m':pen,'max_joint_excursion_rad':joint,'physical_limits_met':bool(physical),'final_edge_rise_m':history[-1]['edge_rise_m']}

def execute(protocol,case,sign):
    m=w.model();assert m.opt.timestep==protocol['physics_dt_s'] and int(m.opt.enableflags)==int(m.opt.disableflags)==0
    initial=w.settle(m,sign);x=initial.copy();edge0=w.material_geometry(m,initial)['edge'][2]
    controller=w.ContactInverseController(m,reuse_workspace=case['reuse_workspace'],max_iterations=case['max_iterations'])
    nstep=round(case['feedback_dt_s']/m.opt.timestep);count=round(protocol['duration_s']/case['feedback_dt_s'])
    assert nstep in [1,5] and abs(nstep*m.opt.timestep-case['feedback_dt_s'])<1e-12
    data=mujoco.MjData(m);history=[];commands=[];rejected=None
    components={k:[]for k in ['optimization','forward_validation','physics','telemetry','full_control_cycle']}
    begin=perf_counter_ns()
    for k in range(count):
        started=perf_counter_ns();target=w.inverse_reference(initial,x,k*case['feedback_dt_s'])
        result=controller.solve(x,*target)
        if not result['accepted']:
            rejected={'command':k,'status':result['status'],'message':str(result.get('message','')),'applied':False};break
        previous=x.copy();tau=result['tau'].copy()
        assert not np.any(np.abs(tau)>np.array(w.INVERSE_PROTOCOL['torque_limits_nm'])+1e-8)
        mujoco.mj_resetData(m,data);data.qpos[:]=x[:6];data.qvel[:]=x[6:];data.ctrl[:]=tau
        phys=0;telemetry=0;unsafe=False
        for j in range(nstep):
            before=np.r_[data.qpos,data.qvel].copy();warm=data.qacc_warmstart.copy()
            t=perf_counter_ns();mujoco.mj_step(m,data);phys+=perf_counter_ns()-t;t=perf_counter_ns()
            x=np.r_[data.qpos,data.qvel].copy()
            penetration=max([0.]+[-float(c.dist)for c in data.contact])
            # Check resulting geometry as well as preintegration soft contacts.
            measure=mujoco.MjData(m);measure.qpos[:]=x[:6];measure.qvel[:]=x[6:];mujoco.mj_forward(m,measure)
            penetration=max(penetration,max([0.]+[-float(c.dist)for c in measure.contact]))
            excursion=max(0.,m.jnt_range[3,0]-x[3],x[3]-m.jnt_range[3,1],m.jnt_range[4,0]-x[4],x[4]-m.jnt_range[4,1])
            geometry=w.material_geometry(m,x)
            assert not data.qfrc_applied.any() and not data.xfrc_applied.any()
            history.append({'time_s':float(k*case['feedback_dt_s']+(j+1)*m.opt.timestep),'command':k,'substep':j,'before':before.tolist(),'after':x.tolist(),'tau':tau.tolist(),'qacc':data.qacc.tolist(),'warmstart_before':warm.tolist(),'warmstart_after':data.qacc_warmstart.tolist(),'forces':w.measured(m,data),'contacts':w.contact_pairs(m,data),'edge_rise_m':float(geometry['edge'][2]-edge0),'penetration_m':penetration,'joint_excursion_rad':float(excursion),'external_forces_zero':True})
            telemetry+=perf_counter_ns()-t
            if penetration>protocol['maximum_penetration_m'] or excursion>protocol['maximum_joint_excursion_rad']:unsafe=True;break
        complete_ns=perf_counter_ns()-started
        commands.append({'index':k,'before':previous.tolist(),'after':x.tolist(),'desired_acceleration':list(target),'optimized_acceleration':result['qacc'].tolist(),'tau':tau.tolist(),'status':result['status'],'accepted':bool(result['accepted']),'root_force_residual':result['root_required'],'inverse_forward_qacc_error':result['qacc_error'],'inverse_forward_force_error':result['force_error'],'iterations':result['iterations'],'full_cycle_ns':complete_ns})
        components['optimization'].append(result['timing_ns']['inverseopt_jacobians']);components['forward_validation'].append(result['timing_ns']['forwardvalidation']);components['physics'].append(phys);components['telemetry'].append(telemetry);components['full_control_cycle'].append(complete_ns)
        if unsafe:rejected={'command':k,'physical_failure_after_application':True};break
    elapsed=(perf_counter_ns()-begin)/1e9
    metrics=assess(protocol,initial,history,rejected is None and len(commands)==count)
    return {'name':case['name'],'sign':sign,'case':case,'initial_state':initial.tolist(),'expected_commands':count,'accepted_commands':len(commands),'rejection':rejected,'metrics':metrics,'timing':{key:summary_ns(value,case['feedback_dt_s'])for key,value in components.items()},'wall_seconds_excluding_settle':elapsed,'simulated_seconds':len(history)*m.opt.timestep,'real_time_factor':len(history)*m.opt.timestep/elapsed,'full_getup':False,'observer':False,'commands':commands,'history':history}

def main():
    protocol=json.loads(PROTOCOL.read_text());assert w.sha(w.XML)==protocol['model_sha256']
    assert protocol['reference_rotation_rad']==w.INVERSE_PROTOCOL['rotation_rad'] and protocol['reference_ramp_s']==w.INVERSE_PROTOCOL['ramp_s']
    trials=[]
    for case in protocol['cases']:
        for sign in protocol['signs']:
            result=execute(protocol,case,sign);trials.append(result)
            print(json.dumps({k:v for k,v in result.items()if k not in ['commands','history','initial_state']},default=plain),flush=True)
    raw=ROOT/'test-results/contact-braking-traces.json';raw.write_text(json.dumps(trials,default=plain,allow_nan=False)+'\n')
    sources=['scripts/wheelbot_contact_wbc.py','scripts/validate_wheelbot_contact_braking.py','tests/fixtures/wheelbot_contact_braking.json','tests/fixtures/wheelbot_contact_inverse.json','assets/wheelbot/contact_model.xml','assets/wheelbot/contact_profile.json']
    report={'schema':'wheelbot-contact-braking-evidence/v1','protocol':protocol,'hashes':{s:w.sha(ROOT/s)for s in sources},'trace_path':str(raw.relative_to(ROOT)),'trace_sha256':w.sha(raw),'native_version':mujoco.__version__,'source_state':'exact state for diagnostic feedback, not an implemented observer','controller':'Existing matched-contact inverse and unchanged reference gains. Explicit change from10mszero-orderhold to2msfeedback; optional scratch-buffer reuse and declared iteration budget.','physical_model_changed':False,'external_force_added':False,'trials':[{k:v for k,v in r.items()if k not in ['commands','history']}for r in trials],'hardware_ready':False,'hard_realtime_certified':False,'full_getup':False}
    (ROOT/'evidence/wheelbot_contact_braking.json').write_text(json.dumps(report,indent=2,default=plain,allow_nan=False)+'\n')

if __name__=='__main__':main()
