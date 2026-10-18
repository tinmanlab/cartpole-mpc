"""Independent replay of the actual contact-inverse diagnostic, not its controller.

Source identities, applied torques, native contact forces, state transitions and
physical metrics are reconstructed. Crossing a lift threshold is not settling.
"""
from pathlib import Path
import hashlib
import json
import numpy as np
import mujoco

ROOT=Path(__file__).resolve().parents[1]
sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
report_path=ROOT/'evidence/wheelbot_contact_inverse.json'
report=json.loads(report_path.read_text())
assert report['mujoco']==mujoco.__version__=='3.7.0'
for path,digest in report['hashes'].items():
    resolved=(ROOT/path).resolve()
    assert resolved.is_relative_to(ROOT) and sha(resolved)==digest,path
raw_path=ROOT/report['trace_path'];assert sha(raw_path)==report['trace_sha256']
raw=json.loads(raw_path.read_text());protocol=report['protocol']
m=mujoco.MjModel.from_xml_path(str(ROOT/'assets/wheelbot/contact_model.xml'))
assert (m.nq,m.nv,m.nu,m.neq)==(6,6,3,0)
assert m.opt.timestep==.002 and int(m.opt.disableflags)==int(m.opt.enableflags)==0
np.testing.assert_array_equal(m.actuator_trnid[:,0],[3,4,5])
np.testing.assert_array_equal(m.actuator_ctrlrange[:,1],[16,16,1.7])
body=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'torso')
geom=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'torso_visual')
kin=mujoco.MjData(m)

def edge(q,sign):
    kin.qpos[:]=q;mujoco.mj_kinematics(m,kin)
    local=np.array([sign*m.geom_size[geom,0],0,m.geom_pos[geom,2]-m.geom_size[geom,2]])
    return kin.xpos[body]+kin.xmat[body].reshape(3,3)@local

def forces(data):
    result={}
    for j in range(data.ncon):
        c=data.contact[j];f=np.zeros(6);mujoco.mj_contactForce(m,data,j,f)
        assert f[0]>=-1e-12
        names='/'.join(mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,int(g)) for g in [c.geom1,c.geom2])
        result[names]=result.get(names,np.zeros(3))+c.frame.reshape(3,3).T@f[:3]
    return result

rows=[];total_control=0;total_physics=0;max_state=0.;max_force=0.;max_root=0.;max_acceleration=0.
assert set(raw)=={t['name']for t in report['trials']} and len(raw)==6
for summary in report['trials']:
    name=summary['name'];trace=raw[name];initial=np.array(trace[0]['state']);sign=float(np.sign(initial[2]));edge0=edge(initial[:6],sign)[2]
    assert abs(edge0)<.002
    current=initial.copy();max_rise=0.;max_rootrise=0.;max_rotation=0.;max_pen=0.;max_joint=0.;crossed=False;peak=np.zeros(3);inverse_count=0
    for k,row in enumerate(trace):
        assert row['step']==k and 'replay'in row
        np.testing.assert_allclose(current,row['state'],atol=1e-12,rtol=0)
        result=row['result'];is_inverse=name.endswith('_inverse')
        tau=np.array(result['tau'] if is_inverse else result['y'][6:9])
        assert np.all(np.abs(tau)<=m.actuator_ctrlrange[:,1]+1e-8)
        peak=np.maximum(peak,np.abs(tau))
        if is_inverse:
            assert result['accepted'] and result['success'] and result['status']==0
            inv=mujoco.MjData(m);inv.qpos[:]=current[:6];inv.qvel[:]=current[6:];inv.qacc[:]=result['qacc'];mujoco.mj_inverse(m,inv)
            root=float(np.max(np.abs(inv.qfrc_inverse[:3])));max_root=max(max_root,root)
            assert root<=protocol['root_tolerance']
            np.testing.assert_allclose(inv.qfrc_inverse[3:],tau,atol=1e-8,rtol=0)
            fw=mujoco.MjData(m);fw.qpos[:]=current[:6];fw.qvel[:]=current[6:];fw.ctrl[:]=tau;mujoco.mj_forward(m,fw)
            error=float(np.max(np.abs(fw.qacc-np.array(result['qacc']))));max_acceleration=max(max_acceleration,error)
            assert error<=protocol['qacc_component_tolerance']
            first,second=forces(inv),forces(fw)
            for key in first.keys()|second.keys():
                force_error=float(np.max(np.abs(first.get(key,np.zeros(3))-second.get(key,np.zeros(3)))))
                max_force=max(max_force,force_error);assert force_error<=protocol['force_tolerance_n']
            inverse_count+=1
        data=mujoco.MjData(m);data.qpos[:]=current[:6];data.qvel[:]=current[6:];data.ctrl[:]=tau
        assert len(row['replay']['substeps'])==5
        for sub in row['replay']['substeps']:
            np.testing.assert_allclose(np.r_[data.qpos,data.qvel],sub['state'],atol=1e-10,rtol=0)
            assert not data.qfrc_applied.any() and not data.xfrc_applied.any()
            mujoco.mj_step(m,data);after=np.r_[data.qpos,data.qvel].copy()
            error=float(np.max(np.abs(after-np.asarray(sub['after']))));max_state=max(max_state,error);assert error<1e-8
            np.testing.assert_allclose(data.actuator_force,tau,atol=1e-10,rtol=0)
            for key,actual in forces(data).items():
                np.testing.assert_allclose(actual,sub['forces'][key],atol=1e-7,rtol=1e-9)
            penetration=max([0.]+[-float(data.contact[j].dist) for j in range(data.ncon)])
            assert abs(penetration-sub['penetration_m'])<1e-9
            max_pen=max(max_pen,penetration)
            excursion=max(0.,m.jnt_range[3,0]-after[3],after[3]-m.jnt_range[3,1],m.jnt_range[4,0]-after[4],after[4]-m.jnt_range[4,1])
            max_joint=max(max_joint,float(excursion))
            rise=float(edge(after[:6],sign)[2]-edge0);rotation=float(sign*(after[2]-initial[2]))
            max_rise=max(max_rise,rise);max_rootrise=max(max_rootrise,float(after[1]-initial[1]));max_rotation=max(max_rotation,rotation)
            crossed|=rise>=protocol['minimum_edge_rise_m'] and rotation>=protocol['minimum_away_rotation_rad']
            total_physics+=1
        current=np.r_[data.qpos,data.qvel].copy();np.testing.assert_allclose(current,row['replay']['state'],atol=1e-8,rtol=0);total_control+=1
    assert len(trace)==summary['completed_commands']==50
    np.testing.assert_allclose([max_rise,max_rootrise,max_rotation,max_joint,max_pen],[summary['max_edge_rise_m'],summary['max_root_rise_m'],summary['max_away_rotation_rad'],summary['max_joint_excursion_rad'],summary['max_penetration_m']],atol=1e-8,rtol=0)
    admissible=max_joint<=protocol['maximum_joint_excursion_rad'] and max_pen<=protocol['maximum_penetration_m']
    assert summary['whole_trial_admissible']==admissible and summary['simultaneous_thresholds_reached']==crossed
    rows.append({'name':name,'steps':len(trace),'contact_inverse_verified_commands':inverse_count,'lift_threshold_crossed':bool(crossed),'physical_limits_met':bool(admissible),'max_hip_edge_rise_m':max_rise,'max_root_rise_m':max_rootrise,'max_away_rotation_rad':max_rotation,'final_rotation_rad':float(sign*(current[2]-initial[2])),'final_rotation_reference_error_rad':float(sign*(current[2]-initial[2])-protocol['rotation_rad']) if is_inverse else None,'final_pitch_velocity_rad_s':float(current[8]),'peak_torque_nm':peak.tolist(),'bounded_target_tracking_established':False,'full_getup_established':False})
assert total_control==300 and total_physics==1500
result={'schema':'contact-inverse-independent-replay/v1','passed_arithmetic_and_physics_replay':True,'source_report_sha256':sha(report_path),'source_trace_sha256':sha(raw_path),'verifier_sha256':sha(Path(__file__)),'model_sha256':sha(ROOT/'assets/wheelbot/contact_model.xml'),'control_steps':total_control,'physics_steps':total_physics,'max_native_state_component_error':max_state,'max_unactuated_required_force':max_root,'max_inverse_forward_qacc_component_error':max_acceleration,'max_inverse_forward_force_error_n':max_force,'rows':rows,'scope':'The new inverse produces realizable motor/contact forces and visible hip-end lift, but overshoots the0.08radreference. Threshold crossing is not successful settling, fullgetup, measured-state control or hardware validation.'}
(ROOT/'evidence/wheelbot_contact_inverse_reference.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
print(json.dumps(result,indent=2,allow_nan=False))
