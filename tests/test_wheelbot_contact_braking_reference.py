"""Independent native replay and tracking acceptance for explicit rate variants.
No controller code is used to reconstruct dynamics or final acceptance.
"""
from pathlib import Path
import hashlib,json
import numpy as np
import mujoco
ROOT=Path(__file__).resolve().parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
receipt=ROOT/'evidence/wheelbot_contact_braking.json'
r=json.loads(receipt.read_text());p=r['protocol'];raw=ROOT/r['trace_path']
assert sha(raw)==r['trace_sha256'] and r['native_version']==mujoco.__version__=='3.7.0'
for name,digest in r['hashes'].items():
 path=(ROOT/name).resolve();assert path.is_relative_to(ROOT) and sha(path)==digest,name
trials=json.loads(raw.read_text());assert len(trials)==4
m=mujoco.MjModel.from_xml_path(str(ROOT/'assets/wheelbot/contact_model.xml'))
assert m.opt.timestep==.002 and int(m.opt.enableflags)==int(m.opt.disableflags)==0
assert (m.nq,m.nv,m.nu,m.neq)==(6,6,3,0)
np.testing.assert_array_equal(m.actuator_ctrlrange[:,1],[16,16,1.7]);np.testing.assert_array_equal(m.actuator_trnid[:,0],[3,4,5])
body=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'torso');geom=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'torso_visual');view=mujoco.MjData(m)

def edge(q,sign):
 view.qpos[:]=q;mujoco.mj_kinematics(m,view)
 point=np.array([sign*m.geom_size[geom,0],0,m.geom_pos[geom,2]-m.geom_size[geom,2]])
 return view.xpos[body]+view.xmat[body].reshape(3,3)@point

max_state=0.;max_force=0.;count=0;commands=0;rows=[]
for t in trials:
 case=t['case'];dt=case['feedback_dt_s'];substeps=round(dt/.002);initial=np.array(t['initial_state']);sign=t['sign'];start_edge=edge(initial[:6],sign)[2]
 d=mujoco.MjData(m);physics=t['history'];bycmd={c['index']:c for c in t['commands']};previous=initial.copy();rotations=[];rates=[];rises=[];penetrations=[];excursions=[];times=[];peak=np.zeros(3)
 for k,z in enumerate(physics):
  command=bycmd[z['command']];tau=np.array(z['tau']);assert command['status']==0 and command['accepted']
  assert np.all(np.abs(tau)<=m.actuator_ctrlrange[:,1]+1e-8)
  if z['substep']==0:
   mujoco.mj_resetData(m,d);d.qpos[:]=previous[:6];d.qvel[:]=previous[6:];d.ctrl[:]=tau
   np.testing.assert_allclose(previous,command['before'],atol=1e-10,rtol=0)
   assert command['root_force_residual']<=1e-7 and max(command['inverse_forward_qacc_error'])<=1e-6 and command['inverse_forward_force_error']<=1e-6
   inv=mujoco.MjData(m);inv.qpos[:]=previous[:6];inv.qvel[:]=previous[6:];inv.qacc[:]=command['optimized_acceleration'];mujoco.mj_inverse(m,inv)
   assert max(abs(inv.qfrc_inverse[:3]))<=1e-7
   np.testing.assert_allclose(inv.qfrc_inverse[3:],tau,atol=1e-8,rtol=0)
   commands+=1
  np.testing.assert_allclose(np.r_[d.qpos,d.qvel],z['before'],atol=1e-10,rtol=0)
  np.testing.assert_allclose(d.qacc_warmstart,z['warmstart_before'],atol=1e-8,rtol=0)
  assert not d.qfrc_applied.any() and not d.xfrc_applied.any()
  mujoco.mj_step(m,d);after=np.r_[d.qpos,d.qvel].copy()
  state_error=float(np.max(np.abs(after-z['after'])));max_state=max(max_state,state_error);assert state_error<1e-8
  np.testing.assert_allclose(d.actuator_force,tau,atol=1e-10,rtol=0)
  assert len(z['contacts'])==d.ncon
  for i,c in enumerate(d.contact):
   f=np.zeros(6);mujoco.mj_contactForce(m,d,i,f);saved=z['contacts'][i]
   assert [int(c.geom1),int(c.geom2)]==saved['geoms'] and f[0]>=-1e-12
   force_error=float(np.max(np.abs(f-saved['local_force'])));max_force=max(max_force,force_error);assert force_error<1e-7
  pen=max([0.]+[-float(c.dist)for c in d.contact]);view.qpos[:]=after[:6];view.qvel[:]=after[6:];mujoco.mj_forward(m,view)
  pen=max(pen,max([0.]+[-float(c.dist)for c in view.contact]))
  exc=max(0.,m.jnt_range[3,0]-after[3],after[3]-m.jnt_range[3,1],m.jnt_range[4,0]-after[4],after[4]-m.jnt_range[4,1])
  rise=float(edge(after[:6],sign)[2]-start_edge)
  np.testing.assert_allclose([pen,exc,rise],[z['penetration_m'],z['joint_excursion_rad'],z['edge_rise_m']],atol=1e-9,rtol=0)
  assert abs(z['time_s']-(k+1)*.002)<1e-12
  rotations.append(float(sign*(after[2]-initial[2])));rates.append(float(after[8]));rises.append(rise);penetrations.append(pen);excursions.append(exc);times.append(z['time_s']);previous=after;peak=np.maximum(peak,np.abs(tau));count+=1
 complete=len(physics)==250 and t['rejection'] is None and len(bycmd)==round(.5/dt)
 tail=np.array(times)>.4+1e-12;phi=np.array(rotations);v=np.array(rates)
 error=float(np.max(np.abs(phi[tail]-.08))) if tail.any() else None
 speed=float(np.max(np.abs(v[tail]))) if tail.any() else None
 overshoot=max(0.,float(phi.max()-.08));physical=max(penetrations)<=.02 and max(excursions)<=.05
 passed=bool(complete and physical and max(rises)>=.005 and error is not None and error<=.02 and speed<=.1 and overshoot<=.02)
 assert passed==t['metrics']['tracking_success']
 assert abs(error-t['metrics']['final_window_angle_error_rad'])<1e-10
 rows.append({'name':t['name'],'sign':sign,'feedback_dt_s':dt,'physics_dt_s':.002,'tracking_success':passed,'final_window_angle_error_rad':error,'final_window_pitch_speed_rad_s':speed,'rotation_overshoot_rad':overshoot,'final_rotation_rad':float(phi[-1]),'edge_rise_m':rises[-1],'max_penetration_m':max(penetrations),'max_joint_excursion_rad':max(excursions),'peak_torque_nm':peak.tolist(),'commands':len(bycmd),'physics_steps':len(physics)})
assert count==1000 and commands==600
assert all(z['tracking_success'] for z in rows if z['name']=='substep_feedback_2ms')
assert all(not z['tracking_success'] for z in rows if z['name']=='original_hold_10ms')
out={'schema':'wheelbot-contact-braking-reference/v1','passed':True,'receipt_sha256':sha(receipt),'trace_sha256':sha(raw),'verifier_sha256':sha(Path(__file__)),'model_sha256':sha(ROOT/'assets/wheelbot/contact_model.xml'),'physics_steps':count,'commands':commands,'max_state_error':max_state,'max_contact_force_error_n':max_force,'rows':rows,'scope':'Same-platform native reconstruction and declared control-rate comparison only. Different feedback rate and numerical iteration budget are explicit. No real-time, observer, arbitrary getup or hardware claim.'}
(ROOT/'evidence/wheelbot_contact_braking_reference.json').write_text(json.dumps(out,indent=2,allow_nan=False)+'\n');print(json.dumps(out,indent=2,allow_nan=False))
