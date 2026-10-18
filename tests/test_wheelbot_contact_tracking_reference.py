"""Independent dynamics, force, Riccati and task-evaluation replay of a fast tracker.

Does not import the tracker or its evaluation helper. Offline derivative checks
are distinct from finite physical task passes and wall-clock observations.
"""
from pathlib import Path
import hashlib,json,ast
import numpy as np
import mujoco
from scipy.linalg import solve
ROOT=Path(__file__).resolve().parents[1]
sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
read=lambda path:json.loads((ROOT/path).read_text())
def pure_runtime_imports(source):
 imports=[]
 for node in ast.walk(ast.parse(source)):
  if isinstance(node,ast.Import):imports.extend(a.name for a in node.names)
  elif isinstance(node,ast.ImportFrom):imports.append(node.module or '')
 return set(imports)<=set(['json','pathlib','numpy','hashlib'])

import unittest,sys
class PureRuntimeImportTests(unittest.TestCase):
 def test_import_boundary(self):
  self.assertTrue(pure_runtime_imports('import hashlib'))
  for source in ['import mujoco','import scipy','from scipy.linalg import solve','import mujoco as mj']:
   self.assertFalse(pure_runtime_imports(source))
if '--imports-only' in sys.argv:
 unittest.main(argv=[sys.argv[0]])

r=read('evidence/wheelbot_contact_tracking.json')
p=read('assets/wheelbot/contact_tracking_profile.json')
c=read('tests/fixtures/wheelbot_contact_tracking.json')
source=ROOT/'assets/wheelbot/contact_model.xml'
assert r['native_version']==mujoco.__version__=='3.7.0'
assert r['profile_sha256']==sha(ROOT/'assets/wheelbot/contact_tracking_profile.json')
for name,digest in {**r['source_hashes'],**r['raw_hashes']}.items():
 path=(ROOT/name).resolve();assert path.is_relative_to(ROOT) and sha(path)==digest,name
assert r['identities']==p['identities']
assert p['identities']['model_sha256']==sha(source)
assert p['identities']['cost_sha256']==sha(ROOT/'assets/wheelbot/contact_profile.json')
assert p['identities']['protocol_sha256']==sha(ROOT/'tests/fixtures/wheelbot_contact_tracking.json')
assert p['identities']['trace_sha256']==sha(ROOT/'test-results/contact-braking-traces.json')
assert p['generator_sha256']==sha(ROOT/'scripts/design_wheelbot_contact_tracking.py')
assert p['runtime_sha256']==sha(ROOT/'scripts/wheelbot_contact_tracking.py')
# Runtime performs only indexed feedback; no inverse/optimizer/plant module import.
assert pure_runtime_imports((ROOT/'scripts/wheelbot_contact_tracking.py').read_text())
assert p['controlledIndices']==[0,1,2,3,4,6,7,8,9,10,11]
assert p['dt']==.002 and p['steps']==250 and p['terminal']=='Q'
assert c['duration_s']==.5 and c['final_window_s']==.1 and c['reference_rotation_rad']==.08
assert c['maximum_final_angle_error_rad']==.02 and c['maximum_final_pitch_speed_rad_s']==.1 and c['maximum_rotation_overshoot_rad']==.02
assert c['minimum_hip_edge_rise_m']==.005 and c['maximum_joint_excursion_rad']==.05 and c['maximum_penetration_m']==.02
idx=np.array(p['controlledIndices']);cost=read('assets/wheelbot/contact_profile.json')
Q=np.array(p['Q']);R=np.array(p['R']);np.testing.assert_array_equal(Q,cost['Q']);np.testing.assert_array_equal(R,cost['R'])
design=read('test-results/contact-tracking-design.json');traces=read('test-results/contact-tracking-trials.json')
m=mujoco.MjModel.from_xml_path(str(source));d=mujoco.MjData(m);geo=mujoco.MjData(m)
assert (m.nq,m.nv,m.nu,m.neq)==(6,6,3,0) and m.opt.timestep==.002
assert int(m.opt.enableflags)==int(m.opt.disableflags)==0
np.testing.assert_array_equal(m.actuator_trnid[:,0],[3,4,5]);np.testing.assert_array_equal(m.actuator_ctrlrange[:,1],[16,16,1.7])
limits=m.actuator_ctrlrange[:,1];body=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'torso');geom=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'torso_visual')
def geometry(x,sign):
 mujoco.mj_resetData(m,geo);geo.qpos[:]=x[:6];geo.qvel[:]=x[6:];mujoco.mj_forward(m,geo)
 local=np.array([sign*m.geom_size[geom,0],0,m.geom_pos[geom,2]-m.geom_size[geom,2]])
 edge=float((geo.xpos[body]+geo.xmat[body].reshape(3,3)@local)[2])
 penetration=max([0.]+[-float(z.dist)for z in geo.contact]);excursion=max(0.,m.jnt_range[3,0]-x[3],x[3]-m.jnt_range[3,1],m.jnt_range[4,0]-x[4],x[4]-m.jnt_range[4,1])
 return edge,penetration,float(excursion)
riccati_error=0.;minP=float('inf')
for sign in ['-1','1']:
 P=Q.copy();values=design['trajectories'][sign];gains=p['trajectories'][sign]['K']
 assert len(gains)==len(values['A'])==len(values['B'])==250
 for k in reversed(range(250)):
  A=np.array(values['A'][k]);B=np.array(values['B'][k]);K=np.array(gains[k]);H=R+B.T@P@B;G=B.T@P@A
  independently=solve(H,G,assume_a='pos');riccati_error=max(riccati_error,float(np.max(np.abs(independently-K))))
  F=A-B@K;P=Q+K.T@R@K+F.T@P@F;P=(P+P.T)/2;minP=min(minP,float(np.linalg.eigvalsh(P).min()))
assert riccati_error<1e-7 and minP>0
expected={(sign,v['name'],mode)for sign in [-1,1]for v in c['variants']for mode in ['feedforward','tvlqr']}
assert {(z['sign'],z['variant']['name'],z['controller'])for z in traces}==expected
assert len(traces)==len(expected)==len(r['trials'])
state_error=0.;command_error=0.;frames=0;results=[]
for trial in traces:
 sign=trial['sign'];nom=np.array(p['trajectories'][str(sign)]['ref']);U=np.array(p['trajectories'][str(sign)]['u']);K=np.array(p['trajectories'][str(sign)]['K']);variant=next(v for v in c['variants']if v['name']==trial['variant']['name'])
 initial=nom[0].copy();initial[variant['index']]+=variant['offset'];np.testing.assert_array_equal(initial,trial['initial_state'])
 edge0,pen0,exc0=geometry(initial,sign);admissible=pen0<=.02 and exc0<=.05;assert trial['initial_admissible']==admissible
 previous=initial.copy();phi=[];velocity=[];heights=[];pens=[];excs=[];tau_max=np.zeros(3)
 for k,row in enumerate(trial['history']):
  requested=U[k]-K[k]@(previous-nom[k])[idx]if trial['controller']=='tvlqr'else U[k]
  u=np.clip(requested,-limits,limits);actual=np.array(row['tau']);command_error=max(command_error,float(np.max(np.abs(actual-u))))
  assert np.all(np.abs(actual)<=limits+1e-12) and row['external_forces_zero']
  mujoco.mj_resetData(m,d);d.qpos[:]=previous[:6];d.qvel[:]=previous[6:];d.ctrl[:]=actual
  assert not d.qfrc_applied.any()and not d.xfrc_applied.any();mujoco.mj_step(m,d)
  now=np.r_[d.qpos,d.qvel].copy();state_error=max(state_error,float(np.max(np.abs(now-row['after']))));edge,pen,exc=geometry(now,sign)
  pen=max(pen,max([0.]+[-float(z.dist)for z in d.contact]));np.testing.assert_allclose([edge-edge0,pen,exc],[row['edge_rise_m'],row['penetration_m'],row['joint_excursion_rad']],atol=1e-8,rtol=0)
  assert abs(row['time_s']-(k+1)*.002)<1e-12
  phi.append(float(sign*(now[2]-initial[2])));velocity.append(float(now[8]));heights.append(edge-edge0);pens.append(pen);excs.append(exc);tau_max=np.maximum(tau_max,np.abs(actual));previous=now;frames+=1
 n=len(phi);tail=np.arange(1,n+1)*.002>.4+1e-12
 error=float(np.max(np.abs(np.array(phi)[tail]-.08)))if tail.any()else None
 speed=float(np.max(np.abs(np.array(velocity)[tail])))if tail.any()else None
 overshoot=max(0.,max(phi,default=0.)-.08);physical=max(pens,default=0.)<=.02 and max(excs,default=0.)<=.05
 success=bool(n==250 and physical and max(heights,default=0.)>=.005 and error is not None and error<=.02 and speed<=.1 and overshoot<=.02)
 assert success==trial['metrics']['tracking_success']
 if error is not None:assert abs(error-trial['metrics']['final_window_angle_error_rad'])<1e-8
 results.append({'sign':sign,'variant':variant['name'],'controller':trial['controller'],'steps':n,'passed':success,'final_window_angle_error_rad':error,'final_window_pitch_speed_rad_s':speed,'overshoot_rad':overshoot,'edge_rise_m':max(heights,default=0.),'peak_torque_nm':tau_max.tolist(),'max_penetration_m':max(pens,default=0.),'max_joint_excursion_rad':max(excs,default=0.)})
assert state_error<1e-8 and command_error<1e-8
feedback=[z for z in results if z['controller']=='tvlqr'];baseline=[z for z in results if z['controller']=='feedforward']
report={'schema':'wheelbot-contact-tracking-independent-reference/v1','arithmetic_and_physics_verified':True,'source_receipt_sha256':sha(ROOT/'evidence/wheelbot_contact_tracking.json'),'source_profile_sha256':sha(ROOT/'assets/wheelbot/contact_tracking_profile.json'),'verifier_sha256':sha(Path(__file__)),'trials':len(results),'physics_steps':frames,'max_native_state_error':state_error,'max_command_error_nm':command_error,'max_riccati_gain_error':riccati_error,'minimum_cost_to_go_eigenvalue':minP,'linearization_validation':r['derivatives'],'feedback_passes':sum(z['passed']for z in feedback),'feedback_trials':len(feedback),'feedforward_passes':sum(z['passed']for z in baseline),'feedforward_trials':len(baseline),'results':results,'scope':'Independent native reconstruction of a known trajectory and declared perturbations. This is time-varying feedback, not an online nonlinear optimizer or fixed standing gain. Derivative admission, runtime deadline and physical passes are distinct; no observer/hardware/fullgetup claim.'}
(ROOT/'evidence/wheelbot_contact_tracking_reference.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n');print(json.dumps(report,indent=2,allow_nan=False))
