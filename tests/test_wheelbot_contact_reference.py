"""Native independent reference for configured full-contact WASM trials.
No success criterion is substituted for a collision merely being detected.
"""
from pathlib import Path
import json,hashlib
import numpy as np
import mujoco
from scipy.linalg import solve_discrete_are
ROOT=Path(__file__).resolve().parents[1]
raw=json.loads((ROOT/'test-results/wheelbot_contact_raw.json').read_text())
summary=json.loads((ROOT/'test-results/wheelbot_contact_validation.json').read_text())
max_state=0.;max_force=0.;frames=0;counts=[]
for trial in raw['raws']:
 m=mujoco.MjModel.from_xml_string(trial['xml']);d=mujoco.MjData(m);meta=trial['metadata'];c=meta['configuration']
 assert (m.nq,m.nv,m.nu,m.na,m.neq)==(6,6,3,0,0)
 np.testing.assert_allclose(m.body_mass.sum(),meta['totalMassKg'],atol=1e-13)
 np.testing.assert_array_equal(m.actuator_trnid[:,0],[3,4,5]);np.testing.assert_allclose(m.actuator_ctrlrange[:,1],c['motor']['torqueLimitNm'])
 for i in range(1,m.nbody):
  inertia=m.body_inertia[i];assert np.all(inertia>0) and max(inertia)<=sum(inertia)-max(inertia)+1e-12
 for name in ['torso_visual','upper_link_visual','lower_link_visual','wheel_visual']:
  g=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,name);assert m.geom_contype[g]==m.geom_conaffinity[g]==1
 prev=np.array(trial['initial']);pairs={};largest_penetration=0.;largest_normal=0.
 for row in trial['trace']:
  u=np.array(row['last']['u']);assert np.all(np.abs(u)<=c['motor']['torqueLimitNm'])
  mujoco.mj_resetData(m,d);d.qpos[:]=prev[:6];d.qvel[:]=prev[6:];d.ctrl[:]=u
  assert not np.any(d.qfrc_applied) and not np.any(d.xfrc_applied)
  mujoco.mj_step(m,d,nstep=5);now=np.r_[d.qpos,d.qvel];truth=np.array(row['truth']);max_state=max(max_state,float(np.max(np.abs(now-truth))))
  # Same post-step state and last applied torque for diagnostic reaction forces.
  mujoco.mj_resetData(m,d);d.qpos[:]=truth[:6];d.qvel[:]=truth[6:];d.ctrl[:]=u;mujoco.mj_forward(m,d)
  received=row['last']['allContacts']['pairs'];assert len(received)==d.ncon
  for k in range(d.ncon):
   contact=d.contact[k];one=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,int(contact.geom1));two=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,int(contact.geom2));r=received[k]
   assert r['geom1']==one and r['geom2']==two
   f=np.zeros(6);mujoco.mj_contactForce(m,d,k,f);max_force=max(max_force,abs(float(f[0])-r['normalForceN']))
   np.testing.assert_allclose(contact.frame.reshape(3,3).T@f[:3],r['forceOnGeom2WorldN'],atol=1e-8,rtol=1e-9)
   np.testing.assert_allclose(contact.dist,r['distanceM'],atol=1e-9)
   pairs[one+'|'+two]=pairs.get(one+'|'+two,0)+1;largest_penetration=max(largest_penetration,-float(contact.dist));largest_normal=max(largest_normal,float(f[0]))
  prev=truth;frames+=1
 result=next(r for r in summary['results']if r['variant']==trial['variant']and r['scenario']==trial['scenario'])
 assert result['pairs']==pairs and result['steps']==len(trial['trace'])
 np.testing.assert_allclose(result['maximumPenetrationM'],largest_penetration,atol=1e-9);np.testing.assert_allclose(result['maximumContactNormalN'],largest_normal,atol=1e-7)
 counts.append({'variant':trial['variant'],'scenario':trial['scenario'],'steps':len(trial['trace']),'contactPairs':pairs,'maxPenetrationM':largest_penetration,'maxNormalN':largest_normal})
assert max_state<1e-8,max_state
assert max_force<1e-6,max_force
# Regenerated local controller is bound to the different full-contact model.
p=json.loads((ROOT/'assets/wheelbot/contact_profile.json').read_text());assert p['assetSha256']==hashlib.sha256((ROOT/p['asset']).read_bytes()).hexdigest()
A,B,Q,R=[np.array(p[k])for k in ['A','B','Q','R']];P=solve_discrete_are(A,B,Q,R);K=np.linalg.solve(R+B.T@P@B,B.T@P@A)
np.testing.assert_allclose(P,p['P'],rtol=1e-9,atol=1e-9);np.testing.assert_allclose(K,p['K'],rtol=1e-9,atol=1e-9)
report={'schema':'wheelbot-full-contact-native-reference/v1','passed':True,'mujocoVersion':mujoco.__version__,'trials':len(counts),'frames':frames,'maxStateComponentError':max_state,'maxNormalForceErrorN':max_force,'counts':counts,'localProfileRecomputed':True,'localProfileSha256':hashlib.sha256((ROOT/'assets/wheelbot/contact_profile.json').read_bytes()).hexdigest(),'scope':'Full body/link collisions and configurable parameters verified. Soft-contact penetration is measured, not claimed zero. No successful stand-up is inferred.'}
(ROOT/'test-results/wheelbot_contact_reference.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
