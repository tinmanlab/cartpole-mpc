"""Independent native replay, primitive/source contracts and NumPy algebra checks.
Run the Node WASM test first to export actual states and observer inputs.
"""
import hashlib, json, sys, xml.etree.ElementTree as ET
from pathlib import Path
import mujoco, numpy as np
from scipy.linalg import solve_discrete_are
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from build_wheelbot_model import model_xml
p=json.loads((ROOT/'assets/wheelbot/profile.json').read_text())
xml=ROOT/'assets/wheelbot/wheelbot.xml'
assert xml.read_text()==model_xml(), 'deterministic generator'
assert hashlib.sha256(xml.read_bytes()).hexdigest()==p['assetSha256']
source=ROOT/'assets/wheelbot/source/upkie.urdf'
assert hashlib.sha256(source.read_bytes()).hexdigest()=='d15965215067276203599a850e5c7ebb319ed6815506f0a2721dae78abac2483'
provenance=json.loads((ROOT/'assets/wheelbot/source/provenance.json').read_text())
assert hashlib.sha256((ROOT/'assets/wheelbot/source/LICENSE').read_bytes()).hexdigest()==provenance['licenseSha256']
assert 'Apache License' in (ROOT/'assets/wheelbot/source/LICENSE').read_text()

u=ET.parse(source).getroot(); tree=ET.parse(xml).getroot()
m=mujoco.MjModel.from_xml_path(str(xml));d=mujoco.MjData(m)
assert (m.nq,m.nv,m.nu,m.neq,m.ntendon)==(6,6,3,0,0)
assert m.nkey==1 and np.allclose(m.key_qpos[0,3:5],[.15,-.3])
assert np.all(m.dof_damping[:3]==0) and np.all(m.dof_armature[:3]==0)
assert np.allclose(m.jnt_axis,[[1,0,0],[0,0,1]]+[[0,1,0]]*4)
assert np.array_equal(m.actuator_trnid[:,0],[3,4,5]) and np.all(m.actuator_gear[:,0]==1)
assert np.all(m.body_mass[1:]>0) and abs(m.body_mass.sum()-1.19915)<1e-12
assert np.all(m.body_inertia[1:]>0)
# Independent analytic inertias of the selected solid primitives.
size=np.array([.125,.17,.25]);expected_box=np.array([size[1]**2+size[2]**2,size[0]**2+size[2]**2,size[0]**2+size[1]**2])/12
assert np.allclose(m.body_inertia[1],expected_box,rtol=1e-12,atol=1e-15)
rod_r=float(u.find("./link[@name='left_femur']/visual/geometry/cylinder").get('radius'))
rod_transverse=.056*(3*rod_r**2+.25**2)/12;rod_axial=.056*rod_r**2/2
assert np.allclose(m.body_inertia[2:4],[[rod_transverse,rod_transverse,rod_axial]]*2,rtol=1e-12,atol=1e-15)
wheel_transverse=.08715*(3*.05**2+.04**2)/12
assert np.allclose(m.body_inertia[4],[wheel_transverse,.08715*.05**2/2,wheel_transverse],rtol=1e-12,atol=1e-15)
for I in m.body_inertia[1:]:assert 2*I.max()<=I.sum()+1e-14
for i,name in enumerate(['hip','knee','wheel']):
 limit=float(u.find(f"./joint[@name='left_{name}']/limit").get('effort'))
 assert np.allclose(m.actuator_ctrlrange[i],[-limit,limit])
for name in ['left_femur','left_tibia']:
 cylinder=u.find(f"./link[@name='{name}']/visual/geometry/cylinder")
 assert float(cylinder.get('length'))==.25
assert float(u.find("./link[@name='left_wheel_tire']/collision/geometry/cylinder").get('radius'))==.05
q=np.array(p['qref']);uref=np.array(p['uref']);idx=np.array(p['controlledIndices']);x=np.r_[q,np.zeros(6)]
def state(z,ctrl):
 mujoco.mj_resetData(m,d);d.qpos[:]=z[:6];d.qvel[:]=z[6:];d.ctrl[:]=ctrl
 mujoco.mj_forward(m,d)
def step(z,ctrl,external=0):
 state(z,ctrl);d.qfrc_applied[0]=external
 for _ in range(5):mujoco.mj_step(m,d)
 return np.r_[d.qpos,d.qvel]
state(x,uref);trim=float(abs(d.qacc).max());assert trim<1e-8 and d.ncon>0
assert abs(q[3])>.1 and abs(q[4])>.2 and abs(uref[1])>.1
state(x,np.zeros(3));assert np.max(abs(d.qacc))>1
motor_effect=[]
for i in range(3):
 a=uref.copy();a[i]+=.1;state(x,a);motor_effect.append(float(abs(d.qacc).max()));assert motor_effect[-1]>.1
shift=x.copy();shift[5]+=.713
phase=float(abs(step(x,uref)[idx]-step(shift,uref)[idx]).max());assert phase<1e-8
hip=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,'hip_site');knee=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,'knee_site');wheel=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,'wheel_site')
state(x,uref)
assert abs(np.linalg.norm(d.site_xpos[hip]-d.site_xpos[knee])-.25)<1e-12
assert abs(np.linalg.norm(d.site_xpos[knee]-d.site_xpos[wheel])-.25)<1e-12
assert np.max(abs(d.site_xpos[:,1]))<1e-12
A,B,Q,R,P,K,L=[np.array(p[k]) for k in ['A','B','Q','R','P','K','L']]
P2=solve_discrete_are(A,B,Q,R);assert np.max(abs(P-P2))<1e-7
assert np.max(abs(K-np.linalg.solve(R+B.T@P@B,B.T@P@A)))<1e-9
assert np.linalg.eigvalsh(P).min()>0
ric=float(np.linalg.norm(A.T@P@A-P-A.T@P@B@K+Q,np.inf)/np.linalg.norm(P,np.inf));assert ric<1e-8
H=np.eye(11)[:5];W=np.array(p['Qe']);V=np.array(p['Re'])
C=solve_discrete_are(A.T,H.T,W,V);gain=np.linalg.solve(H@C@H.T+V,H@C).T
assert np.max(abs(L-gain))<1e-10
nativeA=np.zeros((11,11));nativeB=np.zeros((11,3));eps=1e-6
for j,k in enumerate(idx):
 plus=x.copy();minus=x.copy();plus[k]+=eps;minus[k]-=eps
 nativeA[:,j]=(step(plus,uref)[idx]-step(minus,uref)[idx])/(2*eps)
for j in range(3):
 plus=uref.copy();minus=uref.copy();plus[j]+=eps;minus[j]-=eps
 nativeB[:,j]=(step(x,plus)[idx]-step(x,minus)[idx])/(2*eps)
assert np.max(abs(nativeA-A))<1e-3 and np.max(abs(nativeB-B))<1e-4
r=json.loads((ROOT/'test-results/wheelbot_wasm_reference.json').read_text())
assert r['assetSha256']==p['assetSha256']
fdparity=float(max(np.max(abs(nativeA-r['linA'])),np.max(abs(nativeB-r['linB']))));assert fdparity<1e-4
replay_error=0.;contact=0;air=0
for replay in r['replay']:
 z=np.array(replay['initial'])
 for inp,target in zip(replay['inputs'],replay['states']):
  z=step(z,inp['u'],inp['external']);replay_error=max(replay_error,float(np.max(abs(z-target))))
  contact+=int(d.ncon>0);air+=int(d.ncon==0)
assert replay_error<1e-7 and contact>0 and air>0
f=r['algebra'];ref=x[idx];estimate=np.array(f['estimate']);control=np.array(f['u']);y=np.array(f['y'])
pred=ref+A@(estimate-ref)+B@(control-uref);mean=pred+L@(y-H@pred)
observer_error=float(np.max(abs(mean-f['observer'])));assert observer_error<1e-12
reference=ref.copy();reference[0]+=f['goal'];command=uref-K@(estimate-reference)
controller_error=float(np.max(abs(command-f['control']['requested'])));assert controller_error<1e-12
# Independently reconstruct task counts and physical-unit errors from raw WASM traces.
manifest=json.loads((ROOT/'tests/fixtures/wheelbot_cases.json').read_text())
results=json.loads((ROOT/'evidence/wheelbot_validation.json').read_text())
assert len(results['rows'])==len(manifest['cases'])*len(manifest['seeds'])
for case in r['raw']:
 row=next(z for z in results['rows'] if z['name']==case['name'] and z['seed']==case['seed'])
 c=next(z for z in manifest['cases'] if z['name']==case['name'])
 trace=case['trace'];truth=np.array([z['truth'] for z in trace]);estimate=np.array([z['estimate'] for z in trace])
 np.testing.assert_allclose(np.sqrt(np.mean((truth[:,idx]-estimate)**2,axis=0)),row['estimationRmseByState'],atol=1e-12)
 assert len(row['estimationUnits'])==11 and len(row['measurementUnits'])==5
 assert row['steps']==len(trace)
 complete=len(trace)==manifest['steps'] and not trace[-1]['failed']
 task=complete and all(abs(z['truth'][0]-p['qref'][0]-c['goal'])<manifest['positionTolerance'] and abs(z['truth'][2]-p['qref'][2])<manifest['pitchTolerance'] for z in trace[-manifest['tailSteps']:])
 assert row['completed']==complete and row['taskPassed']==task
 assert row['contactLoss']==sum(not z['last']['contact']['wheelContacts'] for z in trace)
 assert row['saturations']==sum(z['last']['saturated'] for z in trace)
 for z in trace:assert np.all(np.abs(z['last']['u'])<=np.array(p['limitsNm'])+1e-12)
# Physical parameter mismatch is a diagnostic using the same trim feedforward.
# It is not used to select or retune costs; a +20% torso mass is a declared probe.
m.body_mass[1]*=1.2;m.body_inertia[1]*=1.2;mujoco.mj_setConst(m,d)
zm=x.copy()
for _ in range(50): zm=step(zm,uref)
mismatch_displacement=float(np.max(abs(zm-x)))
out={'schema':'wheelbot-native-reference/v1','assetSha256':p['assetSha256'],'trimQaccInf':trim,'riccatiResidual':ric,'closedLoopRadius':float(max(abs(np.linalg.eigvals(A-B@K)))),'phaseInvarianceError':phase,'nativeNonlinearDerivativeErrorA':float(np.max(abs(nativeA-A))),'nativeNonlinearDerivativeErrorB':float(np.max(abs(nativeB-B))),'nativeWasmDerivativeError':fdparity,'replaySteps':160,'replayMaxError':replay_error,'contactSteps':contact,'airborneSteps':air,'observerMeanError':observer_error,'controllerError':controller_error,'motorAccelerationEffects':motor_effect,'mismatchProbe':{'torsoMassScale':1.2,'feedforwardOnlySteps':50,'maxFullStateDeviation':mismatch_displacement,'scope':'open-loop trim sensitivity, not closed-loop robustness'},'passed':True}
(ROOT/'evidence/wheelbot_native_reference.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
