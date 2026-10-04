"""Replay the exact checked-in MJCF and WASM input sequence in native MuJoCo.
No regenerated lookalike asset or copied equations of motion are used here.
"""
from pathlib import Path
import hashlib,json
import numpy as np
import mujoco
ROOT=Path(__file__).resolve().parents[1]
f=json.loads((ROOT/'evidence/mujoco_wasm.json').read_text())
assert mujoco.__version__==f['physics']['version'], 'WASM/native engine versions must match'
asset=ROOT/'assets/cartpole.xml';digest=hashlib.sha256(asset.read_bytes()).hexdigest()
assert digest==f['physics']['assetSha256']
m=mujoco.MjModel.from_xml_path(str(asset));d=mujoco.MjData(m)
assert (m.nq,m.nv,m.nu)==(2,2,1)
assert abs(m.opt.timestep-.005)<1e-15
assert m.actuator_gear[0,0]==1 and m.actuator_trnid[0,0]==0
cart=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'cart')
pole=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'pole')
np.testing.assert_allclose(m.body_mass[[cart,pole]],[1,.1],rtol=0,atol=1e-14)
np.testing.assert_allclose(m.body_ipos[pole],[0,0,.5],rtol=0,atol=1e-14)
np.testing.assert_allclose(m.body_inertia[pole,1],.1*.5**2/3,rtol=0,atol=1e-14)
assert np.all(m.geom_contype==0) and np.all(m.geom_conaffinity==0),'This asset does not claim contact/rolling dynamics'
r=f['replay'];x=r['x0'];d.qpos[:]=[x[0],x[2]];d.qvel[:]=[x[1],x[3]];xs=[]
for u in r['controls']:
    d.ctrl[0]=u
    for _ in range(4):mujoco.mj_step(m,d)
    xs.append([d.qpos[0],d.qvel[0],np.arctan2(np.sin(d.qpos[1]),np.cos(d.qpos[1])),d.qvel[1]])
err=np.asarray(xs)-np.asarray(r['states']);max_error=float(np.max(np.abs(err)))
assert max_error<1e-10, f'native/WASM replay mismatch {max_error}'
# Parameter changes use the exact canonical asset and native MuJoCo APIs, not copied dynamics.
parameter_errors=[]
for case in f['parameterReplays']:
    mm=mujoco.MjModel.from_xml_path(str(asset));dd=mujoco.MjData(mm);p=case['params']
    mm.body_mass[cart]=p['mc'];mm.body_mass[pole]=p['mp'];mm.body_ipos[pole,2]=p['l']
    mm.body_inertia[pole,:2]=p['mp']*p['l']**2/3;mm.dof_damping[0]=p['friction']
    mujoco.mj_setConst(mm,dd)
    x=case['x0'];dd.qpos[:]=[x[0],x[2]];dd.qvel[:]=[x[1],x[3]];trajectory=[]
    for inp in case['inputs']:
        dd.ctrl[0]=inp['u'];dd.qfrc_applied[0]=inp['external']
        for _ in range(4):mujoco.mj_step(mm,dd)
        trajectory.append([dd.qpos[0],dd.qvel[0],np.arctan2(np.sin(dd.qpos[1]),np.cos(dd.qpos[1])),dd.qvel[1]])
    pe=np.asarray(trajectory)-case['states'];pe[:,2]=np.arctan2(np.sin(pe[:,2]),np.cos(pe[:,2]))
    error=float(np.max(np.abs(pe)));assert error<1e-10, f'parameterized asset mismatch: {error}'
    parameter_errors.append({'parameters':p,'steps':len(trajectory),'maxAbsoluteStateError':error})
out={'schema':'cartpole-wasm-native-parity/v1','mujocoVersion':mujoco.__version__,'assetSha256':digest,'controlSteps':len(xs),'maxAbsoluteStateError':max_error,'tolerance':1e-10,'passed':True,'hardwareVerified':False,'parameterizedReplay':parameter_errors}
(ROOT/'evidence/wasm_native_parity.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps(out,indent=2))
