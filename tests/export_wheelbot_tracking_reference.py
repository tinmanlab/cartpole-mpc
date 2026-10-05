"""Fresh native runtime replay; historical trace hashes are provenance only."""
from pathlib import Path
import hashlib, json
import mujoco
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
def read(name): return json.loads((ROOT/name).read_text())
def sha(name): return hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
p = read('assets/wheelbot/contact_tracking_profile.json')
c = read('tests/fixtures/wheelbot_contact_tracking.json')
assert sha('assets/wheelbot/contact_tracking_profile.json') == '8ecc3f2719e63c5dfc3835279871506ba13e04158f5a81b170c2544926d0a67c'
for key, path in [('model','assets/wheelbot/contact_model.xml'),('cost','assets/wheelbot/contact_profile.json'),('protocol','tests/fixtures/wheelbot_contact_tracking.json')]:
    assert p['identities'][key+'_sha256'] == sha(path)
assert c['model_sha256'] == p['identities']['model_sha256']
assert mujoco.__version__ == '3.7.0' and p['steps'] == 250 and p['dt'] == .002
m = mujoco.MjModel.from_xml_path(str(ROOT/'assets/wheelbot/contact_model.xml'))
d, geo = mujoco.MjData(m), mujoco.MjData(m)
assert (m.nq,m.nv,m.nu,m.neq) == (6,6,3,0) and m.opt.timestep == .002
assert int(m.opt.enableflags) == int(m.opt.disableflags) == 0
np.testing.assert_array_equal(m.actuator_trnid[:,0], [3,4,5])
limits = np.array(p['limitsNm']); np.testing.assert_array_equal(limits,m.actuator_ctrlrange[:,1])
idx = p['controlledIndices']; assert idx == [0,1,2,3,4,6,7,8,9,10,11]
body = mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'torso')
g = mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'torso_visual')
def geometry(x, sign):
    mujoco.mj_resetData(m,geo); geo.qpos[:]=x[:6]; geo.qvel[:]=x[6:]; mujoco.mj_forward(m,geo)
    local=np.array([sign*m.geom_size[g,0],0,m.geom_pos[g,2]-m.geom_size[g,2]])
    edge=float((geo.xpos[body]+geo.xmat[body].reshape(3,3)@local)[2])
    pen=max([0.]+[-float(z.dist) for z in geo.contact])
    exc=max(0.,m.jnt_range[3,0]-x[3],x[3]-m.jnt_range[3,1],m.jnt_range[4,0]-x[4],x[4]-m.jnt_range[4,1])
    return edge,pen,float(exc)
trials=[]
for sign in [-1,1]:
    ref=np.array(p['trajectories'][str(sign)]['ref']); U=np.array(p['trajectories'][str(sign)]['u']); K=np.array(p['trajectories'][str(sign)]['K'])
    for v in c['variants']:
        for mode in ['feedforward','tvlqr']:
            x=ref[0].copy(); x[v['index']]+=v['offset']; initial=x.copy()
            edge0,peak_pen,peak_exc=geometry(x,sign); rise=overshoot=error=speed=0.; history=[]
            for k in range(250):
                requested=U[k]-K[k]@(x-ref[k])[idx] if mode=='tvlqr' else U[k]
                u=np.clip(requested,-limits,limits)
                mujoco.mj_resetData(m,d); d.qpos[:]=x[:6]; d.qvel[:]=x[6:]; d.ctrl[:]=u
                assert not d.qfrc_applied.any() and not d.xfrc_applied.any()
                mujoco.mj_step(m,d); assert abs(d.time-.002)<1e-15
                x=np.r_[d.qpos,d.qvel].copy(); assert np.isfinite(x).all()
                edge,pen,exc=geometry(x,sign); pen=max(pen,max([0.]+[-float(z.dist) for z in d.contact]))
                rotation=sign*(x[2]-initial[2]); rise=max(rise,edge-edge0); peak_pen=max(peak_pen,pen); peak_exc=max(peak_exc,exc); overshoot=max(overshoot,rotation-.08)
                if k>=200: error=max(error,abs(rotation-.08)); speed=max(speed,abs(x[8]))
                history.append({'after':x.tolist(),'tau':u.tolist(),'edge_rise_m':edge-edge0,'penetration_m':pen,'joint_excursion_rad':exc,'time_s':(k+1)*.002,'external_forces_zero':True})
            success=bool(peak_pen<=.02 and peak_exc<=.05 and rise>=.005 and error<=.02 and speed<=.1 and overshoot<=.02)
            trials.append({'sign':sign,'variant':v,'controller':mode,'initial_state':initial.tolist(),'history':history,'metrics':{'tracking_success':success,'final_window_angle_error_rad':error,'final_window_pitch_speed_rad_s':speed,'rotation_overshoot_rad':overshoot,'edge_rise_m':rise,'max_penetration_m':peak_pen,'max_joint_excursion_rad':peak_exc}})
assert len(trials)==20 and sum(len(t['history']) for t in trials)==5000
assert sum(t['metrics']['tracking_success'] for t in trials if t['controller']=='tvlqr')==10
out=ROOT/'test-results/tracking-native-runtime.json'; out.parent.mkdir(exist_ok=True)
out.write_text(json.dumps(trials,allow_nan=False)+'\n')
print('Fresh native reference: 20 cases, 5000 actual 2ms steps, 10/10 feedback passes')
