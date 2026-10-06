"""Native trim/FD checks and native replay of actual WASM action transitions."""
import hashlib,json,sys
from pathlib import Path
import numpy as np, mujoco
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from design_wheelbot_live import linearize
bundle=json.loads((ROOT/'assets/wheelbot/pose_profiles.json').read_text())
xml=ROOT/'assets/wheelbot/live_model.xml'
assert hashlib.sha256(xml.read_bytes()).hexdigest()==bundle['assetSha256']
m=mujoco.MjModel.from_xml_path(str(xml));d=mujoco.MjData(m)
assert m.nu==3 and list(m.actuator_trnid[:,0])==[3,4,5]
assert abs(m.body_mass.sum()-1.19915)<1e-12
max_trim=max_fd=max_replay=0.
for p in bundle['profiles']:
    mujoco.mj_resetData(m,d);d.qpos[:]=p['qref'];d.ctrl[:]=p['uref'];mujoco.mj_forward(m,d)
    max_trim=max(max_trim,float(np.max(abs(d.qacc))))
    assert p['closedLoopRadius']<1 and p['observerErrorRadius']<1 and p['dareNormalizedResidual']<1e-8
    assert p['measurementIndices']==[0,1,2,3,4,11] and p['measurementSigma']==[.001,.001,.002,.003,.003,.02]
    A,B,_=linearize(p['qref'],p['uref'])
    max_fd=max(max_fd,float(np.max(abs(A-p['A']))),float(np.max(abs(B-p['B']))))
fixture=json.loads((ROOT/'test-results/wheelbot-box-parity-input.json').read_text())
assert fixture['modelHash']==bundle['assetSha256']
for t in fixture['parity']:
    mujoco.mj_resetData(m,d);d.qpos[:]=t['before'][:6];d.qvel[:]=t['before'][6:];d.ctrl[:]=t['u'];d.qfrc_applied[:3]=0
    for _ in range(5):mujoco.mj_step(m,d)
    max_replay=max(max_replay,float(np.max(abs(np.r_[d.qpos,d.qvel]-t['after']))))
assert max_trim<1e-7 and max_fd<1e-8 and max_replay<1e-6,(max_trim,max_fd,max_replay)
receipt={'nativeVersion':mujoco.__version__,'trimMaxQacc':max_trim,'finiteDifferenceRepeatMaxAbs':max_fd,'wasmNativeTransitionMaxAbs':max_replay,'replayedTransitions':len(fixture['parity'])}
path=ROOT/'test-results/wheelbot-box-atlas.json';path.write_text(json.dumps(receipt,indent=2)+'\n')
print(json.dumps(receipt))
