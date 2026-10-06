import json
from pathlib import Path
import mujoco,numpy as np
root=Path(__file__).resolve().parents[1]
r=json.loads((root/'test-results/wheelbot-box-parity-input.json').read_text())
m=mujoco.MjModel.from_xml_path(str(root/'assets/wheelbot/live_model.xml'));d=mujoco.MjData(m)
error=0
for row in r['parity']:
 mujoco.mj_resetData(m,d);d.qpos[:]=row['before'][:6];d.qvel[:]=row['before'][6:];d.ctrl[:]=row['u']
 mujoco.mj_step(m,d,nstep=5)
 error=max(error,float(np.max(abs(np.r_[d.qpos,d.qvel]-row['after']))))
out={'commands':len(r['parity']),'modelHash':r['modelHash'],'nativeWasmMaxError':error,'passed':error<1e-8}
(root/'test-results/wheelbot-box-parity.json').write_text(json.dumps(out,indent=2)+'\n')
print(out);assert out['passed']
