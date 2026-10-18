"""Replay completed native action traces on the original no-task physical XML."""
from pathlib import Path
import csv,json
import numpy as np
import mujoco
root=Path(__file__).resolve().parents[1]
m=mujoco.MjModel.from_xml_path(str(root/'assets/wheelbot/contact_model.xml'))
for receipt in sorted((root/'evidence').glob('wheelbot_mjpc_*.json')):
    r=json.loads(receipt.read_text())
    if r.get('returncode')!=0 or 'trace' not in r: continue
    with (root/r['trace']).open() as f: rows=list(csv.DictReader(f))
    d=mujoco.MjData(m)
    d.qpos[:]=r['trim_q']
    if r['mode']!='standing':
        d.qpos[1]=.7;d.qpos[2]=-1.1 if r['mode']=='left' else 1.1
        mujoco.mj_step(m,d,nstep=2000)
    mujoco.mj_forward(m,d)
    maximum=0.
    for row,nxt in zip(rows,rows[1:]):
        d.ctrl[:]=[float(row[f'u{i}']) for i in range(3)]
        mujoco.mj_forward(m,d);mujoco.mj_step(m,d)
        expected=np.array([float(nxt[f'{prefix}{i}']) for prefix in ['q','v'] for i in range(6)])
        maximum=max(maximum,float(np.max(abs(np.r_[d.qpos,d.qvel]-expected))))
    assert maximum<1e-6,(receipt,maximum)
    print(receipt.name,'no-task replay max error',maximum)
