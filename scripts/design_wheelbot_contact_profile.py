"""Reuse the existing native LQR/KF designer on an explicitly generated variant.

Changing shape/mass/motor/contact invalidates old model hashes. This command
produces a new local design; it is NOT an automatic self-righting certificate.
"""
from pathlib import Path
import argparse,hashlib,json
import numpy as np
import mujoco
import design_wheelbot_profile as existing

ROOT=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser()
parser.add_argument('--model',default='assets/wheelbot/contact_model.xml')
parser.add_argument('--output',default='assets/wheelbot/contact_profile.json')
args=parser.parse_args()
xml=(ROOT/args.model).resolve();out=(ROOT/args.output).resolve()
if not xml.is_relative_to(ROOT) or not out.is_relative_to(ROOT):raise ValueError('Use paths inside the repository')
existing.XML=xml
model=mujoco.MjModel.from_xml_path(str(xml))
lookup=lambda name:mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,name)
height=model.geom_size[lookup('wheel_visual'),0]+2*model.geom_size[lookup('upper_link_visual'),1]*np.cos(.15)+2*model.geom_size[lookup('lower_link_visual'),1]*np.cos(-.15)-.0001
profile=existing.design(initial_height=height)
geom=lambda name:mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,name)
body=lambda name:mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,name)
profile['asset']=str(xml.relative_to(ROOT));profile['model']='Parameterized full-body-contact planar wheelbot; declared primitives, not calibrated hardware'
profile['limitsNm']=model.actuator_ctrlrange[:,1].tolist()
if np.any(np.abs(profile['uref'])>=model.actuator_ctrlrange[:,1]):raise ValueError('Trim violates selected motor torque ceiling')
profile['modelMetadata']={'totalMassKg':float(model.body_mass.sum()),'torsoMassKg':float(model.body_mass[body('torso')]),'upperMassKg':float(model.body_mass[body('upper_link')]),'lowerMassKg':float(model.body_mass[body('lower_link')]),'upperLengthM':float(model.geom_size[geom('upper_link_visual'),1]*2),'lowerLengthM':float(model.geom_size[geom('lower_link_visual'),1]*2),'wheelRadiusM':float(model.geom_size[geom('wheel_visual'),0]),'fullBodyContact':True,'derivedFrom':'Current compiled configured XML; independent local trim and Riccati design, not copied baseline gains'}
profile['scope']='New local standing design on this exact full-contact model. Fallen-state, obstacle and jump execution need separate validation.'
out.write_text(json.dumps(profile,indent=2,allow_nan=False)+'\n')
print(json.dumps({k:profile[k] for k in ['asset','assetSha256','limitsNm','qref','uref','trimQaccInf','dareNormalizedResidual','closedLoopRadius','designAvailable']},indent=2))
