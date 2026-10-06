"""Acceptance for a jump on the SAME compact robot, not a model swap."""
from pathlib import Path
import hashlib,json
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
p=json.loads((ROOT/'assets/wheelbot/action_jump.json').read_text())
assert p['assetSha256']==hashlib.sha256((ROOT/'assets/wheelbot/live_model.xml').read_bytes()).hexdigest()
assert p['baselineSha256']==hashlib.sha256((ROOT/'assets/wheelbot/live_profile.json').read_bytes()).hexdigest()
assert p['controlDt']==.01 and p['physicsDt']==.002 and p['steps']==600
assert p['measurementIndices']==[0,1,2,3,4,11]
assert len(p['ref'])==601 and all(len(p[k])==600 for k in ['u','A','B','K','L'])
assert p['externalForce']==[0,0,0]
assert np.all(np.abs(p['u'])<=[16,16,1.7])
assert p['referenceMetrics']['passed']
print('Compact jump model/actuator/measurement/trajectory identity PASS')
