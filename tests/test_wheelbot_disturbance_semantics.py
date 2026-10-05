"""Independent native audit of the *existing* disturbance, not a new easier task.

A generalized root-x force equals a Cartesian force at the torso/hip origin,
not a COM force. Preserve this distinction before diagnosing control failure.
"""
from pathlib import Path
import hashlib
import json
import unittest

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
ASSET = ROOT / 'assets/wheelbot/wheelbot.xml'


def force_mappings():
    model = mujoco.MjModel.from_xml_path(str(ASSET))
    data = mujoco.MjData(model)
    profile = json.loads((ROOT / 'assets/wheelbot/recovery_profile.json').read_text())
    data.qpos[:] = profile['qref']
    mujoco.mj_forward(model, data)
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'torso')
    hip = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'hip_site')
    force = np.array([40., 0., 0.])
    mappings = {}
    for name, point in [('hip_root', data.site_xpos[hip]), ('torso_com', data.xipos[body])]:
        generalized = np.zeros(model.nv)
        mujoco.mj_applyFT(model, data, force, np.zeros(3), point, body, generalized)
        mappings[name] = {'worldPointM': point.tolist(), 'generalizedForce': generalized.tolist()}
    moment = np.cross(data.site_xpos[hip] - data.xipos[body], force)
    return mappings, moment


class DisturbanceSemantics(unittest.TestCase):
    def test_generalized_force_is_hip_force_not_com_force(self):
        mappings, moment = force_mappings()
        np.testing.assert_allclose(mappings['hip_root']['generalizedForce'], [40, 0, 0, 0, 0, 0], atol=1e-12)
        self.assertAlmostEqual(mappings['torso_com']['generalizedForce'][2], -moment[1], places=11)
        self.assertGreater(abs(moment[1]), 4.9)
        self.assertAlmostEqual(mappings['hip_root']['worldPointM'][2] + .125 * np.cos(
            json.loads((ROOT/'assets/wheelbot/profile.json').read_text())['qref'][2]),
            mappings['torso_com']['worldPointM'][2], places=11)

    def test_documentation_names_existing_force_point(self):
        text = (ROOT/'docs/WHEELBOT_2D.md').read_text()
        self.assertIn('generalized root-x force', text)
        self.assertIn('torso/hip origin', text)
        self.assertIn('not a force at the torso COM', text)


if __name__ == '__main__':
    tests = unittest.defaultTestLoader.loadTestsFromTestCase(DisturbanceSemantics)
    result = unittest.TextTestRunner(verbosity=2).run(tests)
    if not result.wasSuccessful():
        raise SystemExit(1)
    mappings, moment = force_mappings()
    report = {
        'schema': 'wheelbot-disturbance-semantics/v1', 'passed': True,
        'assetSha256': hashlib.sha256(ASSET.read_bytes()).hexdigest(),
        'mappingsAtTrim': mappings, 'hipForceMomentAboutTorsoComNm': moment.tolist(),
        'unchangedRuntimeDisturbance': True,
        'claim': 'Current force point is physically valid but must be stated. Moving the force to COM changes the task and cannot count as solving the old task.',
        'source': 'https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-applyft',
    }
    (ROOT/'evidence/wheelbot_disturbance_semantics.json').write_text(json.dumps(report, indent=2)+'\n')
