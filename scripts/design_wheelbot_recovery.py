"""One model-only Qx design for position AND local disturbance recovery.

The declared target is evaluated with the existing KF mean in the loop.
No nonlinear assessment trace, noise seed, or outcome is an optimization input.
"""
from pathlib import Path
import copy
import hashlib
import json

import mujoco
import numpy as np
from scipy.linalg import eigvals, norm, solve, solve_discrete_are
from scipy.optimize import brentq

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'assets/wheelbot/profile.json'
PROTOCOL = ROOT / 'tests/fixtures/wheelbot_recovery_validation.json'
OUT = ROOT / 'assets/wheelbot/recovery_profile.json'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def derive():
    base = json.loads(SOURCE.read_text())
    protocol = json.loads(PROTOCOL.read_text())
    spec = protocol['design']
    A, B, Q0, R, L = [np.asarray(base[k], dtype=float) for k in ('A', 'B', 'Q', 'R', 'L')]
    H = np.eye(11)[:5]
    model = mujoco.MjModel.from_xml_path(str(ROOT / base['asset']))
    data = mujoco.MjData(model)
    indices = base['controlledIndices']
    assert mujoco.__version__ == '3.7.0'
    assert sha(ROOT / base['asset']) == base['assetSha256']
    assert base['controlDt'] == protocol['runtime']['controlDt'] == .01
    assert abs(model.opt.timestep * base['substeps'] - .01) < 1e-14

    def forced_step(force):
        mujoco.mj_resetData(model, data)
        data.qpos[:] = base['qref']
        data.ctrl[:] = base['uref']
        data.qfrc_applied[0] = force
        for _ in range(base['substeps']):
            mujoco.mj_step(model, data)
        return np.r_[data.qpos, data.qvel][indices].copy()

    eps = spec['disturbanceDifferenceN']
    eps_check = spec['refinementDifferenceN']
    D = (forced_step(eps) - forced_step(-eps)) / (2 * eps)
    D_check = (forced_step(eps_check) - forced_step(-eps_check)) / (2 * eps_check)
    derivative_error = float(np.max(np.abs(D - D_check)))
    assert np.isfinite(D).all() and derivative_error < 1e-7
    evaluations = []

    def evaluate(log_factor):
        Q = Q0.copy()
        Q[0, 0] *= 10. ** log_factor
        P = solve_discrete_are(A, B, Q, R)
        K = solve(R + B.T @ P @ B, B.T @ P @ A)
        results = {}
        for name, goal, amplitude in [('position', spec['positionStepM'], 0.),
                                      ('push', 0., spec['pulseForceN'])]:
            state = np.zeros(11)
            estimate = np.zeros(11)
            target = np.zeros(11)
            target[0] = goal
            tail = []
            torques = []
            for sample in range(spec['tailSamples'][1] + 1):
                if sample >= spec['tailSamples'][0]:
                    tail.append([abs(state[0] - goal), abs(state[2])])
                delta_u = -K @ (estimate - target)
                torques.append(delta_u + base['uref'])
                lo, hi = spec['pulseSamples']
                force = amplitude if lo <= sample < hi else 0.
                state = A @ state + B @ delta_u + D * force
                prediction = A @ estimate + B @ delta_u
                estimate = prediction + L @ (H @ state - H @ prediction)
            results[name] = {
                'positionTailMaxM': float(np.max(np.asarray(tail)[:, 0])),
                'pitchTailMaxRad': float(np.max(np.asarray(tail)[:, 1])),
                'maxAbsTorqueNm': np.max(np.abs(torques), axis=0).tolist(),
            }
        metric = max(max(v['positionTailMaxM'] / spec['positionTargetM'],
                         v['pitchTailMaxRad'] / spec['pitchTargetRad'])
                     for v in results.values()) - 1.
        evaluations.append({'log10Factor': float(log_factor), 'factor': float(10. ** log_factor),
                            'normalizedTargetResidual': float(metric), 'cases': results})
        return metric, Q, P, K, results

    lower, upper = spec['log10FactorBracket']
    endpoint_residuals = [evaluate(lower)[0], evaluate(upper)[0]]
    if endpoint_residuals[0] * endpoint_residuals[1] > 0:
        raise ValueError('Recovery design rejected: combined target has no endpoint sign bracket')
    z, info = brentq(lambda x: evaluate(x)[0], lower, upper,
                     xtol=1e-10, rtol=1e-12, full_output=True)
    residual, Q, P, K, results = evaluate(z)
    dare = float(norm(A.T @ P @ A - P - A.T @ P @ B @ K + Q, np.inf) / max(1., norm(P, np.inf)))
    radius = float(np.max(np.abs(eigvals(A - B @ K))))
    assert info.converged and abs(residual) <= spec['targetResidualTolerance']
    assert dare < 1e-8 and radius < 1.
    assert all(np.all(np.asarray(v['maxAbsTorqueNm']) < base['limitsNm']) for v in results.values())
    profile = copy.deepcopy(base)
    profile.update(Q=Q.tolist(), P=P.tolist(), K=K.tolist(),
                   dareNormalizedResidual=dare, closedLoopRadius=radius)
    profile['responseDesign'] = {
        'sourceProfileSha256': sha(SOURCE), 'factor': float(10. ** z),
        'changedPreference': 'Qc[0,0] only',
        'method': 'scipy.optimize.brentq; combined position/push target with existing KF mean',
        'scope': 'Nominal local disturbance response design; no contact-loss recovery or robust guarantee',
        'protocolSha256': sha(PROTOCOL),
        'positionTargetM': spec['positionTargetM'], 'pitchTargetRad': spec['pitchTargetRad'],
        'tailSamples': spec['tailSamples'],
    }
    payload = json.dumps(profile, indent=2, allow_nan=False) + '\n'
    report = {
        'schema': 'wheelbot-recovery-design/v1', 'accepted': True,
        'sourceProfileSha256': sha(SOURCE), 'protocolSha256': sha(PROTOCOL),
        'assetSha256': sha(ROOT / base['asset']), 'designScriptSha256': sha(Path(__file__)),
        'profileSha256': hashlib.sha256(payload.encode()).hexdigest(),
        'method': profile['responseDesign']['method'],
        'equation': 'max over position/push of tail position/0.010m and pitch/0.020rad minus 1',
        'log10FactorBracket': [lower, upper], 'endpointResiduals': endpoint_residuals,
        'factor': float(10. ** z), 'rootResidual': float(residual),
        'dareNormalizedResidual': dare, 'closedLoopRadius': radius,
        'disturbanceColumnPerN': D.tolist(), 'disturbanceRefinementError': derivative_error,
        'modelCases': results, 'evaluations': evaluations,
        'actualModelEvaluations': len(evaluations), 'brentFunctionCalls': info.function_calls,
        'brentIterations': info.iterations, 'nonlinearAssessmentInputs': False,
        'scope': spec['model'],
        'sources': ['https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.brentq.html',
                    'https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html'],
    }
    return payload, report


if __name__ == '__main__':
    payload, report = derive()
    OUT.write_text(payload)
    (ROOT / 'evidence/wheelbot_recovery_design.json').write_text(
        json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'evaluations'}, indent=2))
