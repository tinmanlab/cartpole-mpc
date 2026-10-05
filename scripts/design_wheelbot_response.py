"""Model-only one-parameter response design; no nonlinear outcome inputs."""
from pathlib import Path
import hashlib
import json
import numpy as np
from scipy.linalg import solve_discrete_are, solve, eigvals, norm
from scipy.optimize import brentq

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'assets/wheelbot/profile.json'
OUT = ROOT / 'assets/wheelbot/response_profile.json'

def design():
    base = json.loads(SOURCE.read_text())
    A, B, Q0, R = [np.array(base[k]) for k in ('A', 'B', 'Q', 'R')]
    calls = []
    def evaluate(log_factor):
        Q = Q0.copy()
        Q[0, 0] *= 10. ** log_factor
        P = solve_discrete_are(A, B, Q, R)
        K = solve(R + B.T @ P @ B, B.T @ P @ A)
        F = A - B @ K
        e = np.zeros(11)
        e[0] = -.03
        errors = []
        for sample in range(301):
            if sample >= 250:
                errors.append(float(abs(e[0])))
            e = F @ e
        metric = max(errors)
        calls.append({'log10Factor': float(log_factor), 'factor': float(10. ** log_factor),
                      'tailError_m': metric, 'residual_m': metric - .01})
        return Q, P, K, errors
    def equation(z):
        return max(evaluate(z)[3]) - .01
    endpoints = [equation(0.), equation(4.)]
    report = {'schema': 'wheelbot-response-design/v2',
              'correction': 'Prior global-monotonicity rejection was a conservative policy, not mathematical impossibility. Brent requires continuity and a sign bracket; the requested maximum-absolute tail metric is bracketed.',
              'method': 'scipy.optimize.brentq in log10 factor, fixed bracket [0,4]',
              'equation': 'max_{k=250..300} |[(A-B K(10^z))^k (-0.03 unit_x)]_x| - 0.01 = 0',
              'scope': 'model-based one-parameter design, not nonlinear/controller optimality; no nonlinear data read',
              'sourceProfile': 'assets/wheelbot/profile.json',
              'sourceProfileSha256': hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
              'modelSha256': hashlib.sha256((ROOT / base['asset']).read_bytes()).hexdigest(),
              'designScriptSha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'log10Bracket': [0, 4], 'endpointResiduals_m': endpoints,
              'accepted': False, 'rootEdge': False, 'failure': None}
    if endpoints[0] * endpoints[1] > 0:
        report['failure'] = 'No endpoint sign bracket; no profile generated'
    else:
        z, info = brentq(equation, 0., 4., xtol=1e-12, rtol=1e-12, full_output=True)
        Q, P, K, errors = evaluate(z)
        dare = float(norm(A.T @ P @ A - P - A.T @ P @ B @ K + Q, np.inf) / max(1., norm(P, np.inf)))
        radius = float(max(abs(eigvals(A-B@K))))
        assert info.converged and dare < 1e-8 and radius < 1
        assert abs(max(errors)-.01) < 1e-10
        profile = dict(base)
        profile.update(Q=Q.tolist(), P=P.tolist(), K=K.tolist(),
                       dareNormalizedResidual=dare, closedLoopRadius=radius)
        profile['responseDesign'] = {'sourceProfileSha256': report['sourceProfileSha256'],
                                    'factor': float(10. ** z), 'changedPreference': 'Qc[0,0] only',
                                    'method': report['method'], 'scope': report['scope'],
                                    'stepM': .03, 'tailSamples': [250, 300], 'targetM': .01}
        payload = json.dumps(profile, indent=2, allow_nan=False) + '\n'
        # Deterministic model-only regeneration; never consumes test results.
        OUT.write_text(payload)
        report.update(accepted=True, rootLog10Factor=float(z), rootFactor=float(10. ** z),
                      rootResidual_m=max(errors)-.01, predictedTailError_m=max(errors),
                      predictedTailErrors_m=errors, targetAttainedWithinTolerance=True,
                      targetNumericalTolerance_m=1e-10, rootEdge=bool(z <= 1e-10 or z >= 4-1e-10),
                      brentFunctionCalls=info.function_calls, brentIterations=info.iterations,
                      dareNormalizedResidual=dare, closedLoopRadius=radius,
                      responseProfileSha256=hashlib.sha256(payload.encode()).hexdigest())
    report['evaluations'] = calls
    report['actualFunctionEvaluations'] = len(calls)
    (ROOT/'evidence/wheelbot_response_design.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    return report

if __name__ == '__main__':
    report = design()
    print(json.dumps({k: v for k, v in report.items() if k not in ('evaluations', 'predictedTailErrors_m')}, indent=2))
