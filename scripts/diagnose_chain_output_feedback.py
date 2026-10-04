"""Local output-feedback/saturation diagnostics, not a nonlinear safety certificate.
Uses existing SciPy Lyapunov/linear algebra, no new controller or covariance tuner.
"""
import json,hashlib
from pathlib import Path
import numpy as np
from scipy import linalg as la
ROOT=Path(__file__).resolve().parents[1]
ps=json.loads((ROOT/'assets/chains/profiles.json').read_text())['profiles'];rows=[]
for p in ps:
    row={'poles':p['poles'],'designAvailable':p['designAvailable']}
    if not p['designAvailable']:rows.append(row);continue
    A,B,K,H,L,P=[np.array(p[k]) for k in ['A','B','K','H','kalmanGain','P']];n=p['nx'];F=A-B@K;E=(np.eye(n)-L@H)@A
    # e = true state - posterior estimate. Linear unsaturated joint dynamics:
    # [x+;e+] = [[A-BK,BK],[0,(I-LH)A]] [x;e] + [0;-L] measurement_noise.
    Z=np.block([[F,B@K],[np.zeros((n,n)),E]]);D=np.vstack([np.zeros_like(L),-L]);C=np.hstack([-K,K]);R=np.eye(p['dof'])*(2e-4)**2
    Sigma=la.solve_discrete_lyapunov(Z,D@R@D.T)
    lyap_res=la.norm(Z@Sigma@Z.T-Sigma+D@R@D.T,np.inf)/max(1,la.norm(Sigma,np.inf))
    force_var=float((C@Sigma@C.T).item());immediate=float(la.norm(K@L)*2e-4)
    # Independent impulse-energy check of the linear noise-to-command variance.
    reach=D.copy();energy=0.
    for _ in range(4000):energy+=float((C@reach@R@reach.T@C.T).item());reach=Z@reach
    variance_disagreement=abs(force_var-energy)/max(1e-16,abs(energy))
    valid=np.isfinite(Sigma).all() and force_var>=0 and lyap_res<1e-7 and variance_disagreement<1e-3
    nominal_decrease=(P-F.T@P@F);nominal_decrease=(nominal_decrease+nominal_decrease.T)/2
    min_decrease=float(la.eigvalsh(nominal_decrease)[0])
    # Ellipsoid e'Pe <= c guarantees the stated linear LQR's local inequalities only
    # when the decrease test is valid. Absolute link angles are sum of relative q.
    constraints=[(K[0],10.,'force'),(np.eye(n)[0],2.3,'rail at goal=.1')]
    for i in range(p['poles']):
        a=np.zeros(n);a[1:i+2]=1;constraints.append((a,35*np.pi/180,f'absolute_link_{i+1}'))
    bounds=[{'constraint':name,'c':float(bound**2/(a@la.solve(P,a,assume_a='pos')))} for a,bound,name in constraints]
    c=min(v['c'] for v in bounds)
    M=np.eye(n)-L@H
    # One sample input delay, with and without the observer using the known queued command.
    delayed_unmodeled=np.block([[A,np.zeros((n,n)),B],[M@B@K,M@(A-B@K),M@B],[-K,K,np.zeros((1,1))]])
    delayed_known=np.block([[A,np.zeros((n,n)),B],[np.zeros((n,n)),M@A,np.zeros((n,1))],[-K,K,np.zeros((1,1))]])
    test_x=np.linspace(-.02,.03,n);test_e=np.linspace(.003,-.002,n);held=.2;command=float((-K@(test_x-test_e)).item())
    physical=A@test_x+B[:,0]*held;measurement=H@physical
    for matrix,known in [(delayed_unmodeled,False),(delayed_known,True)]:
        predicted=A@(test_x-test_e)+B[:,0]*(held if known else command)
        estimated=predicted+L@(measurement-H@predicted)
        expected=np.r_[physical,physical-estimated,command]
        actual=matrix@np.r_[test_x,test_e,held]
        assert np.max(np.abs(actual-expected))/max(1,np.max(np.abs(expected)))<1e-10
    row['oneSampleDelayProbe']={'equationCheckPassed':True,'seconds':.02,'unmodeledDelayRadius':float(max(abs(la.eigvals(delayed_unmodeled)))),
       'knownAppliedInputObserverRadius':float(max(abs(la.eigvals(delayed_known)))),
       'scope':'Linear diagnostic only. Updating observer input history is not redesigning the controller for delay or proving nonlinear delay robustness.'}
    row.update(controllerRadius=float(max(abs(la.eigvals(F)))),observerRadius=float(max(abs(la.eigvals(E)))),
      jointUnsaturatedRadius=float(max(abs(la.eigvals(Z)))),measurementSigma=2e-4,
      instantaneousMeasurementToForceSigma_N=immediate,
      stationaryUnsaturatedForceSigma_N=float(np.sqrt(max(0,force_var))) if valid else None,
      lyapunovResidual=float(lyap_res),impulseVarianceRelativeDisagreement=float(variance_disagreement),noiseAnalysisNumericallyVerified=bool(valid),
      declaredForceLimit_N=10.,minimumNominalLyapunovDecreaseEigenvalue=min_decrease,
      localLtiEllipsoid={'c':c,'limitingConstraint':min(bounds,key=lambda a:a['c'])['constraint'],'decreaseVerified':min_decrease>1e-8,'scope':'nominal linear unsaturated LQR only; no nonlinear, noisy, actuator or sample-time robust guarantee'})
    rows.append(row)
r={'sourceSha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'schema':'cartpole-chain-output-feedback-diagnosis/v1','rows':rows,'profileSha256':hashlib.sha256((ROOT/'assets/chains/profiles.json').read_bytes()).hexdigest(),
   'scope':'Gaussian white measurement noise only; no process disturbance or saturation in the Lyapunov calculation. Large command sigma flags breakdown of the linear operating assumption, not a guaranteed failure probability.',
   'meaning':'Stable separate controller/observer poles do not establish usable force margins or nonlinear robustness. K*L noise amplification, transient behavior and physical constraints must be examined together.'}
(ROOT/'evidence/chain_output_feedback_diagnosis.json').write_text(json.dumps(r,indent=2,allow_nan=False)+'\n');print(json.dumps(rows,indent=2))
