"""Independent feasibility-only HiGHS witness for the declared linear MPC constraints.
Reuses the existing sparse state/input equations; objective zero is NOT a controller.
No LP witness is applied to MuJoCo or substituted for a rejected QP action.
"""
from pathlib import Path
import hashlib,json
import numpy as np
from scipy.optimize import linprog
from chain_native import ChainQP
ROOT=Path(__file__).resolve().parents[1]
profiles=json.loads((ROOT/'assets/chains/profiles.json').read_text())['profiles']
manifest=json.loads((ROOT/'tests/fixtures/chain_validation.json').read_text())

def witness(profile,world,goal):
    x=np.asarray(world,dtype=float)
    if x.shape!=(profile['nx'],) or not np.isfinite(x).all() or not np.isfinite(goal):raise ValueError('Invalid initial state')
    q=ChainQP(profile,goal);e=x.copy();e[0]-=goal
    q.lower[:q.n]=-e;q.upper[:q.n]=-e
    result=linprog(np.zeros(q.nodes+q.N),A_eq=q.G[:q.nodes],b_eq=q.lower[:q.nodes],
        bounds=list(zip(q.lower[q.nodes:],q.upper[q.nodes:])),method='highs',
        options={'time_limit':2.,'primal_feasibility_tolerance':1e-9,'dual_feasibility_tolerance':1e-9})
    out={'solverStatus':int(result.status),'message':result.message,'witnessVerified':False,'appliedToPlant':False}
    if result.success and result.x is not None and np.isfinite(result.x).all():
        z=result.x;gx=q.G@z
        violation=float(max(0,np.max(q.lower-gx),np.max(gx-q.upper)))
        X=z[:q.nodes].reshape(q.N+1,q.n);U=z[q.nodes:]
        # Directly reconstruct original dynamics in physical state/input coordinates.
        defect=float(np.max(np.abs(X[1:]-X[:-1]@q.A.T-U[:,None]*q.B[:,0])))
        absolute=np.cumsum(X[:,1:profile['dof']],axis=1)
        out.update(witnessVerified=bool(max(violation,defect)<=1e-7),maxConstraintViolation=violation,
            maxDynamicsDefect=defect,maxLinearAbsoluteLinkAngle_rad=float(np.max(np.abs(absolute))),
            maxForce_N=float(np.max(np.abs(U))),maxWorldPosition_m=float(np.max(np.abs(X[:,0]+goal))))
    return out

rows=[]
for p in profiles:
    if not p['designAvailable']:continue
    for case in manifest['cases']:
        a=np.array([case['absoluteAngle']*(-.5 if i%2 else 1) for i in range(p['poles'])])
        x=np.r_[case['cart'],a[0],np.diff(a),np.zeros(p['dof'])]
        rows.append({'poles':p['poles'],'case':case['name'],'initialAuthority':'fixed oracle initial state, not noisy estimate',**witness(p,x,case['goal'])})
# Controls remain physically constrained; independently verify a simple admitted point
# and a point whose fixed initial world coordinate contradicts the hard rail.
p=profiles[0];assert witness(p,np.zeros(p['nx']),0)['witnessVerified']
x=np.zeros(p['nx']);x[0]=2.5;outside=witness(p,x,0);assert outside['solverStatus']==2 and not outside['witnessVerified']
try:witness(p,np.full(p['nx'],np.nan),0);raise AssertionError('NaN state accepted')
except ValueError:pass
report={'schema':'cartpole-chain-feasibility-diagnosis/v1','rows':rows,'outsideRailRejected':True,
    'profileSha256':hashlib.sha256((ROOT/'assets/chains/profiles.json').read_bytes()).hexdigest(),
    'manifestSha256':hashlib.sha256((ROOT/'tests/fixtures/chain_validation.json').read_bytes()).hexdigest(),
    'sourceSha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    'scope':'Feasibility-only LP for the same fixed linear horizon, input bound and cart rail. No angle or terminal-region constraint exists in that original QP, so a witness may predict large angles. Neither nonlinear validity, stability, tracking success nor optimality follows. An unverified LP result is not a global physical impossibility proof.'}
(ROOT/'evidence/chain_feasibility_diagnosis.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
print(json.dumps(rows,indent=2))
