"""Independent verification of unchanged JS DARE output and QP using SciPy/OSQP."""
from pathlib import Path
import hashlib, json, sys, subprocess
import numpy as np
import scipy, scipy.linalg
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tests'))
from test_native_reference import solve_qp
subprocess.run(['node','tests/test_terminal.mjs'],cwd=ROOT,check=True,timeout=30)
fixture_path=ROOT/'test-results/terminal_fixture.json'
f=json.loads(fixture_path.read_text());m=json.loads((ROOT/'tests/fixtures/terminal_validation.json').read_text());t=m['numericalTolerances'];rows=[]
for case in f['cases']:
    A,B,Q,P=(np.asarray(case[k],dtype=float) for k in ['A','B','Q','Qf'])
    R=np.array([[case['R']]])
    reference=scipy.linalg.solve_discrete_are(A,B,Q,R)
    K=np.linalg.solve(R+B.T@P@B,B.T@P@A)
    residual=A.T@P@A-P-A.T@P@B@K+Q
    normalized=float(np.linalg.norm(residual,np.inf)/max(1,np.linalg.norm(P,np.inf)))
    matrix_error=float(np.max(np.abs(P-reference)))
    symmetry=float(np.max(np.abs(P-P.T)));eigen_min=float(np.min(np.linalg.eigvalsh((P+P.T)/2)))
    rho=float(np.max(np.abs(np.linalg.eigvals(A-B@K))))
    qp=solve_qp(case,rail=2.4)
    assert qp['accepted'],qp
    action_error=abs(case['U'][0]-qp['action'])
    cost_error=abs(case['J']-qp['J'])/max(1,abs(qp['J']))
    passed=matrix_error<=t['matrixMaxAbsoluteError'] and normalized<=t['normalizedDareResidual'] and symmetry<1e-8 and eigen_min>0 and rho<1 and action_error<=t['firstActionError'] and cost_error<=t['relativeCost']
    rows.append({'id':case['id'],'scipyP':reference.tolist(),'matrixMaxAbsoluteError':matrix_error,'normalizedDareResidual':normalized,'symmetryError':symmetry,'minimumEigenvalue':eigen_min,'unconstrainedClosedLoopSpectralRadius':rho,'firstActionError':action_error,'relativeCostError':cost_error,'nativeQpStatus':qp['status'],'passed':bool(passed)})
out={'schema':'cartpole-terminal-reference/v1','fixtureSha256':hashlib.sha256(fixture_path.read_bytes()).hexdigest(),'scipyVersion':scipy.__version__,'sourceSha256':f['sourceSha256'],'cases':rows,'passed':all(r['passed'] for r in rows),'scope':'Numerical DARE/QP parity; unconstrained linear spectral radius is not a constrained nonlinear stability certificate'}
(ROOT/'evidence/terminal_reference.json').write_text(json.dumps(out,indent=2,allow_nan=False)+'\n')
print(json.dumps({'passed':out['passed'],'cases':[{k:v for k,v in r.items() if k!='scipyP'} for r in rows]},indent=2))
assert out['passed']
