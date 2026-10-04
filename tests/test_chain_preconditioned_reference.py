"""Independent stable-coordinate transcription, OSQP solve and original-cost checks.
The browser optimizer remains quadprog. This test does not relax direct-QP guards.
"""
import hashlib,json,sys
from pathlib import Path
import numpy as np
from scipy import linalg as la,sparse
import osqp
ROOT=Path(__file__).resolve().parents[1]
f=json.loads((ROOT/'test-results/chain_preconditioned_fixture.json').read_text());rows=[]
for c in f['cases']:
    p=c['profile'];n=p['nx'];N=p['horizon'];A,B,Q,P,K=[np.array(p[k],float) for k in ['A','B','Q','P','K']];R=p['R'];F=A-B@K
    e=np.array(c['x0']);e[0]-=c['goal'];z=e.copy();G=np.zeros((n,N));H=np.zeros((N,N));g=np.zeros(N);constant=0.;constraints=[];lower=[];gx=[];gu=[];xo=[];uo=[]
    # Independent NumPy transcription using physical x and u maps, not JS exported H.
    for k in range(N+1):
        W=P if k==N else Q;H+=2*G.T@W@G;g+=2*G.T@W@z;constant+=float(z@W@z);gx.append(G.copy());xo.append(z.copy())
        if k<N:
            h=np.eye(N)[k]-K@G;v0=-float((K@z).item());gu.append(h.ravel());uo.append(v0)
            H+=2*R*(h.T@h);g+=2*R*h.ravel()*v0;constant+=R*v0*v0
            G=F@G;G[:,k]+=B[:,0];z=F@z
    H=(H+H.T)/2
    for sign in [1,-1]:
        for k in range(N):constraints.append(sign*gu[k]);lower.append(-p['forceLimit']-sign*uo[k])
    for k in range(1,N+1):constraints.extend([gx[k][0],-gx[k][0]]);lower.extend([-p['railLimit']-xo[k][0]-c['goal'],xo[k][0]+c['goal']-p['railLimit']])
    C=np.array(constraints);lo=np.array(lower);js=c['condensed']
    herr=float(np.max(np.abs(H-np.array(js['H'])))/max(1,np.max(np.abs(H))));gerr=float(np.max(np.abs(g-js['f']))/max(1,np.max(np.abs(g))))
    assert herr<1e-7 and gerr<1e-7,(p['poles'],herr,gerr)
    np.testing.assert_allclose(C,js['columns'],rtol=1e-8,atol=1e-8);np.testing.assert_allclose(lo,js['lower'],rtol=1e-8,atol=1e-8)
    solver=osqp.OSQP();solver.setup(P=sparse.triu(sparse.csc_matrix(H),format='csc'),q=g,A=sparse.csc_matrix(C),l=lo,u=np.full(len(lo),np.inf),eps_abs=1e-10,eps_rel=1e-10,max_iter=20000,polishing=True,verbose=False)
    r=solver.solve(raise_error=False);assert r.info.status=='solved',(p['poles'],r.info.status)
    V=r.x;U=np.array(uo)+np.array(gu)@V
    X=np.array([xo[k]+gx[k]@V for k in range(N+1)])
    cost=sum(float(X[k]@Q@X[k]+R*U[k]**2) for k in range(N))+float(X[-1]@P@X[-1])
    violation=float(max(0,np.max(lo-C@V),np.max(np.abs(U))-p['forceLimit'],np.max(np.abs(X[:,0]+c['goal']))-p['railLimit']))
    action=abs(U[0]-c['result']['U'][0]);gap=abs(cost-c['result']['J'])/max(1,abs(cost))
    assert violation<1e-8 and action<.001 and gap<1e-4,(p['poles'],violation,action,gap)
    # Original open-loop condensed Hessian, used only for conditioning diagnostics.
    G0=np.zeros((n,N));Hu=np.eye(N)*2*R
    for k in range(N+1):
        Hu+=2*G0.T@(P if k==N else Q)@G0
        if k<N:G0=A@G0;G0[:,k]+=B[:,0]
    rows.append({'label':c.get('label','fixed-nominal'),'poles':p['poles'],'stableHessianCondition':float(np.linalg.cond(H)),'directHessianCondition':float(np.linalg.cond(Hu)),
      'hessianRelativeError':herr,'gradientRelativeError':gerr,'firstActionError':float(action),'relativeCostError':float(gap),'physicalViolation':violation,
      'osqpIterations':int(r.info.iter),'osqpStatus':r.info.status,'qpPrimalResidual':float(r.info.prim_res),'qpDualResidual':float(r.info.dual_res),'passed':True})
report={'schema':'cartpole-preconditioned-reference/v1','fixtureSha256':hashlib.sha256((ROOT/'test-results/chain_preconditioned_fixture.json').read_bytes()).hexdigest(),'rows':rows,'passed':True,
 'scope':'Identical model/Q/R/P/horizon and physical bounds. Full-cost transcription, including finite-precision Riccati residual effects. Transformed absolute and reconstructed physical residuals have separate documented meanings.'}
(ROOT/'evidence/chain_preconditioned_reference.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n');print(json.dumps(rows,indent=2))
