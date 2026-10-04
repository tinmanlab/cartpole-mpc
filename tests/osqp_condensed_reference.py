"""Offline independent OSQP transcription; no hardware actions.
Eliminate linear state equalities and normalize constraint rows without changing
the original physical feasibility and first-action acceptance tolerances.
"""
import numpy as np
from scipy import sparse
import osqp

def solve_active_rail(c, rail):
    A,B,Q,Qf=(np.asarray(c[k],dtype=float) for k in ['A','B','Q','Qf'])
    x0=np.asarray(c['x'],dtype=float);n=len(x0);N=int(c['N'])
    if not np.isfinite(x0).all() or abs(x0[0])>rail:
        return {'accepted':False,'status':'invalid_or_infeasible_initial_state','action':None}
    F=np.vstack([np.linalg.matrix_power(A,k) for k in range(N+1)])
    G=np.zeros(((N+1)*n,N))
    for k in range(1,N+1):
        for j in range(k):G[k*n:(k+1)*n,j]=(np.linalg.matrix_power(A,k-j-1)@B).ravel()
    W=sparse.block_diag([Q]*N+[Qf],format='csc');offset=F@x0
    H=2*(G.T@W@G+c['R']*np.eye(N));linear=2*G.T@(W@offset)
    C=np.vstack([np.eye(N),G[n::n,:]]);position=offset[n::n]
    lo=np.r_[np.full(N,-c['limit']),-rail-position];hi=np.r_[np.full(N,c['limit']),rail-position]
    scales=np.linalg.norm(C,axis=1)
    solver=osqp.OSQP();solver.setup(P=sparse.csc_matrix(H),q=linear,
        A=sparse.csc_matrix(C/scales[:,None]),l=lo/scales,u=hi/scales,
        eps_abs=1e-9,eps_rel=1e-9,max_iter=200000,adaptive_rho_interval=25,
        adaptive_rho_tolerance=2,polishing=True,verbose=False)
    r=solver.solve(raise_error=False)
    out={'status':r.info.status,'accepted':False,'action':None,'iterations':int(r.info.iter),
         'transcription':'independent condensed states; unit-norm constraint rows'}
    if r.info.status!='solved' or r.x is None or not np.isfinite(r.x).all():return out
    U=r.x;X=offset+G@U;physical=C@U;dual=r.y/scales
    primal=float(max(0,np.max(lo-physical),np.max(physical-hi)))
    stationarity=float(np.max(np.abs(H@U+linear+C.T@dual)))
    out.update(primal_residual=primal,dual_residual=stationarity,
        J=float(X@(W@X)+c['R']*(U@U)),U=U.tolist(),max_position=float(np.max(np.abs(X[::n]))))
    out['accepted']=primal<1e-6 and stationarity<1e-6
    if out['accepted']:out['action']=float(U[0])
    return out
