"""Dimension-generic OSQP reference using its official sparse MPC formulation.
Optional offline control, not a browser/native bridge service or hardware controller.
"""
import numpy as np
from scipy import sparse
import osqp,time

class ChainQP:
    def __init__(self,p,goal):
        self.p=p;self.goal=goal;self.n=p['nx'];self.N=p['horizon'];self.nodes=(self.N+1)*self.n
        self.A=np.asarray(p['A']);self.B=np.asarray(p['B']);self.Q=np.asarray(p['Q']);self.Pf=np.asarray(p['P']);self.R=p['R']
        n,N=self.n,self.N;nx=self.nodes
        self.H=2*sparse.block_diag([sparse.kron(sparse.eye(N),self.Q),self.Pf,self.R*sparse.eye(N)],format='csc')
        Ax=sparse.kron(sparse.eye(N+1),-sparse.eye(n))+sparse.kron(sparse.eye(N+1,k=-1),self.A)
        Bu=sparse.kron(sparse.vstack([sparse.csc_matrix((1,N)),sparse.eye(N)]),self.B)
        self.G=sparse.vstack([sparse.hstack([Ax,Bu]),sparse.eye(nx+N)],format='csc')
        lo=np.full(nx+N,-np.inf);hi=np.full(nx+N,np.inf)
        lo[nx:]=-p['forceLimit'];hi[nx:]=p['forceLimit'];lo[:nx:n]=-p['railLimit']-goal;hi[:nx:n]=p['railLimit']-goal
        self.lower=np.r_[np.zeros(nx),lo];self.upper=np.r_[np.zeros(nx),hi]
        self.s=osqp.OSQP();self.s.setup(P=sparse.triu(self.H,format='csc'),q=np.zeros(nx+N),A=self.G,l=self.lower,u=self.upper,verbose=False,eps_abs=1e-8,eps_rel=1e-8,max_iter=20000,polishing=True)
    def solve(self,x):
        start=time.perf_counter();x=np.asarray(x,dtype=float)
        if x.shape!=(self.n,) or not np.isfinite(x).all() or abs(x[0])>self.p['railLimit']+1e-10:return {'accepted':False,'action':None,'status':'invalid-or-outside-state'}
        e=x.copy();e[0]-=self.goal;self.lower[:self.n]=-e;self.upper[:self.n]=-e
        self.s.update(l=self.lower,u=self.upper);r=self.s.solve(raise_error=False)
        out={'accepted':False,'action':None,'status':r.info.status,'iterations':r.info.iter,'hostMs':1000*(time.perf_counter()-start)}
        if r.info.status!='solved' or r.x is None or not np.isfinite(r.x).all():return out
        gz=self.G@r.x;violation=float(max(0,np.max(self.lower-gz),np.max(gz-self.upper)))
        station=self.H@r.x+self.G.T@r.y
        normalized=float(np.max(np.abs(station))/max(1.,np.max(np.abs(self.H@r.x)),np.max(np.abs(self.G.T@r.y))))
        accepted=violation<=1e-7 and normalized<=1e-7
        out.update(accepted=bool(accepted),action=float(r.x[self.nodes]) if accepted else None,primalViolation=violation,normalizedStationarity=normalized,cost=float(.5*r.x@(self.H@r.x)),inputSequence=r.x[self.nodes:].tolist())
        return out
