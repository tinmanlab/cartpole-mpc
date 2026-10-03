"""Independent numerical cross-checks for the control/observer core.

Requires NumPy and SciPy. It does not participate in CI; it writes an audit
receipt that can be regenerated on a scientific Python environment.
"""
from __future__ import annotations
import json, subprocess
from pathlib import Path
import numpy as np
from scipy.linalg import solve_discrete_are, eigvals
from scipy.optimize import minimize

ROOT=Path(__file__).resolve().parents[1]
node=r"""
const L=require('./src/engine'),P=require('./src/plant');
const spec={...P.DEFAULT_SPEC,actuator:'ideal',force:10,friction:0};
const m=L.linearModel(spec), ctl=new L.LQRController(spec);
const kf=new L.KFObserver(spec,{R:[.008**2,.004**2]});
kf.reset([.1,.05]); kf.step([.11,.045],.2);

const lm=new L.LinearMPCController(spec),xlm=[.3,.1,.08,-.05];
lm.reset();lm.act(xlm,0);
const fn=new L.FullNMPCController(spec),xfn=[.2,.05,.07,-.04];
fn.reset();fn.act(xfn,0);

const plant=new L.LabPlant({seed:1,scenario:'nominal',spec});
plant.reset([0,.1,.05,-.02]);
const cm=new L.CentroidalMPCController(plant.spec),z0=cm.reducedState(plant.s,0),ucom=2;
const out=plant.step(ucom),z1=cm.reducedState(out.state,0);

console.log(JSON.stringify({
 A:m.A,B:m.B,K:ctl.K,
 kf:{x:kf.x,P:kf.P,K:kf.last.K,innovation:kf.last.innovation},
 linearMpc:{x:xlm,U:lm.lastControls,cost:lm.lastCost,limit:lm.limit,N:lm.N},
 fullNmpc:{x:xfn,U:fn.lastControls,cost:fn.lastCost,limit:fn.limit,N:fn.N,iterations:fn.lastIterations},
 com:{z0,z1,u:ucom,mass:spec.mc+spec.mp,dt:L.DT}
}));
"""
data=json.loads(subprocess.check_output(["node","-e",node],cwd=ROOT,text=True))
A=np.asarray(data["A"],float)
B=np.asarray(data["B"],float)

# LQR / DARE
Q=np.diag([2,.5,55,3])
Rctl=np.array([[.12]])
Pdare=solve_discrete_are(A,B,Q,Rctl)
Kref=np.linalg.solve(Rctl+B.T@Pdare@B,B.T@Pdare@A).ravel()
Kjs=np.asarray(data["K"],float)

# Observability
H=np.array([[1,0,0,0],[0,0,1,0]],float)
O=np.vstack([H,H@A,H@A@A,H@A@A@A])
C=np.hstack([B,A@B,A@A@B,A@A@A@B])

# One KF predict/update, independently in NumPy
x0=np.array([.1,0,.05,0.])
P0=np.diag([.1,1,.04,1.])
Qk=np.diag([2e-5,2e-3,2e-5,4e-3])
Rk=np.diag([.008**2,.004**2])
u=.2
y=np.array([.11,.045])
xm=A@x0+B[:,0]*u
Pm=A@P0@A.T+Qk
innov=y-H@xm
S=H@Pm@H.T+Rk
Kgain=Pm@H.T@np.linalg.inv(S)
xp=xm+Kgain@innov
I=np.eye(4)
Pp=(I-Kgain@H)@Pm@(I-Kgain@H).T+Kgain@Rk@Kgain.T

# Linear MPC optimum, independently with SciPy L-BFGS-B.
lm=data["linearMpc"]
xlin=np.asarray(lm["x"],float)
Qm=np.diag([2,.45,68,3.5])
Qfm=np.diag([8,1.5,110,7])
Rm=.14
N=int(lm["N"])
def rollout(U):
    xs=[xlin]
    for uk in U:
        xs.append(A@xs[-1]+B[:,0]*uk)
    return np.asarray(xs)
def mpc_cost(U):
    X=rollout(U)
    return float(sum(X[k]@Qm@X[k]+Rm*U[k]*U[k] for k in range(N)) + X[-1]@Qfm@X[-1])
opt=minimize(mpc_cost,np.zeros(N),method="L-BFGS-B",bounds=[(-lm["limit"],lm["limit"])]*N,
             options={"ftol":1e-13,"gtol":1e-10,"maxiter":2000})
Ujs=np.asarray(lm["U"],float)

# Full nonlinear NMPC optimum, independently with the same ideal CartPole equations.
fn=data["fullNmpc"]
mc,mp,l,g=1.0,.1,.5,9.8
def wrap_py(a): return (a+np.pi)%(2*np.pi)-np.pi
def substep_py(state,u,dt=.005):
    x,v,th,w=state; c=np.cos(th); sn=np.sin(th); D=mc+mp-.75*mp*c*c
    rhs=mp*l*w*w*sn-.75*mp*g*sn*c
    nv=(D*v+dt*(u+rhs))/D
    acc=(nv-v)/dt
    alpha=(g*sn-c*acc)/(l*4/3)
    nw=w+alpha*dt
    return np.array([x+nv*dt,nv,wrap_py(th+nw*dt),nw])
def step_py(state,u):
    x=np.asarray(state,float).copy()
    for _ in range(4): x=substep_py(x,u)
    return x
Qn=np.diag([8,1,80,4]);Qfn=np.diag([24,3,130,8]);Rn=.12
xfn=np.asarray(fn["x"],float);Nn=int(fn["N"])
def full_cost(U):
    x=xfn.copy(); J=0.0
    for uk in U:
        J += float(x@Qn@x + Rn*uk*uk); x=step_py(x,uk)
    return J + float(x@Qfn@x)
opt_fn=minimize(full_cost,np.zeros(Nn),method="L-BFGS-B",bounds=[(-fn["limit"],fn["limit"])]*Nn,
                options={"ftol":1e-12,"gtol":1e-8,"maxiter":1000})

# CoM reduced physics relation: finite-difference c_dot acceleration vs F/M.
com=data["com"]
com_acc=(com["z1"][1]-com["z0"][1])/com["dt"]
com_ref=com["u"]/com["mass"]

result={
 "schema":"cartpole-mpc-algorithm-audit/v2",
 "lqr":{
   "max_abs_K_error_vs_scipy":float(np.max(np.abs(Kref-Kjs))),
   "K_js":Kjs.tolist(),
   "K_scipy":Kref.tolist(),
   "closed_loop_eigenvalues":[[float(z.real),float(z.imag)] for z in eigvals(A-B@Kref[None,:])],
 },
 "observability":{"rank":int(np.linalg.matrix_rank(O)),"state_dim":4},
 "controllability":{"rank":int(np.linalg.matrix_rank(C)),"state_dim":4},
 "kf_one_step":{
   "max_abs_state_error_vs_numpy":float(np.max(np.abs(xp-np.asarray(data["kf"]["x"])))),
   "max_abs_covariance_error_vs_numpy":float(np.max(np.abs(Pp-np.asarray(data["kf"]["P"])))),
   "max_abs_gain_error_vs_numpy":float(np.max(np.abs(Kgain-np.asarray(data["kf"]["K"])))),
   "max_abs_innovation_error_vs_numpy":float(np.max(np.abs(innov-np.asarray(data["kf"]["innovation"])))),
 },
 "linear_mpc":{
   "scipy_success":bool(opt.success),
   "js_cost":float(lm["cost"]),
   "scipy_optimum_cost":float(opt.fun),
   "relative_cost_gap":float((lm["cost"]-opt.fun)/max(1.0,abs(opt.fun))),
   "first_action_js":float(Ujs[0]),
   "first_action_scipy":float(opt.x[0]),
   "first_action_abs_error":float(abs(Ujs[0]-opt.x[0])),
 },
 "full_nmpc":{
   "scipy_success":bool(opt_fn.success),
   "js_cost":float(fn["cost"]),
   "scipy_optimum_cost":float(opt_fn.fun),
   "relative_cost_gap":float((fn["cost"]-opt_fn.fun)/max(1.0,abs(opt_fn.fun))),
   "first_action_js":float(fn["U"][0]),
   "first_action_scipy":float(opt_fn.x[0]),
   "first_action_abs_error":float(abs(fn["U"][0]-opt_fn.x[0])),
   "js_iterations":int(fn["iterations"]),
 },
 "centroidal_reduced_model":{
   "finite_difference_com_accel":float(com_acc),
   "force_over_mass":float(com_ref),
   "abs_error":float(abs(com_acc-com_ref)),
 },
}
out=ROOT/"evidence"/"algorithm_audit.json"
out.write_text(json.dumps(result,indent=2),encoding="utf-8")
print(json.dumps(result,indent=2))
