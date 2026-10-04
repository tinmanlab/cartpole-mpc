"""Optional native acados/HPIPM research adapter, NOT a browser/hardware controller.

Formulation follows acados v0.6.0 getting_started and discrete-dynamics examples.
The small symbolic adapter expresses this repository's uniform-rod transition;
its value and derivatives must pass MuJoCo WASM differential checks before use.
"""
from __future__ import annotations
import os, time, xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import casadi as ca
from scipy.linalg import solve_discrete_are
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
ROOT=Path(__file__).resolve().parents[1]
N=30; DT=.02; R=.14
Q=np.array([2.,.45,68.,3.5]); QF=np.array([8.,1.5,110.,7.])
TOL={'dynamics':1e-5,'constraint':1e-7,'initial':1e-8,'cost':1e-6}
UPSTREAM='503364817c872d474ab5bed219c26760ac267769'

def read_model(path:Path)->dict:
    """Reject incompatible assets rather than pretending this is a general MJCF parser."""
    e=ET.parse(path).getroot();cart=e.find('.//body[@name="cart"]');pole=e.find('.//body[@name="pole"]')
    if cart is None or pole is None:raise ValueError('Missing canonical bodies')
    cj=cart.find('joint');pj=pole.find('joint');motor=e.find('actuator/motor')
    if cj.get('type')!='slide' or cj.get('axis')!='1 0 0' or pj.get('axis')!='0 1 0' or motor.get('gear')!='1':
        raise ValueError('Unsupported coordinate or actuation contract')
    if len(e.findall('.//joint[@name]'))!=2 or len(e.find('actuator'))!=1:raise ValueError('Unsupported DOF/actuator count')
    ci=cart.find('inertial');pi=pole.find('inertial');ipos=np.fromstring(pi.get('pos'),sep=' ');inertia=np.fromstring(pi.get('diaginertia'),sep=' ')
    p={'mc':float(ci.get('mass')),'mp':float(pi.get('mass')),'l':float(ipos[2]),'inertia':float(inertia[1]),'g':-float(e.find('option').get('gravity').split()[2]),'friction':0.}
    if not all(np.isfinite(v) and v>0 for k,v in p.items() if k!='friction'):raise ValueError('Invalid physical parameters')
    if not np.allclose(ipos[:2],0) or not np.isclose(p['inertia'],p['mp']*p['l']**2/3,rtol=0,atol=1e-12):raise ValueError('Not the admitted uniform-rod model')
    if e.find('option').get('integrator')!='Euler' or float(e.find('option').get('timestep'))!=.005:raise ValueError('Unsupported integration contract')
    return p

def discrete_model(p:dict):
    x=ca.SX.sym('x',4);u=ca.SX.sym('u',1);z=x
    # Nominal ideal force model. 4 x 5ms semi-implicit MuJoCo Euler transition.
    # l is COM distance, with I_COM=mp*l^2/3; NOT the upstream point-mass pendulum.
    for _ in range(4):
        pos,v,theta,w=ca.vertsplit(z);sn=ca.sin(theta);co=ca.cos(theta);h=DT/4
        den=p['mc']+p['mp']-.75*p['mp']*co*co
        force=u+p['mp']*p['l']*w*w*sn-.75*p['mp']*p['g']*sn*co
        vn=(den*v+h*force)/(den+h*p['friction'])
        acc=(vn-v)/h;wn=w+h*(p['g']*sn-co*acc)/(p['l']*4/3)
        z=ca.vertcat(pos+h*vn,vn,theta+h*wn,wn)
    return ca.Function('uniform_rod_step',[x,u],[z]),ca.Function('uniform_rod_A',[x,u],[ca.jacobian(z,x)]),ca.Function('uniform_rod_B',[x,u],[ca.jacobian(z,u)])

def audit_plan(X,U,x0,goal,f,terminal=None)->dict:
    X=np.asarray(X,dtype=float);U=np.asarray(U,dtype=float)
    bad={'accepted':False,'reason':'invalid-plan','action':None}
    if X.shape!=(N+1,4) or U.shape!=(N,1) or not np.isfinite(X).all() or not np.isfinite(U).all():return bad
    defects=[];rollout=[np.asarray(x0,dtype=float)]
    for k in range(N):
        defects.append(np.max(np.abs(np.asarray(f(X[k],U[k])).ravel()-X[k+1])))
        rollout.append(np.asarray(f(rollout[-1],U[k])).ravel())
    rollout=np.asarray(rollout)
    if not np.isfinite(rollout).all():return bad
    init=float(np.max(np.abs(X[0]-x0)));defect=float(max(defects))
    violation=float(max(0,np.max(np.abs(X[:,0]))-2.4,np.max(np.abs(U))-10,np.max(np.abs(rollout[:,0]))-2.4))
    errors=X-np.array([goal,0,0,0]);P=np.diag(QF) if terminal is None else terminal
    cost=float(np.sum(errors[:-1]**2*Q)+R*np.sum(U**2)+errors[-1]@P@errors[-1])
    ok=init<=TOL['initial'] and defect<=TOL['dynamics'] and violation<=TOL['constraint']
    return {'accepted':bool(ok),'reason':'admitted-nominal-plan' if ok else 'nonlinear-plan-rejected','action':float(U[0,0]) if ok else None,'max_dynamics_defect':defect,'initial_error':init,'constraint_violation':violation,'computed_cost':cost}

class NativeNMPC:
    def __init__(self,algorithm='SQP',terminal='original'):
        if algorithm not in ('SQP','SQP_RTI'):raise ValueError('Unsupported algorithm')
        source=Path(os.environ.get('ACADOS_SOURCE_DIR',ROOT/'test-results/acados-v0.6.0')).resolve()
        os.environ['ACADOS_SOURCE_DIR']=str(source)
        self.algorithm=algorithm;self.f,self.A,self.B=discrete_model(read_model(ROOT/'assets/cartpole.xml'))
        if terminal not in ('original','dare'):raise ValueError('Unknown terminal design')
        self.terminal_kind=terminal
        self.terminal=np.diag(QF) if terminal=='original' else solve_discrete_are(np.asarray(self.A(np.zeros(4),0)),np.asarray(self.B(np.zeros(4),0)),np.diag(Q),np.array([[R]]))
        model=AcadosModel();model.name='cartpole_uniform_'+algorithm.lower()+'_'+terminal;model.x=ca.SX.sym('x',4);model.u=ca.SX.sym('u',1)
        model.disc_dyn_expr=self.f(model.x,model.u)
        ocp=AcadosOcp();ocp.name=model.name;ocp.model=model
        opts=ocp.solver_options;opts.N_horizon=N;opts.tf=N*DT;opts.integrator_type='DISCRETE'
        opts.qp_solver='PARTIAL_CONDENSING_HPIPM';opts.qp_solver_cond_N=10;opts.hessian_approx='GAUSS_NEWTON';opts.nlp_solver_type=algorithm
        opts.nlp_solver_max_iter=100;opts.qp_solver_iter_max=100;opts.tol=1e-7
        opts.cost_scaling=np.ones(N+1)  # Match the lab's SUM, not dt-scaled integral.
        if algorithm=='SQP':opts.globalization='MERIT_BACKTRACKING'
        ocp.cost.cost_type='LINEAR_LS';ocp.cost.cost_type_e='LINEAR_LS'
        ocp.cost.W=np.diag(np.r_[2*Q,2*R]);ocp.cost.W_e=2*self.terminal
        ocp.cost.Vx=np.vstack([np.eye(4),np.zeros((1,4))]);ocp.cost.Vu=np.r_[np.zeros(4),1].reshape(5,1);ocp.cost.Vx_e=np.eye(4)
        ocp.cost.yref=np.zeros(5);ocp.cost.yref_e=np.zeros(4)
        ocp.constraints.idxbu=np.array([0]);ocp.constraints.lbu=np.array([-10.]);ocp.constraints.ubu=np.array([10.])
        ocp.constraints.idxbx=np.array([0]);ocp.constraints.lbx=np.array([-2.4]);ocp.constraints.ubx=np.array([2.4])
        ocp.constraints.idxbx_e=np.array([0]);ocp.constraints.lbx_e=np.array([-2.4]);ocp.constraints.ubx_e=np.array([2.4]);ocp.constraints.x0=np.zeros(4)
        ocp.code_gen_options.code_export_directory=str(ROOT/'test-results'/('codegen_'+model.name))
        self.solver=AcadosOcpSolver(ocp,verbose=False);self.previous=None
    def reset(self):
        self.solver.reset();self.previous=None
    def solve(self,x0,goal=0)->dict:
        start=time.perf_counter();x0=np.asarray(x0,dtype=float)
        if x0.shape!=(4,) or not np.isfinite(x0).all() or not np.isfinite(goal) or abs(x0[0])>2.4:
            return {'accepted':False,'action':None,'reason':'invalid-or-outside-initial-state','native_ms':0,'host_ms':1000*(time.perf_counter()-start)}
        s=self.solver;U=np.zeros((N,1)) if self.previous is None else np.vstack([self.previous[1:],self.previous[-1:]])
        z=x0.copy()
        # Re-roll shifted controls from the current estimate, not the old measured state.
        for k in range(N):s.set(k,'x',z);s.set(k,'u',U[k]);z=np.asarray(self.f(z,U[k])).ravel()
        s.set(N,'x',z)
        yref=np.array([goal,0,0,0,0],dtype=float)
        for k in range(N):s.cost_set(k,'yref',yref)
        s.cost_set(N,'yref',yref[:4]);s.constraints_set(0,'lbx',x0);s.constraints_set(0,'ubx',x0)
        status=int(s.solve());native_ms=1000*float(s.get_stats('time_tot'))
        residuals=np.asarray(s.get_residuals(recompute=True),dtype=float).ravel()
        X=np.array([s.get(k,'x') for k in range(N+1)]);U=np.array([s.get(k,'u') for k in range(N)])
        result=audit_plan(X,U,x0,goal,self.f,self.terminal);cost=float(s.get_cost())
        cost_error=abs(cost-result.get('computed_cost',np.inf))
        result.update(status=status,native_ms=native_ms,host_ms=1000*(time.perf_counter()-start),algorithm=self.algorithm,residuals=residuals.tolist(),iterations=int(s.get_stats('sqp_iter')),cost_error=cost_error)
        ok=status==0 and result['accepted'] and np.isfinite(residuals).all() and cost_error<=TOL['cost']
        if self.algorithm=='SQP':ok=ok and float(np.max(residuals))<1e-5
        if not ok:result.update(accepted=False,action=None,reason='solver-status-or-plan-gate');self.previous=None
        else:self.previous=U.copy()
        result['nonlinearOptimalityConverged']=bool(np.isfinite(residuals).all() and float(np.max(residuals))<1e-5)
        return result

    def solve_with_recovery(self,x0,goal,backup):
        # Same current estimate/model/constraints. Never apply the rejected RTI plan.
        start=time.perf_counter();primary=self.solve(x0,goal)
        if primary['accepted']:
            return {**primary,'fallbackUsed':False,'solverAlgorithmApplied':self.algorithm}
        result=backup.solve(x0,goal)
        if result['accepted']:self.previous=backup.previous.copy()
        return {**result,'native_ms':primary['native_ms']+result['native_ms'],
            'host_ms':1000*(time.perf_counter()-start),'fallbackUsed':True,
            'solverAlgorithmApplied':'SQP' if result['accepted'] else None,
            'primaryStatus':primary.get('status'),'primaryDefect':primary.get('max_dynamics_defect'),
            'primaryReason':primary['reason']}
