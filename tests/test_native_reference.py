"""Independent numerical authorities for the CartPole teaching implementation.
Run with .venv-native/bin/python tests/test_native_reference.py.
--require-browser-parity also rejects approximate browser QP plans outside tolerance.
No hardware interface or automatic application of controls is provided.
"""
from __future__ import annotations
import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
from scipy import linalg, sparse, stats
import osqp
import mujoco

ROOT = Path(__file__).resolve().parents[1]
# Fixed BEFORE measurement; never tune these tolerances to pass a receipt.
TOL = dict(dare=1e-7, acceleration=1e-9, feasibility=1e-6,
           relative_cost_gap=1e-4, first_action_gap=1e-3)


def fixture() -> dict:
    code = r"""
const L=require('./src/engine'),P=require('./src/plant');
const s=new L.LabPlant().spec,m=L.linearModel(s),l=new L.LQRController(s);
const cases=[[.3,.1,.08,-.05],[0,0,.3,0],[1.8,.2,-.05,.1],[-.6,-.8,-.12,.4]].map(x=>{
 const c=new L.LinearMPCController(s);c.reset();c.act(x,0);
 return {x,A:c.A,B:c.B,Q:c.Q,Qf:c.Qf,R:c.R,N:c.N,limit:c.limit,
 U:c.lastControls,J:c.lastCost,converged:c.lastConverged};});
const dynamics=[];
for(const p of [{mc:1,mp:.1,l:.5},{mc:1.25,mp:.08,l:.575},{mc:.8,mp:.14,l:.42}]){
 const spec={...s,...p};
 for(const x of [[0,0,0,0],[.2,.5,.4,-.7],[-.3,-.4,-.6,.8],[0,.8,1.2,1.4]])for(const u of [-7,0,5]){
  const r=P.integrate(x,spec,{...p,friction:0},u,0,.005);
  dynamics.push({spec,x,u,acc:[r.drive.acc,r.drive.alpha]});}}
const controls=Array.from({length:40},(_,k)=>.5*Math.sin(.17*k)),x0=[0,0,.03,0];
const replay=[1,4,16,64].map(n=>{let x=x0.slice();return {n,X:controls.map(u=>{x=L.nonlinearStepSubsteps(x,u,s,null,n);return x;})};});
console.log(JSON.stringify({spec:s,A:m.A,B:m.B,K:l.K,cases,dynamics,controls,x0,replay}));
"""
    result = subprocess.run(['node', '-e', code], cwd=ROOT, capture_output=True,
                            text=True, check=True, timeout=30)
    return json.loads(result.stdout)


def solve_qp(c: dict, rail: float | None = None) -> dict:
    """OSQP sparse state/input transcription, following the official MPC example.

    Variables are x_0..x_N,u_0..u_(N-1); cost matches JS with no factor-2 error.
    Status AND independently calculated constraint residuals gate the first input.
    An absent input means rejection, not a zero-force safety instruction.
    """
    A, B, Q, Qf = (np.asarray(c[k], dtype=float) for k in ['A','B','Q','Qf'])
    x0 = np.asarray(c['x'], dtype=float)
    if x0.shape != (4,) or not np.isfinite(x0).all():
        return dict(status='invalid_initial_state', accepted=False, action=None)
    n, N = 4, int(c['N'])
    nx = (N + 1)*n
    H = 2*sparse.block_diag([sparse.kron(sparse.eye(N), Q), Qf,
                             c['R']*sparse.eye(N)], format='csc')
    Ax = sparse.kron(sparse.eye(N+1), -sparse.eye(n)) + sparse.kron(sparse.eye(N+1,k=-1), A)
    Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1,N)), sparse.eye(N)]), B)
    eq = sparse.hstack([Ax, Bu], format='csc')
    beq = np.concatenate([-x0, np.zeros(N*n)])
    lo = np.full(nx+N, -np.inf); hi = np.full(nx+N, np.inf)
    lo[nx:] = -c['limit']; hi[nx:] = c['limit']
    if rail is not None:
        lo[:nx:4] = -rail; hi[:nx:4] = rail
    G = sparse.vstack([eq, sparse.eye(nx+N)], format='csc')
    lower = np.concatenate([beq,lo]); upper = np.concatenate([beq,hi])
    solver = osqp.OSQP()
    solver.setup(P=H, q=np.zeros(nx+N), A=G, l=lower, u=upper,
                 verbose=False, eps_abs=1e-9, eps_rel=1e-9,
                 max_iter=20000, polishing=True)
    res = solver.solve(raise_error=False)
    out = dict(status=res.info.status, accepted=False, action=None,
               primal_residual=float(res.info.prim_res), dual_residual=float(res.info.dual_res))
    if res.info.status != 'solved' or res.x is None or not np.isfinite(res.x).all():
        return out
    z = res.x; gz = G @ z
    violation = float(max(0, np.max(lower-gz), np.max(gz-upper)))
    out.update(constraint_violation=violation, J=float(.5*z @ (H @ z)),
               U=z[nx:].tolist(), max_position=float(np.max(np.abs(z[:nx:4]))))
    out['accepted'] = violation <= TOL['feasibility'] and max(out['primal_residual'],out['dual_residual']) <= TOL['feasibility']
    if out['accepted']:
        out['action'] = float(z[nx])
    return out


def model(spec: dict, dt: float = .02/64) -> mujoco.MjModel:
    # The teaching model is a uniform rod, full length 2*l, COM distance l.
    # Explicit COM inertia prevents accidentally comparing a point-mass pole.
    mc, mp, length, g = [float(spec[k]) for k in ['mc','mp','l','gravity']]
    inertia = mp*length*length/3
    xml = f'''<mujoco><option timestep="{dt}" gravity="0 0 {-g}" integrator="RK4"/>
      <worldbody><body name="cart"><joint name="slide" type="slide" axis="1 0 0"/>
      <inertial pos="0 0 0" mass="{mc}" diaginertia="1 1 1"/>
      <body name="pole"><joint name="hinge" type="hinge" axis="0 1 0"/>
      <inertial pos="0 0 {length}" mass="{mp}" diaginertia="{inertia} {inertia} 0.00000001"/>
      </body></body></worldbody><actuator><motor joint="slide" gear="1"/></actuator></mujoco>'''
    return mujoco.MjModel.from_xml_string(xml)


def audit() -> dict:
    f = fixture()
    A, B = np.asarray(f['A']), np.asarray(f['B'])
    P = linalg.solve_discrete_are(A, B, np.diag([2,.5,55,3]), np.array([[.12]]))
    K = linalg.solve(np.array([[.12]]) + B.T @ P @ B, B.T @ P @ A)
    dare_error = float(np.max(np.abs(K.ravel()-f['K'])))
    qp = []
    for c in f['cases']:
        ref = solve_qp(c)
        gap = abs(c['J']-ref['J'])/max(1.,abs(ref['J'])) if ref['accepted'] else None
        action_gap = abs(c['U'][0]-ref['action']) if ref['accepted'] else None
        parity = ref['accepted'] and gap <= TOL['relative_cost_gap'] and action_gap <= TOL['first_action_gap']
        qp.append(dict(x0=c['x'],js_cost=c['J'],js_action=c['U'][0],js_converged=c['converged'],
                       native=ref,relative_cost_gap=gap,first_action_gap=action_gap,parity_pass=bool(parity)))
    feasible = solve_qp(f['cases'][0], rail=2.4)
    infeasible = solve_qp({**f['cases'][0],'x':[2.5,0,0,0]}, rail=2.4)
    invalid = solve_qp({**f['cases'][0],'x':[float('nan'),0,0,0]}, rail=2.4)
    errors=[]
    for row in f['dynamics']:
        m=model(row['spec']); d=mujoco.MjData(m); x=row['x']
        d.qpos[:]=[x[0],x[2]]; d.qvel[:]=[x[1],x[3]]; d.ctrl[0]=row['u']
        mujoco.mj_forward(m,d)
        errors.append(float(np.max(np.abs(d.qacc-np.asarray(row['acc'])))))
    m=model(f['spec']); d=mujoco.MjData(m); x=f['x0']; ref=[]
    d.qpos[:]=[x[0],x[2]]; d.qvel[:]=[x[1],x[3]]
    for u in f['controls']:
        d.ctrl[0]=u
        for _ in range(64): mujoco.mj_step(m,d)
        ref.append([d.qpos[0],d.qvel[0],d.qpos[1],d.qvel[1]])
    refinement=[]
    for replay in f['replay']:
        e=np.asarray(replay['X'])-ref; e[:,2]=(e[:,2]+np.pi)%(2*np.pi)-np.pi
        refinement.append(dict(substeps=replay['n'],rmse=float(np.sqrt(np.mean(e*e))),max_abs=float(np.max(np.abs(e)))))
    decreasing=all(b['rmse']<a['rmse'] for a,b in zip(refinement,refinement[1:]))
    core = dare_error<TOL['dare'] and max(errors)<TOL['acceleration'] and decreasing and feasible['accepted'] and infeasible['status']=='primal infeasible' and infeasible['action'] is None and invalid['action'] is None and all(r['native']['accepted'] for r in qp)
    paths=['src/engine.js','src/plant.js','src/commissioning.js','tests/test_native_reference.py']
    return dict(schema='cartpole-native-reference/v1',scope='offline numerical reference; not hardware commissioning',
      versions={k:importlib.metadata.version(k) for k in ['numpy','scipy','osqp','mujoco']},
      source_sha256={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in paths},
      fixture_sha256=hashlib.sha256(json.dumps(f,sort_keys=True).encode()).hexdigest(),tolerances=TOL,
      references={'osqp':'https://osqp.org/docs/examples/mpc.html','dare':'https://docs.scipy.org/doc/scipy/reference/generated/scipy.linalg.solve_discrete_are.html','mujoco':'https://mujoco.readthedocs.io/en/stable/computation/index.html'},
      dare_max_abs_error=dare_error,chi_square_pointwise95={str(n):stats.chi2.ppf([.025,.975],n).tolist() for n in [2,4]},
      qp_cases=qp,hard_rail_feasible=feasible,hard_rail_infeasible=infeasible,invalid_state_rejected=invalid,
      dynamics=dict(cases=len(errors),max_acceleration_error=max(errors),refinement=refinement,refinement_pass=decreasing),
      native_reference_pass=bool(core),browser_qp_parity_pass=all(r['parity_pass'] for r in qp),
      hardware_admission='NOT_EVALUATED')


if __name__ == '__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--require-browser-parity',action='store_true')
    args=parser.parse_args();report=audit()
    (ROOT/'evidence/native_reference.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    summary={k:report[k] for k in ['versions','dare_max_abs_error','dynamics','native_reference_pass','browser_qp_parity_pass','hardware_admission']}
    summary['qp_cases']=[{k:r[k] for k in ['x0','relative_cost_gap','first_action_gap','parity_pass']} for r in report['qp_cases']]
    print(json.dumps(summary,indent=2))
    sys.exit(0 if report['native_reference_pass'] and (not args.require_browser_parity or report['browser_qp_parity_pass']) else 1)
