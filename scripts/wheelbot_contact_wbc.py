"""Bounded full-state inverse dynamics diagnostic using MuJoCo and OSQP."""
from pathlib import Path
import hashlib
import json
from time import perf_counter_ns
import mujoco
import numpy as np
import osqp
from scipy import sparse

ROOT = Path(__file__).resolve().parents[1]
XML = ROOT / 'assets/wheelbot/contact_model.xml'
PROFILE = ROOT / 'assets/wheelbot/contact_profile.json'
PROTOCOL = json.loads((ROOT / 'tests/fixtures/wheelbot_contact_wbc.json').read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def model():
    if sha(XML) != PROTOCOL['model_sha256']:
        raise ValueError('Physical XML hash mismatch')
    m = mujoco.MjModel.from_xml_path(str(XML))
    assert mujoco.__version__ == '3.7.0' and (m.nq, m.nv, m.nu) == (6, 6, 3)
    assert m.opt.timestep == .002
    np.testing.assert_array_equal(m.actuator_ctrlrange[:, 1], [16, 16, 1.7])
    return m


def settle(m, sign):
    x = np.r_[PROTOCOL['fall_qpos'], np.zeros(6)]
    x[2] *= sign
    d = mujoco.MjData(m)
    for _ in range(400):
        mujoco.mj_resetData(m, d); d.qpos[:] = x[:6]; d.qvel[:] = x[6:]
        mujoco.mj_step(m, d, nstep=5)
        x = np.r_[d.qpos, d.qvel].copy()
    return x


def jac(m, d, point, body):
    J = np.zeros((3, 6)); D = np.zeros((3, 6))
    mujoco.mj_jac(m, d, J, None, point, body)
    mujoco.mj_jacDot(m, d, D, None, point, body)
    return J, D


def inspect(m, x):
    if np.shape(x) != (12,) or not np.isfinite(x).all():
        raise ValueError('Invalid full state')
    d = mujoco.MjData(m); d.qpos[:] = x[:6]; d.qvel[:] = x[6:]
    mujoco.mj_forward(m, d)
    M = np.zeros((6, 6)); mujoco.mj_fullM(m, M, d.qM)
    B = np.zeros((6, 3))
    for a in range(3):
        B[m.jnt_dofadr[m.actuator_trnid[a, 0]], a] = m.actuator_gear[a, 0]
    assert not B[:3].any()
    contacts = []
    for k in range(d.ncon):
        c = d.contact[k]
        b1, b2 = int(m.geom_bodyid[c.geom1]), int(m.geom_bodyid[c.geom2])
        J1, D1 = jac(m, d, c.pos, b1); J2, D2 = jac(m, d, c.pos, b2)
        frame = c.frame.reshape(3, 3).copy()
        names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, int(g)) for g in [c.geom1, c.geom2]]
        wheel_floor = names == ['floor', 'wheel_visual']
        # Floor wheel normal uses the hub, never a moving contact as material point.
        if wheel_floor:
            Jn, Dn = jac(m, d, d.xpos[b2], b2)
            Jn, Dn = frame[0] @ Jn, frame[0] @ Dn
        else:
            Jn, Dn = frame[0] @ (J2-J1), frame[0] @ (D2-D1)
        contacts.append(dict(b1=b1, b2=b2, pos=c.pos.copy(), frame=frame,
            J=frame @ (J2-J1), Jn=Jn, drift=float(Dn @ d.qvel),
            dist=float(c.dist), mu=c.friction[:2].copy(), names=names,
            wheel_floor=wheel_floor, support=True))
    return d, M, B, contacts


def build(m, x, mode, static=False):
    start = perf_counter_ns()
    if mode not in ['hold', 'pivot']:
        raise ValueError('Unsupported contact mode')
    d, M, B, contacts = inspect(m, x)
    h = (d.qfrc_bias - d.qfrc_passive).copy()  # deliberately excludes qfrc_constraint
    model_ns = perf_counter_ns()-start; start = perf_counter_ns()
    if mode == 'pivot':
        edges = [c for c in contacts if c['names'] == ['floor', 'torso_visual']]
        if edges:
            pivot_x = (min if x[2] > 0 else max)(c['pos'][0] for c in edges)
            for c in edges:
                c['support'] = abs(c['pos'][0]-pivot_x) < 1e-5
    n = 9+3*len(contacts)
    rows, lower, upper = [], [], []
    def constrain(row, lo, hi):
        rows.append(row); lower.append(lo); upper.append(hi)
    dyn = np.zeros((6, n)); dyn[:, :6] = M; dyn[:, 6:9] = -B
    for k, c in enumerate(contacts):
        dyn[:, 9+3*k:12+3*k] = -c['J'].T
    for i in range(6):
        constrain(dyn[i], -h[i], -h[i])
    for a, limit in enumerate(PROTOCOL['torque_limits_nm']):
        row = np.zeros(n); row[6+a] = 1; constrain(row, -limit, limit)
    for k, c in enumerate(contacts):
        i = 9+3*k
        row = np.zeros(n); row[i] = 1; constrain(row, 0, np.inf if c['support'] else 0)
        for s1 in [-1, 1]:
            for s2 in [-1, 1]:
                row = np.zeros(n); row[i:i+3] = [-1, s1/c['mu'][0], s2/c['mu'][1]]
                constrain(row, -np.inf, 0)  # inner friction diamond
        row = np.zeros(n); row[:6] = c['Jn']
        target = -20 * (c['Jn'] @ d.qvel) - c['drift']
        constrain(row, target, target if c['support'] else np.inf)
    # Semi-implicit Euler prediction: sum(1..5)*dt² = .00006 at 10ms.
    # Also constrain the first 2ms substep and brake outward velocity over 50ms.
    for j in [3, 4]:
        for dt, coefficient in [(.002, .000004), (.01, .00006)]:
            row = np.zeros(n); row[j] = coefficient
            constrain(row, m.jnt_range[j, 0]-.05-x[j]-dt*x[6+j],
                      m.jnt_range[j, 1]+.05-x[j]-dt*x[6+j])
        row = np.zeros(n); row[j] = .01
        constrain(row, (m.jnt_range[j, 0]-.05-x[j])/.05-x[6+j],
                  (m.jnt_range[j, 1]+.05-x[j])/.05-x[6+j])
    if static:
        for j in range(6):
            row = np.zeros(n); row[j] = 1; constrain(row, 0, 0)
    # Dimensionless objective: pitch/(1 rad/s²), COM-z/(1 m/s²),
    # other accelerations/(1 unit/s²), torque/limits, force/(total weight).
    C = np.zeros((8, n)); target = np.zeros(8)
    C[:6, :6] = np.eye(6)*.01
    C[6, 2] = 1
    target[6] = (-np.sign(x[2]) if mode == 'pivot' else 0) - 2*x[8]
    mass = m.body_mass.sum(); Jcom = np.zeros((3, 6)); drift = np.zeros(3)
    for b in range(1, m.nbody):
        J, D = jac(m, d, d.xipos[b], b)
        Jcom += m.body_mass[b]/mass * J
        drift += m.body_mass[b]/mass * (D @ d.qvel)
    C[7, :6] = Jcom[2]
    target[7] = (.1 if mode == 'pivot' else 0) - 2*(Jcom[2] @ d.qvel)-drift[2]
    reg = np.r_[np.zeros(6), 1e-6/np.square(PROTOCOL['torque_limits_nm']),
                np.full(n-9, 1e-6/(mass*9.81)**2)]
    return dict(P=2*(C.T@C+np.diag(reg)), q=-2*C.T@target,
                A=np.array(rows), lo=np.array(lower), hi=np.array(upper),
                contacts=contacts, M=M, B=B, h=h, x=x.copy(), mode=mode,
                timing_ns={'model_contacts_jac': model_ns, 'qp_build': perf_counter_ns()-start})


def check(qp, y):
    if y is None or np.shape(y) != qp['q'].shape or not np.isfinite(y).all():
        return dict(accepted=False, reason='nonfinite or missing primal')
    value = qp['A'] @ y
    violation = np.maximum(np.maximum(qp['lo']-value, value-qp['hi']), 0)
    # Row-wise scale measured in the native row units, no solver status shortcut.
    scale = 1+np.abs(qp['A']) @ np.abs(y)
    raw, scaled = float(violation.max()), float((violation/scale).max())
    return dict(accepted=raw <= 1e-5 and scaled <= 1e-6,
                residual_inf=raw, scaled_residual_inf=scaled,
                dynamics_residual_inf=float(np.max(np.abs(value[:6]-qp['lo'][:6]))))


def solve(qp):
    t = perf_counter_ns(); solver = osqp.OSQP()
    solver.setup(P=sparse.csc_matrix(qp['P']), q=qp['q'], A=sparse.csc_matrix(qp['A']),
                 l=qp['lo'], u=qp['hi'], verbose=False, eps_abs=1e-8,
                 eps_rel=1e-8, max_iter=20000, polishing=True)
    setup = perf_counter_ns()-t; t = perf_counter_ns(); r = solver.solve(raise_error=False)
    elapsed = perf_counter_ns()-t
    checked = check(qp, r.x)
    dual = float(r.info.dual_res); primal = float(r.info.prim_res)
    accepted = (r.info.status == 'solved' and checked['accepted'] and
                np.isfinite([dual, primal]).all() and max(dual, primal) <= 1e-5)
    return dict(checked, accepted=bool(accepted), status=r.info.status, y=r.x,
                solver_primal_residual=primal, solver_dual_residual=dual,
                timing_ns={'solver_setup': setup, 'solver_solve': elapsed})


def measured(m, d):
    forces = {}
    for k in range(d.ncon):
        c = d.contact[k]; f = np.zeros(6); mujoco.mj_contactForce(m, d, k, f)
        key = '/'.join(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, int(g)) for g in [c.geom1, c.geom2])
        forces[key] = (np.array(forces.get(key, [0., 0., 0.]))+c.frame.reshape(3, 3).T@f[:3]).tolist()
    return forces


def advance(m, x, qp, result):
    if not result['accepted'] or result['status'] != 'solved' or not check(qp, result['y'])['accepted']:
        raise ValueError('Rejected QP: no physics advance')
    d = mujoco.MjData(m); d.qpos[:] = x[:6]; d.qvel[:] = x[6:]
    d.ctrl[:] = result['y'][6:9]
    substeps = []; physics_ns = 0
    for k in range(5):
        warm = d.qacc_warmstart.copy(); before = np.r_[d.qpos, d.qvel].copy()
        t = perf_counter_ns(); mujoco.mj_step(m, d); physics_ns += perf_counter_ns()-t
        # mj_step's forces/qacc refer to BEFORE integration; don't forward again.
        substeps.append(dict(time_s=k*.002, state=before.tolist(),
            forces=measured(m, d), qacc=d.qacc.tolist(),
            qacc_warmstart_input=warm.tolist(), qacc_warmstart_output=d.qacc_warmstart.tolist(),
            actuator_force=d.actuator_force.tolist(),
            penetration_m=max([0.]+[-float(d.contact[i].dist) for i in range(d.ncon)])))
    return dict(state=np.r_[d.qpos, d.qvel], substeps=substeps,
                external_forces=d.qfrc_applied.tolist()+d.xfrc_applied.ravel().tolist(),
                timing_ns={'physics_5steps': physics_ns})


# Same-contact inverse diagnostic; the rigid-contact comparison above is preserved.
from scipy.optimize import minimize
INVERSE_PROTOCOL_PATH = ROOT / 'tests/fixtures/wheelbot_contact_inverse.json'
INVERSE_PROTOCOL = json.loads(INVERSE_PROTOCOL_PATH.read_text())


def contact_pairs(m, d):
    rows = []
    for k in range(d.ncon):
        c = d.contact[k]; f = np.zeros(6)
        mujoco.mj_contactForce(m, d, k, f)
        rows.append(dict(geoms=[int(c.geom1), int(c.geom2)], pos=c.pos.tolist(),
                         frame=c.frame.tolist(), local_force=f.tolist(), dist=float(c.dist)))
    return rows


def inverse_eval(m, x, acceleration, derivative=False, *, scratch=None):
    if np.shape(x) != (12,) or np.shape(acceleration) != (6,) or not np.isfinite(np.r_[x, acceleration]).all():
        raise ValueError('Invalid inverse state/acceleration')
    d = mujoco.MjData(m) if scratch is None else scratch
    if scratch is not None:
        if d.qpos.shape != (m.nq,) or d.qvel.shape != (m.nv,):
            raise ValueError('Inverse workspace dimensions mismatch')
        mujoco.mj_resetData(m, d)  # preserve fresh-state semantics, not solver history
    d.qpos[:] = x[:6]; d.qvel[:] = x[6:]
    d.qacc[:] = acceleration
    mujoco.mj_inverse(m, d)
    force = d.qfrc_inverse.copy(); J = None
    if derivative:
        transposed = np.zeros((6, 6))
        mujoco.mjd_inverseFD(m, d, INVERSE_PROTOCOL['jacobian_eps'], 0,
                            None, None, transposed, None, None, None, None)
        J = transposed.T.copy()
        d.qacc[:] = acceleration; mujoco.mj_inverse(m, d)
    return force, J, d


def acceleration_bounds(m, x):
    lo = np.full(6, -np.inf); hi = -lo
    for j in [3, 4]:
        for dt, coefficient in [(.002, .000004), (.01, .00006), (.05, .0005)]:
            # Third row is the existing 50ms outward-velocity braking bound.
            lo[j] = max(lo[j], (m.jnt_range[j, 0]-.05-x[j]-dt*x[6+j])/coefficient)
            hi[j] = min(hi[j], (m.jnt_range[j, 1]+.05-x[j]-dt*x[6+j])/coefficient)
    return lo, hi


def verify_inverse(m, x, result):
    p = INVERSE_PROTOCOL
    if (not result.get('success', False) or result.get('status') != 0 or
        np.shape(result.get('state')) != (12,) or not np.array_equal(x, result['state'])):
        return dict(accepted=False, reason='solver status or state mismatch')
    a, tau = np.asarray(result['qacc']), np.asarray(result['tau'])
    if a.shape != (6,) or tau.shape != (3,) or not np.isfinite(np.r_[x,a,tau]).all():
        return dict(accepted=False, reason='nonfinite or malformed command')
    f, _, inv = inverse_eval(m, x, a)
    d = mujoco.MjData(m); d.qpos[:] = x[:6]; d.qvel[:] = x[6:]; d.ctrl[:] = tau
    # Both fresh MjData instances have zero outside forces and cold warmstart.
    mujoco.mj_forward(m, d)
    ip, fp = contact_pairs(m, inv), contact_pairs(m, d)
    ordered = len(ip) == len(fp) and all(a['geoms'] == b['geoms'] and np.allclose(a['pos'], b['pos'], atol=1e-12, rtol=0) for a,b in zip(ip,fp))
    fi, ff = measured(m, inv), measured(m, d)
    force_error = max([0.] + [float(np.max(np.abs(np.array(fi.get(k,[0]*3))-ff.get(k,[0]*3)))) for k in fi.keys() | ff.keys()])
    errors = np.abs(d.qacc-a); root = float(np.max(np.abs(f[:3])))
    torque = float(max(0., np.max(np.abs(tau)-p['torque_limits_nm'])))
    lo, hi = acceleration_bounds(m, x)
    accepted = bool(ordered and np.all(errors <= p['qacc_component_tolerance']) and root <= p['root_tolerance'] and torque <= p['torque_violation_tolerance_nm'] and np.max(np.abs(f[3:]-tau)) <= 1e-8 and force_error <= p['force_tolerance_n'] and np.all(a >= lo-1e-8) and np.all(a <= hi+1e-8) and all(c['local_force'][0] >= -1e-12 for c in fp))
    return dict(accepted=accepted, root_required=root, qacc_error=errors.tolist(),
                qacc_units=['m/s2','m/s2','rad/s2','rad/s2','rad/s2','rad/s2'],
                torque_violation=torque, force_error=force_error, ordered_contacts=ordered,
                inverse_contacts=ip, forward_contacts=fp, inverse_forces=fi, forward_forces=ff,
                forward_qacc=d.qacc.tolist(), required_force=f.tolist(), warmstart_input=[0.]*6)


class ContactInverseController:
    def __init__(self, m, *, reuse_workspace=False, max_iterations=None):
        self.m = m
        self.max_iterations = INVERSE_PROTOCOL['maxiter'] if max_iterations is None else max_iterations
        if type(self.max_iterations) is not int or not 1 <= self.max_iterations <= 400:
            raise ValueError('Iteration budget must be an integer from1to400')
        self._inverse_workspace = mujoco.MjData(m) if reuse_workspace else None
        _, _, B, _ = inspect(m, np.zeros(12))
        np.testing.assert_array_equal(B, np.r_[np.zeros((3,3)), np.eye(3)])

    def solve(self, x, pitch_acc, root_z_acc):
        t = perf_counter_ns(); m = self.m; p = INVERSE_PROTOCOL
        cache = {}
        def evaluate(a):
            key = a.tobytes()
            if cache.get('key') != key:
                f, J, _ = inverse_eval(m, x, a, True, scratch=self._inverse_workspace)
                cache.update(key=key, f=f, J=J)
            return cache['f'], cache['J']
        def objective(a):
            return (a[2]-pitch_acc)**2+.1*(a[1]-root_z_acc)**2+1e-5*(a@a)
        def gradient(a):
            g = 2e-5*a.copy(); g[2] += 2*(a[2]-pitch_acc); g[1] += .2*(a[1]-root_z_acc)
            return g
        limits = np.array(p['torque_limits_nm'])
        lo, hi = acceleration_bounds(m, x)
        d = mujoco.MjData(m); d.qpos[:] = x[:6]; d.qvel[:] = x[6:]; mujoco.mj_forward(m,d)
        r = minimize(objective, np.clip(d.qacc,lo,hi), jac=gradient, method='SLSQP',
            bounds=list(zip(lo, hi)), constraints=[
                dict(type='eq', fun=lambda a:evaluate(a)[0][:3], jac=lambda a:evaluate(a)[1][:3]),
                dict(type='ineq', fun=lambda a:np.r_[limits-evaluate(a)[0][3:], limits+evaluate(a)[0][3:]],
                     jac=lambda a:np.r_[-evaluate(a)[1][3:],evaluate(a)[1][3:]])],
            options=dict(maxiter=self.max_iterations, ftol=p['ftol']))
        f, _, _ = inverse_eval(m, x, r.x)
        result = dict(state=x.copy(), qacc=r.x.copy(), tau=f[3:].copy(), success=bool(r.success),
                      status=int(r.status), message=r.message, iterations=int(r.nit))
        opt_ns = perf_counter_ns()-t; t = perf_counter_ns()
        result.update(verify_inverse(m,x,result))
        result['timing_ns'] = dict(inverseopt_jacobians=opt_ns, forwardvalidation=perf_counter_ns()-t)
        return result


def advance_inverse(m, x, result):
    validation_start = perf_counter_ns()
    if not result.get('accepted', False) or not verify_inverse(m,x,result)['accepted']:
        raise ValueError('Rejected inverse command: no physics advance')
    validation_ns = perf_counter_ns()-validation_start
    d = mujoco.MjData(m); d.qpos[:] = x[:6]; d.qvel[:] = x[6:]; d.ctrl[:] = result['tau']
    rows = []; physics_ns = 0; logging_ns = 0
    for k in range(5):
        before = np.r_[d.qpos,d.qvel].copy(); warm = d.qacc_warmstart.copy()
        t = perf_counter_ns(); mujoco.mj_step(m,d); physics_ns += perf_counter_ns()-t
        t = perf_counter_ns()
        rows.append(dict(state=before.tolist(), after=np.r_[d.qpos,d.qvel].tolist(),
            qacc=d.qacc.tolist(), contacts=contact_pairs(m,d), forces=measured(m,d),
            qacc_warmstart_input=warm.tolist(), qacc_warmstart_output=d.qacc_warmstart.tolist(),
            actuator_force=d.actuator_force.tolist(), tau=d.ctrl.tolist(), solver_status=result['status'],
            penetration_m=max([0.]+[-float(d.contact[i].dist) for i in range(d.ncon)])))
        logging_ns += perf_counter_ns()-t
    return dict(state=np.r_[d.qpos,d.qvel].copy(), substeps=rows,
                external_forces=d.qfrc_applied.tolist()+d.xfrc_applied.ravel().tolist(),
                timing_ns=dict(physics_5steps=physics_ns, logging=logging_ns, advance_validation=validation_ns))


def material_geometry(m, x):
    d = mujoco.MjData(m); d.qpos[:] = x[:6]; mujoco.mj_kinematics(m,d)
    b = mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'torso')
    side = np.sign(x[2])*.0625; R = d.xmat[b].reshape(3,3)
    return dict(edge=(d.xpos[b]+R@np.array([side,0,0])).tolist(),
                head=(d.xpos[b]+R@np.array([side,0,.25])).tolist())


def inverse_reference(initial, x, t):
    p = INVERSE_PROTOCOL; T = p['ramp_s']; u = np.clip(t/T,0,1)
    s = 3*u*u-2*u*u*u; sd = (6*u-6*u*u)/T if t < T else 0.
    sdd = (6-12*u)/T**2 if t < T else 0.
    sign = np.sign(initial[2]); angle = initial[2]+sign*p['rotation_rad']*s
    vel = sign*p['rotation_rad']*sd; acc = sign*p['rotation_rad']*sdd
    # Fixed material head edge: zroot = zhead + side*sin(pitch)-L*cos(pitch).
    side = sign*.0625; L = .25
    offset = lambda a: side*np.sin(a)-L*np.cos(a)
    z = initial[1]+offset(angle)-offset(initial[2])
    dz = (side*np.cos(angle)+L*np.sin(angle))*vel
    ddz = (side*np.cos(angle)+L*np.sin(angle))*acc+(-side*np.sin(angle)+L*np.cos(angle))*vel**2
    return (acc+p['kp_s_inv2']*(angle-x[2])+p['kd_s_inv']*(vel-x[8]),
            ddz+p['kp_s_inv2']*(z-x[1])+p['kd_s_inv']*(dz-x[7]))
