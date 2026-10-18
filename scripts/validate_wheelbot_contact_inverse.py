"""Frozen, full-state 0.5s development comparison. No simulator flags changed."""
import json
from pathlib import Path
from time import perf_counter_ns
import numpy as np
import mujoco
import scipy
import wheelbot_contact_wbc as w

P = w.INVERSE_PROTOCOL
ARTIFACT = w.ROOT / 'test-results/contact-inverse-traces.json'


def plain(v):
    if isinstance(v, np.ndarray): return v.tolist()
    if isinstance(v, np.generic): return v.item()
    raise TypeError(type(v).__name__)


def stats(values):
    a = np.asarray(values)
    return dict(median_ns=float(np.median(a)), p95_ns=float(np.percentile(a,95)),
                max_ns=int(max(a)), over_10ms=int(np.sum(a > 10_000_000))) if len(a) else None


def metrics(m, initial, x):
    g = w.material_geometry(m,x); g0 = w.material_geometry(m,initial)
    d = mujoco.MjData(m); d.qpos[:] = x[:6]; d.qvel[:] = x[6:]
    mujoco.mj_fwdPosition(m,d)
    penetration = max([0.]+[-float(d.contact[i].dist) for i in range(d.ncon)])
    excursion = max(0., *(m.jnt_range[j,0]-x[j] for j in [3,4]), *(x[j]-m.jnt_range[j,1] for j in [3,4]))
    return dict(penetration_m=penetration, edge_rise_m=g['edge'][2]-g0['edge'][2], edge_clearance_m=g['edge'][2],
                root_rise_m=float(x[1]-initial[1]), away_rotation_rad=float(np.sign(initial[2])*(x[2]-initial[2])),
                joint_excursion_rad=float(excursion))


def baseline_diagnostic(m, x, qp, r):
    a = r['y'][:6]; tau = r['y'][6:9]
    f, _, inv = w.inverse_eval(m,x,a)
    d = mujoco.MjData(m); d.qpos[:] = x[:6]; d.qvel[:] = x[6:]; d.ctrl[:] = tau
    mujoco.mj_forward(m,d)
    predicted = {}
    for k,c in enumerate(qp['contacts']):
        key = '/'.join(c['names']); force = c['frame'].T@r['y'][9+3*k:12+3*k]
        predicted[key] = (np.array(predicted.get(key,[0.]*3))+force).tolist()
    actual = w.measured(m,d)
    error = max([0.] + [float(np.max(np.abs(np.array(predicted.get(k,[0]*3))-actual.get(k,[0]*3)))) for k in predicted.keys() | actual.keys()])
    return dict(root_required=float(max(abs(f[:3]))), qacc_error=abs(d.qacc-a).tolist(),
                force_error=error, rigid_predicted_forces=predicted, forward_forces=actual,
                inverse_contacts=w.contact_pairs(m,inv), forward_contacts=w.contact_pairs(m,d))


def trial(m, sign, mode):
    initial = w.settle(m,sign); x = initial.copy(); controller = w.ContactInverseController(m)
    trace = []; all_metrics = [metrics(m,initial,x)]; max_pen = 0.; timings = {}
    wall = perf_counter_ns(); rejected = None; matched = False
    for step in range(50):
        start = perf_counter_ns()
        if mode == 'inverse':
            targets = w.inverse_reference(initial,x,step*.01)
            r = controller.solve(x,*targets); diag = {k:r[k] for k in ['root_required','qacc_error','force_error'] if k in r}
        else:
            qp = w.build(m,x,mode); r = w.solve(qp)
            diag = baseline_diagnostic(m,x,qp,r) if r['accepted'] else {}
        row = dict(step=step, state=x.copy(), result=r, diagnostic=diag)
        if not r['accepted']:
            rejected = dict(step=step,status=r['status'],message=r.get('message',''))
            trace.append(row); break
        out = w.advance_inverse(m,x,r) if mode == 'inverse' else w.advance(m,x,qp,r)
        if mode != 'inverse':
            # Existing comparison stepper exposes preintegration state and warmstart.
            for sub in out['substeps']:
                d = mujoco.MjData(m); d.qpos[:] = sub['state'][:6]; d.qvel[:] = sub['state'][6:]
                d.ctrl[:] = r['y'][6:9]; d.qacc_warmstart[:] = sub['qacc_warmstart_input']
                mujoco.mj_forward(m,d); sub['contacts'] = w.contact_pairs(m,d)
            for k, sub in enumerate(out['substeps']):
                sub['after'] = out['substeps'][k+1]['state'] if k < 4 else out['state'].tolist()
                sub['tau'] = r['y'][6:9].tolist(); sub['solver_status'] = r['status']
        if mode == 'inverse':
            np.testing.assert_allclose(out['substeps'][0]['qacc'],r['qacc'],atol=1e-6,rtol=0)
        assert not np.asarray(out['external_forces']).any()
        for sub in out['substeps']:
            assert len(sub['actuator_force']) == 3
            assert all(c['local_force'][0] >= -1e-12 for c in sub['contacts'])
            assert np.all(np.abs(sub['actuator_force']) <= np.array(P['torque_limits_nm'])+1e-8)
        x = out['state']; row['replay'] = out
        for sub in out['substeps']:
            met = metrics(m,initial,np.asarray(sub['after'])); all_metrics.append(met)
            max_pen = max(max_pen,sub['penetration_m'],met['penetration_m'])
            matched |= met['edge_rise_m'] >= .005 and met['away_rotation_rad'] >= .05
        row['metrics'] = metrics(m,initial,x)
        t = perf_counter_ns(); json.dumps(row,default=plain); logging = perf_counter_ns()-t
        timing = dict(r['timing_ns'], **out['timing_ns']); timing['logging'] = timing.get('logging',0)+logging
        timing['forwardvalidation'] = timing.get('forwardvalidation',0)+timing.pop('advance_validation',0)
        timing['fullstep'] = perf_counter_ns()-start; row['timing_ns'] = timing
        for key,value in timing.items(): timings.setdefault(key,[]).append(value)
        trace.append(row)
    wall_ns = perf_counter_ns()-wall
    n = sum('replay' in r for r in trace)
    max_joint = max(r['joint_excursion_rad'] for r in all_metrics)
    physics = max_joint <= .05 and max_pen <= .02
    force_errors = [r['diagnostic']['force_error'] for r in trace if 'force_error' in r['diagnostic']]
    accel_errors = [max(r['diagnostic']['qacc_error']) for r in trace if 'qacc_error' in r['diagnostic']]
    summary = dict(name=('right' if sign > 0 else 'left')+'_'+mode, requested_commands=50,
        completed_commands=n, simulated_s=n*.01, rejection=rejected,
        qp_accepted_commands=n if mode != 'inverse' else None,
        inverse_slsqp_accepted_commands=n if mode == 'inverse' else None,
        forward_prediction_match=bool(accel_errors and max(accel_errors)<=1e-6 and max(force_errors)<=1e-6),
        primitive_success=bool(n==50 and matched and physics and mode=='inverse'),
        simultaneous_thresholds_reached=bool(matched), whole_trial_admissible=bool(physics and n==50),
        full_getup=False, max_edge_rise_m=max(r['edge_rise_m'] for r in all_metrics),
        max_root_rise_m=max(r['root_rise_m'] for r in all_metrics),
        max_away_rotation_rad=max(r['away_rotation_rad'] for r in all_metrics),
        max_joint_excursion_rad=max_joint, max_penetration_m=max_pen,
        max_force_error_n=max(force_errors,default=None), max_qacc_error=max(accel_errors,default=None),
        final=metrics(m,initial,x), final_pitch_velocity_rad_s=float(x[8]),
        final_reference_rotation_error_rad=float(np.sign(initial[2])*(x[2]-initial[2])-P['rotation_rad']) if mode=='inverse' else None,
        peak_rotation_overshoot_rad=max(0., max(r['away_rotation_rad'] for r in all_metrics)-P['rotation_rad']) if mode=='inverse' else None,
        timing_ns={k:stats(v) for k,v in timings.items()},
        fullwall_ns_excluding_settle=wall_ns, wall_to_requested_0_5s=wall_ns/500_000_000)
    return summary,trace


def main():
    m = w.model(); assert w.sha(w.XML) == P['model_sha256']
    sources = ['scripts/wheelbot_contact_wbc.py','scripts/validate_wheelbot_contact_inverse.py',
               'tests/test_wheelbot_contact_inverse.py','tests/fixtures/wheelbot_contact_inverse.json',
               'tests/fixtures/wheelbot_contact_wbc.json','assets/wheelbot/contact_model.xml',
               'assets/wheelbot/contact_profile.json','docs/WHEELBOT_CONTACT_WBC.md']
    report = dict(schema='same-contact-inverse-evidence/v1', protocol=P,
        hashes={s:w.sha(w.ROOT/s) for s in sources}, mujoco=mujoco.__version__,scipy=scipy.__version__,
        method='MuJoCo inverse/contact/forward; SciPy SLSQP with transposed mjd_inverseFD DfDa; no TSID binary',
        full_state_only=True, development_poses=True, observer=False, rl=False, native_wasm=False,
        full_getup=False, hard_realtime=False, settle_excluded_s_per_trial=4,
        numerical_flags=dict(enable=int(m.opt.enableflags),disable=int(m.opt.disableflags)),trials=[])
    traces = {}; ARTIFACT.parent.mkdir(exist_ok=True)
    for sign in [-1,1]:
        for mode in ['inverse','hold','pivot']:
            summary,trace = trial(m,sign,mode); report['trials'].append(summary); traces[summary['name']] = trace
            # Preserve completed evidence after each bounded trial.
            ARTIFACT.write_text(json.dumps(traces,default=plain))
            report['trace_path'] = str(ARTIFACT.relative_to(w.ROOT)); report['trace_sha256'] = w.sha(ARTIFACT)
            (w.ROOT/'evidence/wheelbot_contact_inverse.json').write_text(json.dumps(report,indent=2,default=plain)+'\n')
            print(json.dumps(summary,default=plain),flush=True)

if __name__ == '__main__': main()
