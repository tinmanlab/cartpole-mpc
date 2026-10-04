"""Optional matched experiment: solver choice and terminal design are separate factors.
Actual plant and EKF remain in one Node/MuJoCo WASM process. No hardware I/O.
"""
from __future__ import annotations
import hashlib, importlib.metadata, json, os, platform, selectors, subprocess, time
from pathlib import Path
import numpy as np
from native_nmpc import ROOT, UPSTREAM, NativeNMPC, read_model, discrete_model, N, DT, Q, QF, R, TOL

class PlantBridge:
    def __init__(self):
        self.p=subprocess.Popen(['node','scripts/native_plant_bridge.mjs'],cwd=ROOT,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,bufsize=1)
    def call(self,request):
        self.p.stdin.write(json.dumps(request,allow_nan=False)+'\n');self.p.stdin.flush()
        with selectors.DefaultSelector() as selector:
            selector.register(self.p.stdout,selectors.EVENT_READ)
            if not selector.select(timeout=30):raise TimeoutError('MuJoCo bridge response missing')
        line=self.p.stdout.readline()
        if not line:raise RuntimeError('MuJoCo bridge closed: '+self.p.stderr.read()[-2000:])
        return json.loads(line)
    def close(self):
        try:self.call({'op':'close'})
        finally:
            self.p.stdin.close()
            try:self.p.wait(timeout=5)
            except subprocess.TimeoutExpired:self.p.terminate();self.p.wait(timeout=5)

def model_audit(bridge):
    fixture=bridge.call({'op':'models'});assert fixture['ok'];p=read_model(ROOT/'assets/cartpole.xml')
    value_error=0.;derivative_error=0.
    for c in fixture['cases']:
        f,a,b=discrete_model({**p,**c['p']})
        value_error=max(value_error,float(np.max(np.abs(np.asarray(f(c['x'],c['u'])).ravel()-c['next']))))
        derivative_error=max(derivative_error,float(np.max(np.abs(np.asarray(a(c['x'],c['u']))-c['A']))),float(np.max(np.abs(np.asarray(b(c['x'],c['u']))-c['B']))))
    assert value_error<1e-10,(value_error,'discrete model mismatch')
    assert derivative_error<1e-7,(derivative_error,'CasADi derivative mismatch')
    return {'cases':len(fixture['cases']),'maxTransitionError':value_error,'maxJacobianError':derivative_error,'transitionTolerance':1e-10,'jacobianTolerance':1e-7,'physics':fixture['physics'],'passed':True}

def distribution(values):
    if not values:return None
    return {'samples':len(values),'p50':float(np.percentile(values,50)),'p95':float(np.percentile(values,95)),'p99':float(np.percentile(values,99)),'max':float(max(values))}

def episode(bridge,case,algorithm,solver=None,terminal_matrix=None,backup=None,terminal_kind='original'):
    extra={'terminalMatrix':terminal_matrix.tolist()} if terminal_matrix is not None else {}
    state=bridge.call({'op':'reset',**case,**extra});assert state['ok']
    if solver:solver.reset()
    if backup:backup.reset()
    states=[];actions=[];native_times=[];host_times=[];ipc_times=[];total_times=[];last_plan=None
    outcome='completed';reason=None;max_defect=0.;max_violation=0.;max_cost_error=0.;max_residual=0.;not_converged=0;fallbacks=0
    for k in range(240):
        start=time.perf_counter()
        if solver:
            # Information barrier: never pass state['truth'] to the controller.
            last_plan=solver.solve_with_recovery(state['estimate'],case['goal'],backup) if backup else solver.solve(state['estimate'],case['goal'])
            fallbacks+=int(last_plan.get('fallbackUsed',False));native_times.append(last_plan['native_ms']);host_times.append(last_plan['host_ms'])
            if not last_plan['accepted']:
                outcome='plan-rejected';reason=last_plan['reason'];total_times.append(1000*(time.perf_counter()-start));break
            max_defect=max(max_defect,last_plan['max_dynamics_defect']);max_violation=max(max_violation,last_plan['constraint_violation']);max_cost_error=max(max_cost_error,last_plan['cost_error'])
            max_residual=max(max_residual,max(last_plan['residuals']));not_converged+=int(not last_plan['nonlinearOptimalityConverged'])
            request={'op':'step','u':last_plan['action']}
        else:request={'op':'step','baseline':True}
        t=time.perf_counter();next_state=bridge.call(request);ipc_times.append(1000*(time.perf_counter()-t));total_times.append(1000*(time.perf_counter()-start))
        if not next_state['ok']:
            outcome='solver-rejected' if next_state['error'].startswith('QP rejected:') else 'execution-error';reason=next_state['error'];break
        state=next_state;states.append(state['truth']);actions.append(state['u'])
        if not solver:native_times.append(state['baselineSolveMs'])
        if state['failed']:outcome='envelope-failure';break
    x=np.asarray(states).reshape(-1,4)
    return {'case':case,'algorithm':algorithm,'terminalDesign':terminal_kind,'fallbackCalls':fallbacks,'outcome':outcome,'reason':reason,'appliedSteps':len(actions),'finalTruth':state['truth'],'finalEstimate':state['estimate'],
        'positionTrackingRmse_m':float(np.sqrt(np.mean((x[:,0]-case['goal'])**2))) if len(x) else None,
        'angleRms_rad':float(np.sqrt(np.mean(x[:,2]**2))) if len(x) else None,'maxPosition_m':float(np.max(np.abs(x[:,0]))) if len(x) else None,
        'rmsForce_N':float(np.sqrt(np.mean(np.asarray(actions)**2))) if actions else None,
        'solverTimeMs':distribution(native_times),'controllerHostTimeMs':distribution(host_times),'bridgeRoundTripMs':distribution(ipc_times),'completeIterationHostMs':distribution(total_times),
        'maxAdmittedDynamicsDefect':max_defect if solver else None,'maxAdmittedConstraintViolation':max_violation if solver else None,'maxCostError':max_cost_error if solver else None,
        'maxAdmittedNlpResidual':max_residual if solver else None,'admittedButNotNlpConvergedSteps':not_converged if solver else None,
        'lastPlan':last_plan,'measurementCovarianceSource':state['measurementCovarianceSource']}

def main():
    source=Path(os.environ.get('ACADOS_SOURCE_DIR',ROOT/'test-results/acados-v0.6.0')).resolve()
    actual=subprocess.check_output(['git','rev-parse','HEAD'],cwd=source,text=True).strip()
    if actual!=UPSTREAM:raise RuntimeError('Unverified acados source revision')
    bridge=PlantBridge()
    try:
        audit=model_audit(bridge)
        cases=[{'scenario':'nominal','seed':43,'goal':.5},{'scenario':'mixed','seed':302,'goal':.5},{'scenario':'sim2real','seed':303,'goal':.5}]
        build_start=time.perf_counter()
        solvers={t:{a:NativeNMPC(a,terminal=t) for a in ['SQP','SQP_RTI']} for t in ['original','dare']}
        build_ms=1000*(time.perf_counter()-build_start);rows=[]
        for case in cases:
            for terminal in ['original','dare']:
                family=solvers[terminal];P=family['SQP'].terminal if terminal=='dare' else None
                for algorithm,solver in [('quadprog-linear',None),('SQP',family['SQP']),('SQP_RTI+SQP',family['SQP_RTI'])]:
                    row=episode(bridge,case,algorithm,solver,terminal_matrix=P,backup=family['SQP'] if algorithm=='SQP_RTI+SQP' else None,terminal_kind=terminal);rows.append(row)
                    print(json.dumps({k:row[k] for k in ['case','terminalDesign','algorithm','outcome','appliedSteps','fallbackCalls']}),flush=True)
        pick=lambda scenario,algorithm,terminal:next(r for r in rows if r['case']['scenario']==scenario and r['algorithm']==algorithm and r['terminalDesign']==terminal)
        checks={'modelAndDerivativeParity':audit['passed'],'nominalSQPCompleted':pick('nominal','SQP','original')['outcome']=='completed',
          'baselineMixedFailureReproduced':pick('mixed','quadprog-linear','original')['outcome']=='envelope-failure',
          'baselineSim2realRejectionReproduced':pick('sim2real','quadprog-linear','original')['outcome']=='solver-rejected',
          'noExecutionError':all(r['outcome']!='execution-error' for r in rows)}
        paths=['assets/cartpole.xml','src/engine.js','src/mujoco_backend.mjs','src/qp.js','scripts/native_nmpc.py','scripts/native_plant_bridge.mjs','scripts/run_native_nmpc.py','tests/test_native_nmpc.py']
        report={'schema':'cartpole-native-nmpc/v2','upstream':{'version':'v0.6.0','commit':actual,'submodules':subprocess.check_output(['git','submodule','status','external/blasfeo','external/hpipm'],cwd=source,text=True).strip().splitlines()},
          'nativeLibrarySha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (source/'lib').glob('*.so')},
          'versions':{k:importlib.metadata.version(k) for k in ['casadi','numpy','scipy']},'host':{'platform':platform.platform(),'machine':platform.machine(),'build':'Release / GENERIC BLASFEO and HPIPM / OpenMP OFF'},
          'sourceSha256':{p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in paths},'modelAudit':audit,
          'problem':{'horizon':N,'controlDt':DT,'Q':Q.tolist(),'R':R,'forceBound':10,'railBound':2.4,'costScaling':'all ones, W=2Q and W_e=2P; matches summed JS cost','push':{'step':96,'force':3,'durationSteps':10},'episodeSteps':240},
          'terminalDesigns':{'originalDiagonal':QF.tolist(),'dareFullMatrix':solvers['dare']['SQP'].terminal.tolist(),'meaning':'SciPy DARE for nominal discrete A/B/Q/R; local terminal cost, no invariant terminal set or global safety claim'},
          'acceptanceTolerances':TOL,'solverConstructionHostMs':build_ms,'rows':rows,'checks':checks,'passed':all(checks.values()),
          'boundaries':['Optional offline native research lane; deployed browser remains quadprog + actual MuJoCo WASM.','Matching formulation data does not make LTI and nonlinear OCPs identical; no global solver-speed or optimality ranking.','SQP_RTI status zero is not nonlinear convergence; admitted plans pass nonlinear rollout and defect checks.','Rejected RTI plans are re-solved by SQP at the same current estimate; no rejected action or zero-force guess is applied.','Host timing includes process/diagnostic overhead where labelled and is not hardware end-to-end or WCET.','Noise covariance remains disclosed simulated oracle. No hidden plant mass/gain/delay enters the controller.','Terminal design was proposed after baseline diagnosis. These development cases are not untouched held-out tests or a robust recovery certificate.'],
          'hardwareAdmission':'NOT_EVALUATED','browserSolverPromotion':'NOT_AUTHORIZED_BY_THIS_EXPERIMENT'}
        (ROOT/'evidence/native_nmpc.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        if not report['passed']:raise AssertionError(checks)
    finally:bridge.close()

if __name__=='__main__':main()
