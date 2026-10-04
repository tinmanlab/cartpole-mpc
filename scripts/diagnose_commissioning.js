'use strict';
const fs=require('fs'),path=require('path');

function mean(a){return a.length?a.reduce((s,v)=>s+v,0)/a.length:0;}
function ratio(a,b){return Math.abs(b)>1e-12?a/b:null;}
function findRow(rows,controller,observer,scenario){return rows.find(r=>r.controller===controller&&r.observer===observer&&r.scenario===scenario);}
function finding(id,severity,status,evidence,interpretation,next){
  return {id,severity,status,evidence,interpretation,next};
}
function diagnose(e){
  const out=[];
  const sid=e.systemIdentification,aid=e.actuatorIdentification,ec=e.estimatorCalibration,ct=e.controllerTuning,cot=e.controlEstimationCoTuning,mt=e.mpcTuning,sf=e.safetySupervisor,rp=e.robustMpcProbe,mh=e.modelHierarchy||[],stress=e.stress||[],abl=e.singleFactorAblation||[],adv=e.advancedFailures,disc=e.discretization;
  if(sid){
    const r=ratio(sid.heldout.identifiedLoss,sid.heldout.nominalLoss);
    out.push(finding('D01_MODEL_ID','high',r!==null&&r<.1?'PASS':'INVESTIGATE',
      {heldoutNominal:sid.heldout.nominalLoss,heldoutIdentified:sid.heldout.identifiedLoss,ratio:r,identifiability:sid.identifiability},
      r!==null&&r<.1?'identified physical parameters materially improve held-out prediction':'model structure, excitation, or parameter identifiability remains suspect',
      'If INVESTIGATE: inspect residual structure, sensitivity correlation, excitation and model class before controller tuning.'));
  }
  if(aid){
    const r=ratio(aid.heldout.identifiedLoss,aid.heldout.nominalLoss);
    out.push(finding('D02_ACTUATOR_ID','high',r!==null&&r<.1?'PASS':'INVESTIGATE',
      {true:aid.trueParams,identified:aid.identified,heldoutRatio:r},
      r!==null&&r<.1?'gain/lag/delay identification closes most command-to-applied mismatch':'actuator bandwidth/latency model is not identified well enough',
      'Do not let controller gains compensate for an unidentified actuator.'));
  }
  if(ec){
    out.push(finding('D03_ESTIMATOR_CALIBRATION','high',ec.accepted?'PASS':'REJECT',
      ec.scores,ec.accepted?'estimator calibration improves held-out validation and test':'estimator calibration overfits or degrades held-out data',
      ec.accepted?'Keep the calibrated candidate only with its validity domain and consistency report.':'Reject candidate; revisit Q_e/R_e parameterization, bias/contact model, or calibration data.'));
  }
  if(ct){
    out.push(finding('D04_CONTROLLER_TUNING','high',ct.accepted?'PASS':'REJECT',
      ct.scores,ct.accepted?'controller tuning improves held-out validation and test':'training improvement does not generalize to held-out closed-loop conditions',
      ct.accepted?'Admit only inside tested uncertainty envelope.':'Reject candidate; preserve baseline and inspect objective, uncertainty split, constraints and model mismatch.'));
  }
  if(cot){
    out.push(finding('D04A_CONTROL_ESTIMATION_COTUNING','high',cot.accepted?'PASS':'REJECT',
      cot.scores,cot.accepted?'joint low-dimensional controller-estimator candidate improves held-out validation and test':'joint candidate overfits or degrades held-out closed-loop conditions',
      cot.accepted?'Admit only as a low-dimensional black-box bridge; robot scale should use differentiable/bilevel co-design when tractable.':'Reject candidate; preserve staged baseline and inspect model, estimator consistency, constraints and objective coupling.'));
  }
  if(mt){
    out.push(finding('D04B_MPC_TUNING','high',mt.accepted?'PASS':'REJECT',
      {baseline:mt.baseline,tuned:mt.tuned,scores:mt.scores},
      mt.accepted?'structured MPC horizon/cost candidate improves held-out validation and test':'MPC hyperparameter candidate does not generalize despite its training score',
      mt.accepted?'Keep candidate only inside the tested scenario envelope; robot-scale tuning should use native solver sensitivities/BO as appropriate.':'Reject candidate; distinguish horizon/weight tuning from missing constraints or model-class errors.'));
  }
  if(mh.length){
    out.push(finding('D04C_MODEL_HIERARCHY','medium','INFO',
      {rows:mh.map(r=>({controller:r.controller,thetaDeg:r.thetaDeg,failures:r.failures,maxPosition:r.maxPosition,rmsControl:r.rmsControl}))},
      'initial-angle sweep measures the validity envelope of fixed-linear, successive-linearization, state-aware and nonlinear formulations under the same truth-state plant',
      'Use the sweep to choose the next model class; do not infer a universal controller ranking from one CartPole envelope.'));
  }
  if(sf){
    out.push(finding('D04E_SAFETY_SUPERVISOR','high',sf.accepted?'PASS':'INVESTIGATE',
      {rows:sf.rows},sf.accepted?'runtime backup supervisor reduces the fixed large-angle track-boundary failure without changing the primary NMPC':'backup supervisor does not improve the fixed safety probe',
      'This is only a runtime monitor/backup bridge; robot deployment still needs native safety constraints, verified backup/interlock and formal analysis when required.'));
  }
  if(rp){
    out.push(finding('D04D_ROBUST_MPC_FORMULATION','high',rp.accepted?'PASS':'REJECT',
      {rows:rp.rows},
      rp.interpretation,
      rp.accepted?'Admit only as a finite-scenario bridge; no tube/min-max/chance guarantee follows.':'Do not relabel scenario diversity as robustness. Revisit uncertainty coverage, constraints, risk measure and compute budget before deployment.'));
  }
  for(const pipeline of abl){
    const base=pipeline.rows.find(r=>r.scenario==='nominal');if(!base)continue;
    const ranked=pipeline.rows.filter(r=>r.scenario!=='nominal').map(r=>({
      scenario:r.scenario,
      deltaRmse:r.deltaRmse,
      deltaMaxPosition:r.deltaMaxPosition,
      deltaCommandMismatch:r.deltaCommandMismatch,
      failures:r.failures
    })).sort((a,b)=>(b.failures-a.failures)||(b.deltaMaxPosition-a.deltaMaxPosition)||(b.deltaRmse-a.deltaRmse));
    out.push(finding('D05_SINGLE_FACTOR_'+pipeline.controller.toUpperCase(),'medium','INFO',
      {controller:pipeline.controller,observer:pipeline.observer,ranked},
      'single-factor ranking localizes which realism axis most changes this pipeline before combined randomization',
      'Start debugging with the highest-ranked single factor, not the combined sim2real scenario.'));
  }
  if(adv){
    const colored=adv.sensorRows.filter(r=>r.scenario==='colored'),drop=adv.sensorRows.filter(r=>r.scenario==='dropout'),stuck=adv.sensorRows.filter(r=>r.scenario==='stuck'),
      jitter=adv.actuatorRows.filter(r=>r.scenario==='jitter'),ts=adv.actuatorRows.filter(r=>r.scenario==='torque_speed'),thermal=adv.actuatorRows.filter(r=>r.scenario==='thermal');
    out.push(finding('D05B_SENSOR_FAULTS','high','INFO',
      {coloredLag1:mean(colored.map(r=>r.innovationLag1)),dropoutStaleSamples:drop.reduce((a,r)=>a+r.staleSamples,0),stuckFaultSamples:stuck.reduce((a,r)=>a+r.faultSamples,0)},
      'colored noise, stale packets and stuck measurements produce signatures that fixed white-Gaussian R tuning does not explain',
      'On robot scale add timestamp/freshness telemetry, innovation whiteness tests and fault isolation before changing controller gains.'));
    out.push(finding('D05C_ACTUATOR_ENVELOPE','high','INFO',
      {jitterCommandMismatch:mean(jitter.map(r=>r.rmsCommandMismatch)),torqueSpeedMinForce:Math.min(...ts.map(r=>r.minForceLimit)),thermalMinForce:Math.min(...thermal.map(r=>r.minForceLimit)),thermalState:Math.max(...thermal.map(r=>r.maxThermalState))},
      'transport jitter and state/history-dependent actuator authority are distinguishable from nominal control-cost tuning',
      'Replace educational limits with measured torque-speed/current/voltage/thermal envelopes and synchronized command-vs-applied telemetry.'));
  }
  if(disc){
    const coarse=disc.rows[0],fine=disc.rows.at(-1),ratio=coarse&&fine&&fine.rmseVs16>0?coarse.rmseVs16/fine.rmseVs16:null;
    out.push(finding('D05D_DISCRETIZATION','medium',ratio!==null&&ratio>2?'INVESTIGATE':'PASS',
      {rows:disc.rows,coarseToFineErrorRatio:ratio},
      ratio!==null&&ratio>2?'trajectory changes materially with integration refinement':'this fixed refinement probe shows limited integration sensitivity',
      ratio!==null&&ratio>2?'Validate timestep/integrator convergence before retuning controller or estimator parameters.':'Keep the refinement check when model or control rate changes.'));
  }
  const lin=findRow(stress,'linear_mpc','ekf','mixed'),bounded=findRow(stress,'state_mpc','ekf','mixed');
  if(lin&&bounded){
    out.push(finding('D06_STATE_CONSTRAINT_SIGNAL','medium',
      bounded.failures<=lin.failures&&bounded.maxPosition<=lin.maxPosition?'PASS':'INVESTIGATE',
      {linear:{failures:lin.failures,maxPosition:lin.maxPosition},stateAware:{failures:bounded.failures,maxPosition:bounded.maxPosition}},
      'compares an input-bounded MPC with a soft state-bound analogue; improvement indicates objective/constraint formulation matters independently of solver sophistication',
      'For robot scale replace soft educational penalty with explicit native state/path constraints and feasibility diagnostics.'));
  }
  const lat=stress.filter(r=>r.scenario==='latency'),act=stress.filter(r=>r.scenario==='actuator');
  out.push(finding('D07_TRANSPORT_SIGNATURE','medium','INFO',
    {latencyCommandMismatch:mean(lat.map(r=>r.rmsCommandMismatch)),actuatorCommandMismatch:mean(act.map(r=>r.rmsCommandMismatch))},
    'nonzero command-to-applied mismatch separates transport/actuator failure from pure controller-state tracking error',
    'On hardware log commanded and applied torque/current with synchronized timestamps.'));
  const misses=stress.reduce((s,r)=>s+r.deadlineMissRate,0);
  out.push(finding('D08_REALTIME','high',stress.length?(misses===0?'MEASURED_ONLY':'INVESTIGATE'):'UNAVAILABLE',
    {meanSolverDeadlineMissRate:stress.length?misses/stress.length:null,measurementScope:'solver-call-only',endToEndRealtimeVerified:false},
    misses===0?'no observed solver-call deadline miss; end-to-end real-time admission is not established':'observed solver-call deadline violations require investigation',
    'Measure sensor acquisition-to-actuator application age, stale-plan rejection, scheduler/concurrency and fallback behavior on the target processor; solver timing alone is insufficient.'));
  const combined=stress.filter(r=>r.scenario==='sim2real');
  out.push(finding('D09_COMBINED_SIM2REAL','high',combined.some(r=>r.failures>0)?'BOUNDARY_FOUND':'PASS',
    {rows:combined.map(r=>({controller:r.controller,observer:r.observer,failures:r.failures,rmseState:r.rmseState,maxPosition:r.maxPosition,rmsCommandMismatch:r.rmsCommandMismatch}))},
    combined.some(r=>r.failures>0)?'combined hidden mismatch exposes a failure boundary not visible in all nominal runs':'tested combined stack passes, within this CartPole uncertainty envelope only',
    'If failures exist, return to D05 single-factor evidence before changing algorithms.'));
  return {schema:'cartpole-diagnosis/v1',sourceSchema:e.schema,sourceGeneratedAt:e.generatedAt,findings:out};
}
if(require.main===module){
  const root=path.join(__dirname,'..'),src=path.join(root,'evidence','commissioning.json'),dest=path.join(root,'evidence','diagnosis.json');
  const result=diagnose(JSON.parse(fs.readFileSync(src,'utf8')));fs.writeFileSync(dest,JSON.stringify(result,null,2));console.log(dest);console.log(JSON.stringify(result,null,2));
}
module.exports={diagnose};
