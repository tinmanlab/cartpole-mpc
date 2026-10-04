// Narrow shared lesson runner: existing MuJoCo plant, EKF and LQR are the authorities.
// No new controller/filter, no optimizer, no controller feedback from evaluation truth.
(function(root,factory){const api=factory(typeof module==='object'&&module.exports?require('./engine'):ControlLab);if(typeof module==='object'&&module.exports)module.exports=api;else root.CalibrationLab=api;})(typeof globalThis!=='undefined'?globalThis:this,function(L){
 'use strict';
 const mean=a=>a.length?a.reduce((s,v)=>s+v,0)/a.length:null;
 const lag1=a=>{if(a.length<3)return null;const m=mean(a),d=a.reduce((s,v)=>s+(v-m)**2,0);return d>0?a.slice(1).reduce((s,v,i)=>s+(v-m)*(a[i]-m),0)/d:null;};
 const clone=A=>A?.map(r=>r.slice())??null;
 function estimateMeasurementNoise(measurements,screen={}){
  const minimum=screen.minimumSamples??32;
  if(!Array.isArray(measurements)||measurements.length<minimum||measurements.length>100000||!measurements.every(y=>Array.isArray(y)&&y.length===2&&y.every(Number.isFinite)))throw Error('Need enough finite two-channel stationary measurements');
  // Local circular chart; unknown static angle mean is NOT an identified bias.
  const anchor=measurements[0][1],samples=measurements.map(y=>[y[0],anchor+L.wrap(y[1]-anchor)]);
  if(Math.max(...samples.map(y=>y[1]))-Math.min(...samples.map(y=>y[1]))>=Math.PI)throw Error('Stationary local angle chart is invalid');
  const mu=[0,0],M=[[0,0],[0,0]];let count=0;
  for(const y of samples){count++;const d=y.map((v,i)=>v-mu[i]);for(let i=0;i<2;i++)mu[i]+=d[i]/count;for(let i=0;i<2;i++)for(let j=0;j<2;j++)M[i][j]+=d[i]*(y[j]-mu[j]);}
  const covariance=M.map(r=>r.map(v=>v/(count-1))),Rdiag=[covariance[0][0],covariance[1][1]];
  if(!Rdiag.every(v=>Number.isFinite(v)&&v>0))throw Error('Positive finite measurement covariance required');
  const correlation=covariance[0][1]/Math.sqrt(Rdiag[0]*Rdiag[1]),lags=[0,1].map(i=>lag1(samples.map(y=>y[i]))),warnings=[];
  if(lags.some(v=>Math.abs(v)>(screen.maximumAbsLagOne??.2)))warnings.push('serial correlation or motion: white stationary sample assumption not supported');
  if(Math.abs(correlation)>(screen.maximumAbsChannelCorrelation??.2))warnings.push('diagonal R does not represent measured channel correlation');
  return {source:'stationary-measurements',samples:count,mean:mu,covariance,Rdiag,standardDeviation:Rdiag.map(Math.sqrt),channelCorrelation:correlation,lagOne:lags,usable:warnings.length===0,warnings,
   covarianceUnits:[['m^2','m*rad'],['m*rad','rad^2']],biasIdentified:false,scope:'Repeatability around an unknown constant in a stationary fixture. Diagnostic screens are not proof of stationarity/whiteness. No Q_e, constant bias, dynamics or delay is identified.'};
 }
 function collectStationary(m){
  const c=m.calibration;if(!c.stationaryAssumption||!Number.isInteger(c.samples)||c.samples<32)throw Error('Explicit stationary fixture and finite sample count required');
  const p=new L.LabPlant({seed:c.seed,scenario:c.scenario});p.reset(c.fixtureState);
  // Clamped calibration acquisition, not a hidden feedback trial or a falling free plant.
  const measurements=Array.from({length:c.samples},()=>p.sensor().slice());
  return {seed:c.seed,fixture:'declared stationary clamped acquisition; no free plant evolution',channels:['position_m','angle_rad'],measurements,usedSimulationTruthInEstimator:false,usedInjectedVarianceInEstimator:false};
 }
 function armCovariance(arm,fit){
  if(!fit?.usable)throw Error('Calibration sample assumptions rejected');
  const R=arm.Rdiag?arm.Rdiag.slice():fit.Rdiag.map(v=>v*arm.calibrationScale);
  if(R.length!==2||!R.every(v=>Number.isFinite(v)&&v>0))throw Error('Positive measurement covariance required');return R;
 }
 function nis(last){if(!last?.S||!last.innovation)return null;const z=L.solveLinearSystem(last.S,last.innovation);return last.innovation.reduce((s,v,i)=>s+v*z[i],0);}
 function createRun(m,test,arm,fit){
  if(L.physicsInfo().backend!=='mujoco-wasm')throw Error('This lesson requires actual MuJoCo WASM');
  if(test.seed===m.calibration.seed)throw Error('Calibration and evaluation seeds must be disjoint');
  const Rdiag=armCovariance(arm,fit),plant=new L.LabPlant({seed:test.seed,scenario:test.scenario});plant.reset(test.initialState);
  // Scenario fixture supplies matched physical sensor amplitude. The calibration
  // estimator is never passed this setting; mean/variance is fitted from readings only.
  const noise=m.noiseInjectionForMatchedCases;plant.sensorStd=[noise.sigmaPosition_m,noise.sigmaAngle_rad];
  plant.goal=test.goal;const controller=new L.LQRController(plant.spec,m.controller),observer=new L.EKFObserver(plant.spec,{Q:m.observer.Q,R:Rdiag});
  let measurement=plant.sensor();observer.reset(measurement);controller.reset();let estimate=observer.x.slice(),trace=[],outcome='running',reason=null;
  function step(){
   if(outcome!=='running')return false;
   const k=trace.length,truth=plant.s.slice(),eHat=estimate.slice(),eTruth=truth.slice();eHat[0]-=test.goal;eTruth[0]-=test.goal;
   try{
    const requestedForce=-controller.K.reduce((s,v,i)=>s+v*eHat[i],0),command=controller.act(estimate,test.goal);
    if(!Number.isFinite(command)||Math.abs(command)>m.controller.forceLimit+1e-12)throw Error('Controller command violates declared force bound');
    const error=estimate.map((v,i)=>i===2?L.wrap(v-truth[i]):v-truth[i]);
    // These truth-based quantities are evaluator-only and never change command.
    const oracleForce=-controller.K.reduce((s,v,i)=>s+v*eTruth[i],0),contributions=estimate.map((v,i)=>-controller.K[i]*(v-truth[i]));
    const row={k,measurementTime:k*m.controlDt,commandTime:k*m.controlDt,nextTime:(k+1)*m.controlDt,measurement:measurement.slice(),truth,estimate:estimate.slice(),estimationError:error,
      innovation:k?observer.last.innovation?.slice()??null:null,S:k?clone(observer.last.S):null,P:clone(observer.P),filterGain:k?clone(observer.last.K):null,nis:k?nis(observer.last):null,
      goal:test.goal,requestedForce,oracleForce,estimationForceContributions:contributions,command,saturated:Math.abs(requestedForce)>m.controller.forceLimit,clipLoss:requestedForce-command};
    if(test.push&&k===test.push.step)plant.applyPush(test.push.force,test.push.duration);
    const out=plant.step(command);row.appliedForce=out.appliedCommand;row.external=out.external;row.nextTruth=out.state.slice();
    measurement=plant.sensor();estimate=observer.step(plant.sensorMeta.fresh?measurement:null,command);row.nextMeasurement=measurement.slice();row.nextEstimate=estimate.slice();
    if(!estimate.every(Number.isFinite))throw Error('Nonfinite estimate');trace.push(row);
    if(out.failed){outcome='envelope-failure';reason=Math.abs(out.state[0])>2.4?'rail':'angle';}else if(trace.length>=m.steps)outcome='completed';
    return outcome==='running';
   }catch(e){outcome='execution-error';reason=String(e.message);return false;}
  }
  function result(){
   const rms=a=>a.length?Math.sqrt(mean(a.map(v=>v*v))):null,N=trace.length,saturated=trace.filter(r=>r.saturated).length,last=trace.slice(-m.evaluation.goalWindowSteps);
   const taskPassed=outcome==='completed'&&last.length===m.evaluation.goalWindowSteps&&last.every(r=>Math.abs(r.nextTruth[0]-test.goal)<=m.evaluation.positionTolerance_m&&Math.abs(r.nextTruth[2])<=m.evaluation.angleTolerance_rad);
   return {caseId:test.id,group:test.group,scenario:test.scenario,seed:test.seed,armId:arm.id,label:arm.label,Rdiag,parameterSource:arm.Rdiag?'explicit-prior-assumption':'stationary-measurements',calibrationMultiplier:arm.calibrationScale??null,
    K:controller.K.slice(),Qc:controller.Q,Rc:controller.R,Qe:m.observer.Q.slice(),outcome,reason,appliedSteps:N,requestedSteps:m.steps,taskPassed,
    positionTrackingRmse_m:rms(trace.map(r=>r.truth[0]-r.goal)),estimationRmseByState:[0,1,2,3].map(i=>rms(trace.map(r=>r.estimationError[i]))),stateUnits:['m','m/s','rad','rad/s'],
    rmsForce_N:rms(trace.map(r=>r.command)),rmsRequestedForce_N:rms(trace.map(r=>r.requestedForce)),rmsEstimationForce_N:rms(trace.map(r=>r.requestedForce-r.oracleForce)),
    rmsCommandSlew_N:rms(trace.slice(1).map((r,i)=>r.command-trace[i].command)),saturatedSamples:saturated,saturationFraction:N?saturated/N:null,
    meanNis:mean(trace.map(r=>r.nis).filter(Number.isFinite)),innovationLagOne:[0,1].map(i=>lag1(trace.filter(r=>r.innovation).map(r=>r.innovation[i]))),
    maxPosition_m:N?Math.max(...trace.map(r=>Math.abs(r.nextTruth[0]))):null,finalTruth:plant.s.slice(),finalEstimate:estimate.slice(),trace,
    information:{controllerInput:'observer-output only',calibration:'separate sensor-readings only',truth:'evaluation/plots only',fixture:'simulated noise amplitude and static acquisition are test assumptions',timing:'measurement and estimate at t_k drive force during [t_k,t_(k+1)); post-step measurement is next frame'}};
  }
  return {step,result,get done(){return outcome!=='running';}};
 }
 function replayObserver(m,recording,arms,fit){
  return arms.map(arm=>{
   const reference=recording.trace;if(!reference.length)throw Error('No replay data');
   const p=new L.LabPlant({scenario:recording.scenario,seed:recording.seed});const o=new L.EKFObserver(p.spec,{Q:m.observer.Q,R:armCovariance(arm,fit)});o.reset(reference[0].measurement);
   const sq=[0,0,0,0];let maxDifference=0;
   for(let i=0;i<reference.length;i++){
    if(i)o.step(reference[i].measurement,reference[i-1].command);
    const x=o.x;for(let j=0;j<4;j++){const d=j===2?L.wrap(x[j]-reference[i].truth[j]):x[j]-reference[i].truth[j];sq[j]+=d*d;maxDifference=Math.max(maxDifference,Math.abs(x[j]-reference[i].estimate[j]));}
   }
   return {armId:arm.id,recordedArm:recording.armId,samples:reference.length,estimationRmseByState:sq.map(v=>Math.sqrt(v/reference.length)),stateUnits:['m','m/s','rad','rad/s'],
    matchedArmMaximumDifference:arm.id===recording.armId?maxDifference:null,controllerApplied:false,scope:'Same recorded measurements/commands, no new plant evolution; not a new closed-loop performance measurement'};
  });
 }
 function comparePair(base,candidate){
  const steps=Math.min(base.trace.length,candidate.trace.length),MSE=(r,f)=>steps?r.trace.slice(0,steps).reduce((s,v)=>s+f(v)**2,0)/steps:null;
  return {commonPrefixSteps:steps,bothCompleted:base.outcome==='completed'&&candidate.outcome==='completed',
   baselineTrackingMse:MSE(base,r=>r.truth[0]-r.goal),candidateTrackingMse:MSE(candidate,r=>r.truth[0]-r.goal),
   baselineEstimatePositionMse:MSE(base,r=>r.estimationError[0]),candidateEstimatePositionMse:MSE(candidate,r=>r.estimationError[0]),
   scope:'Closed loops share fixture/RNG, not identical measurements after different controls; early endings are not scored as long-horizon success'};
 }
 return {estimateMeasurementNoise,collectStationary,armCovariance,createRun,replayObserver,comparePair};
});
