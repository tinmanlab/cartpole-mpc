'use strict';
const assert = require('assert');
const fs = require('fs');
const Lab = require('../src/engine');
const actor = JSON.parse(fs.readFileSync('./assets/ppo_actor.json','utf8'));
const residual = JSON.parse(fs.readFileSync('./assets/residual_model.json','utf8'));

function mean(a){return a.reduce((s,v)=>s+v,0)/a.length;}

const controllers=['pid','lqr','linear_mpc','scenario_mpc','state_mpc','ltv_mpc','centroidal_mpc','full_nmpc','supervised_nmpc','ppo'];
for(const c of controllers){
  const rs=[1,2,3].map(seed=>Lab.runEpisode({
    controller:c, observer:'truth', scenario:'nominal', seed,
    steps:300, actor, residualModel:residual, pushAt:120, pushForce:2
  }));
  assert(rs.every(r=>!r.failed), c+' must survive the fixed nominal truth-state probe');
}

const plant = new Lab.LabPlant({seed:9,scenario:'nominal'});
const linear = new Lab.LinearMPCController(plant.spec);
const scenarioMpc = new Lab.ScenarioMPCController(plant.spec);
const stateMpc = new Lab.StateAwareLinearMPCController(plant.spec);
const ltv = new Lab.LTVMPCController(plant.spec);
const centroidal = new Lab.CentroidalMPCController(plant.spec);
const full = new Lab.FullNMPCController(plant.spec);
linear.reset(); stateMpc.reset(); ltv.reset(); centroidal.reset(); full.reset();
linear.act([0,0,.04,0],0);
scenarioMpc.act([0,0,.04,0],0);
stateMpc.act([0,0,.04,0],0);
ltv.act([0,0,.04,0],0);
centroidal.act([0,0,.04,0],0);
full.act([0,0,.04,0],0);
assert.equal(linear.lastPrediction.length-1,30);
assert.equal(scenarioMpc.lastPrediction.length-1,30);
assert.equal(scenarioMpc.lastScenarioCosts.length,5);
assert.equal(stateMpc.lastPrediction.length-1,30);
assert.equal(ltv.lastPrediction.length-1,30);
assert.equal(ltv.lastIterations,1);
assert.equal(centroidal.lastPrediction.length-1,32);
assert.equal(full.lastPrediction.length-1,30);
assert.equal(centroidal.lastReference.length,2);
assert(full.lastIterations>=1 && full.lastIterations<=2);
assert(full.lastSolveMs>0);

const sensor = [1,2,3].map(seed=>Lab.runEpisode({
  controller:'centroidal_mpc', observer:'kf', scenario:'sensor', seed,
  steps:300, actor, residualModel:residual, pushAt:120, pushForce:3
}));
assert(sensor.every(r=>!r.failed));

const nmpc = [1,2,3].map(seed=>Lab.runEpisode({
  controller:'full_nmpc', observer:'truth', scenario:'nominal', seed,
  steps:300, actor, residualModel:residual, pushAt:120, pushForce:3
}));
assert(nmpc.every(r=>!r.failed));
assert(nmpc.every(r=>Number.isFinite(r.meanSolveMs)&&r.meanSolveMs>=0));

const noisyRaw=[1,2,3].map(seed=>Lab.runEpisode({
  controller:'lqr', observer:'raw', scenario:'sensor', seed,
  steps:300, actor, residualModel:residual, pushAt:120, pushForce:3
}));
assert(noisyRaw.filter(r=>r.failed).length>=2);

// Zero configured delay must not create a hidden one-tick command delay.
const nominalPlant=new Lab.LabPlant({seed:5,scenario:'nominal'});nominalPlant.reset([0,0,0,0]);
const nominalOut=nominalPlant.step(1.25);assert(Math.abs(nominalOut.appliedCommand-1.25)<1e-12);

// MHE is a windowed nonlinear estimator with deliberately uncalibrated output covariance.
const mheRuns=[1,2,3].map(seed=>Lab.runEpisode({controller:'lqr',observer:'mhe',scenario:'sensor',seed,steps:180,pushAt:999,pushForce:0}));
assert(mheRuns.every(r=>!r.failed&&Number.isFinite(r.rmseState)));
assert(mheRuns.every(r=>r.trace.some(q=>q.observerWindow>=2)));
assert(mheRuns.every(r=>r.trace.every(q=>q.P===null)));

// A soft state-bound formulation should reduce the fixed track-boundary failures relative to input-only linear MPC.
const linStress=[301,302,303].map(seed=>Lab.runEpisode({controller:'linear_mpc',observer:'ekf',scenario:'sim2real',seed,steps:260,pushAt:100,pushForce:3}));
const boundStress=[301,302,303].map(seed=>Lab.runEpisode({controller:'state_mpc',observer:'ekf',scenario:'sim2real',seed,steps:260,pushAt:100,pushForce:3}));
assert(boundStress.filter(r=>r.failed).length < linStress.filter(r=>r.failed).length);

// Backup supervisor must reduce the known large-angle track-boundary failure without claiming formal safety.
const baseLarge=[1,2,3].map(seed=>Lab.runEpisode({controller:'full_nmpc',observer:'truth',scenario:'nominal',seed,steps:300,pushAt:999,pushForce:0,initialState:[0,0,.45,0]}));
const safeLarge=[1,2,3].map(seed=>Lab.runEpisode({controller:'supervised_nmpc',observer:'truth',scenario:'nominal',seed,steps:300,pushAt:999,pushForce:0,initialState:[0,0,.45,0]}));
assert(safeLarge.filter(r=>r.failed).length < baseLarge.filter(r=>r.failed).length,'backup supervisor should reduce the fixed large-angle failure count');

// Combined sim2real preset must create a measurable command/application mismatch.
const sim2real=Lab.runEpisode({controller:'lqr',observer:'ekf',scenario:'sim2real',seed:91,steps:120,pushAt:999,pushForce:0});
assert(sim2real.rmsCommandMismatch>0);

// Single-factor sim2real ablations isolate actuator, latency, and sensor-bias effects.
const actuatorRun=Lab.runEpisode({controller:'lqr',observer:'ekf',scenario:'actuator',seed:91,steps:100,pushAt:999,pushForce:0});
assert(actuatorRun.rmsCommandMismatch>0,'actuator-only scenario should create command/application mismatch');
const latencyPlant=new Lab.LabPlant({seed:3,scenario:'latency'});latencyPlant.reset([0,0,0,0]);
const lat1=latencyPlant.step(2),lat2=latencyPlant.step(2),lat3=latencyPlant.step(2);
assert(Math.abs(lat1.appliedCommand)<1e-12&&Math.abs(lat2.appliedCommand)<1e-12&&Math.abs(lat3.appliedCommand)>0,'two-tick latency scenario should delay command application');
const biasPlant=new Lab.LabPlant({seed:3,scenario:'bias'});biasPlant.reset([0,0,0,0]);
const firstBias=biasPlant.sensorBias.slice();for(let i=0;i<50;i++)biasPlant.sensor();const laterBias=biasPlant.sensorBias.slice();
assert(Math.abs(laterBias[0]-firstBias[0])+Math.abs(laterBias[1]-firstBias[1])>0,'bias scenario should evolve sensor bias');

// UKF is a real sigma-point nonlinear observer and must remain numerically well-behaved.
const ukfRuns=[1,2,3].map(seed=>Lab.runEpisode({controller:'lqr',observer:'ukf',scenario:'nonlinear',seed,steps:220,pushAt:999,pushForce:0}));
assert(ukfRuns.every(r=>!r.failed&&Number.isFinite(r.rmseState)));
assert(ukfRuns.every(r=>r.trace.every(q=>q.P===null||q.P.every(row=>row.every(Number.isFinite)))));

// Colored noise must create temporal correlation that a white-noise R alone does not represent.
const coloredPlant=new Lab.LabPlant({seed:12,scenario:'colored'});coloredPlant.reset([0,0,0,0]);
const coloredErr=[];for(let i=0;i<180;i++){const y=coloredPlant.sensor();coloredErr.push(y[0]-coloredPlant.s[0]);}
const cm=mean(coloredErr),lagNum=coloredErr.slice(1).reduce((ss,v,i)=>ss+(v-cm)*(coloredErr[i]-cm),0),lagDen=coloredErr.reduce((ss,v)=>ss+(v-cm)*(v-cm),0);
assert(lagNum/lagDen>.5,'colored-noise scenario should exhibit strong lag-1 correlation');

// Packet dropout and stuck-sensor faults must be observable as distinct metadata signatures.
const drop=Lab.runEpisode({controller:'lqr',observer:'ekf',scenario:'dropout',seed:7,steps:180,pushAt:90,pushForce:2});
assert(drop.trace.some(q=>q.sensorFresh===false&&q.sensorFault==='dropout-hold'));
const stuck=Lab.runEpisode({controller:'lqr',observer:'ekf',scenario:'stuck',seed:7,steps:180,pushAt:90,pushForce:2});
assert(stuck.trace.some(q=>q.sensorFault==='stuck'));

// Actuator envelope degradation must change available authority rather than only the controller cost.
const tsPlant=new Lab.LabPlant({seed:3,scenario:'torque_speed'});tsPlant.reset([0,2.0,0,0]);const tsOut=tsPlant.step(10);
assert(tsOut.forceLimit<tsPlant.spec.force,'torque-speed scenario must reduce force authority at high speed');
const thPlant=new Lab.LabPlant({seed:3,scenario:'thermal'});thPlant.reset([0,0,0,0]);let thOut;
for(let i=0;i<250;i++)thOut=thPlant.step(i%2?10:-10);
assert(thOut.thermalState>0&&thOut.forceLimit<thPlant.spec.force,'thermal scenario must accumulate state and derate force authority');

console.log(JSON.stringify({
  controllers:controllers.length+"/"+controllers.length+" nominal truth probes pass",
  horizons:{linear:30,stateAware:30,ltv:30,centroidal:32,fullNmpc:30},
  fullNmpcMeanSolveMs:mean(nmpc.map(r=>r.meanSolveMs)),
  noisyRawFailures:noisyRaw.filter(r=>r.failed).length
}, null, 2));
