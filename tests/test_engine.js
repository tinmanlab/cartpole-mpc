'use strict';
const assert = require('assert');
const fs = require('fs');
const Lab = require('../src/engine');
const actor = JSON.parse(fs.readFileSync('./assets/ppo_actor.json','utf8'));
const residual = JSON.parse(fs.readFileSync('./assets/residual_model.json','utf8'));

function mean(a){return a.reduce((s,v)=>s+v,0)/a.length;}

const controllers=['pid','lqr','linear_mpc','centroidal_mpc','full_nmpc','ppo'];
for(const c of controllers){
  const rs=[1,2,3].map(seed=>Lab.runEpisode({
    controller:c, observer:'truth', scenario:'nominal', seed,
    steps:300, actor, residualModel:residual, pushAt:120, pushForce:2
  }));
  assert(rs.every(r=>!r.failed), c+' must survive the fixed nominal truth-state probe');
}

const plant = new Lab.LabPlant({seed:9,scenario:'nominal'});
const linear = new Lab.LinearMPCController(plant.spec);
const centroidal = new Lab.CentroidalMPCController(plant.spec);
const full = new Lab.FullNMPCController(plant.spec);
linear.reset(); centroidal.reset(); full.reset();
linear.act([0,0,.04,0],0);
centroidal.act([0,0,.04,0],0);
full.act([0,0,.04,0],0);
assert.equal(linear.lastPrediction.length-1,30);
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
assert(mean(nmpc.map(r=>r.meanSolveMs))<20);

const noisyRaw=[1,2,3].map(seed=>Lab.runEpisode({
  controller:'lqr', observer:'raw', scenario:'sensor', seed,
  steps:300, actor, residualModel:residual, pushAt:120, pushForce:3
}));
assert(noisyRaw.filter(r=>r.failed).length>=2);

console.log(JSON.stringify({
  controllers:"6/6 nominal truth probes pass",
  horizons:{linear:30,centroidal:32,fullNmpc:30},
  fullNmpcMeanSolveMs:mean(nmpc.map(r=>r.meanSolveMs)),
  noisyRawFailures:noisyRaw.filter(r=>r.failed).length
}, null, 2));
