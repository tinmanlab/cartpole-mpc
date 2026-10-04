'use strict';
// Contract regressions: a stable animation is not evidence of correct estimation.
const assert = require('assert');
const Lab = require('../src/engine');
const C = require('../src/commissioning');
const spec = new Lab.LabPlant().spec;
const results = [];
function check(name, fn) {
  try { fn(); results.push({name, passed:true}); }
  catch (error) { results.push({name, passed:false, error:error.message}); }
}
const close = (a,b,tol=1e-10) => assert(Math.abs(a-b)<tol, `${a} != ${b}`);
const tr = P => P.reduce((s,r,i)=>s+r[i],0);

check('UKF measurement projection includes additive process noise', () => {
  const a = new Lab.UKFObserver(spec, {Q:[0,0,0,0]});
  const b = new Lab.UKFObserver(spec, {Q:[.01,.02,.03,.04]});
  a.reset([0,0]); b.reset([0,0]); a.step([0,0],0); b.step([0,0],0);
  // Position/angle measurements are linear in the local state chart.
  close(b.last.S[0][0]-a.last.S[0][0], .01);
  close(b.last.S[1][1]-a.last.S[1][1], .03);
});

check('missing measurements cause prediction only, not a repeated correction', () => {
  for (const name of ['kf','ekf','so2','ukf','adaptive','residual','mhe','raw']) {
    const obs=Lab.makeObserver(name,spec); obs.reset([0,0]);
    const before=obs.outputCovariance?.(); const p0=before ? tr(before) : null;
    for (let k=0;k<3;k++) {
      const x=obs.step(null,0);
      assert(x.every(Number.isFinite), `${name}: non-finite prediction`);
      assert.strictEqual(obs.last?.measurementUsed,false, `${name}: missing sample used`);
      assert.strictEqual(obs.last?.S ?? null,null, `${name}: stale innovation covariance`);
    }
    const after=obs.outputCovariance?.();
    if (after) assert(tr(after)>p0, `${name}: false certainty during outage`);
    assert(obs.step([0,0],0).every(Number.isFinite), `${name}: recovery failed`);
  }
});

check('dropout metadata reaches the closed-loop estimator', () => {
  const run=Lab.runEpisode({observer:'ekf',scenario:'dropout',steps:100,pushAt:999});
  const missing=run.trace.filter(q=>!q.sensorFresh);
  assert(missing.length>0);
  assert(missing.every(q=>q.measurementUsed===false && q.S===null && q.innovation===null));
});

check('raw finite differences use elapsed time across a missing interval', () => {
  const o=new Lab.RawObserver(); o.reset([0,0]);
  o.step(null); o.step(null); const x=o.step([.06,.03]);
  close(x[1],1); close(x[3],.5);
});

check('unavailable consistency is not reported as perfect consistency', () => {
  const r=C.estimatorConsistency({trace:[]});
  assert.strictEqual(r.nisMean,null); assert.strictEqual(r.neesMean,null);
  assert.strictEqual(r.available,false); assert.strictEqual(r.penalty,null);
});

check('zero innovations and zero state errors remain zero, not chi-square means', () => {
  const r=C.estimatorConsistency({trace:[{S:[[1,0],[0,1]],innovation:[0,0],
    truth:[0,0,0,0],estimate:[0,0,0,0],P:Lab.eye(4)}]});
  assert.strictEqual(r.nisMean,0); assert.strictEqual(r.neesMean,0);
  assert(r.available && r.penalty>10);
});

check('MHE arrival residual retains covariance correlations and angle geometry', () => {
  const o=new Lab.ShootingMHEObserver(spec); o.reset([0,0]);
  o.records[0].baseP=[[4,1,0,0],[1,2,0,0],[0,0,1,0],[0,0,0,1]];
  const r=o.residualVector([1,2,0,0]).r;
  close(r.reduce((s,v)=>s+v*v,0),2);
  const eps=Math.PI/180; o.records[0].baseX=[0,0,Math.PI-eps,0];
  o.records[0].baseP=Lab.eye(4);
  const a=o.residualVector([0,0,-Math.PI+eps,0]).r;
  close(a.reduce((s,v)=>s+v*v,0),(2*eps)**2);
});

check('backup trigger uses physical track coordinates, not goal-relative error', () => {
  const o=new Lab.SupervisedNMPCController(spec);
  o.primary.act=()=>{o.primary.lastPrediction=[[2.2,0,0,0]];return 1;};
  o.backup.act=()=>-1;
  close(o.act([2,0,0,0],2),-1);
  assert(o.lastBackup);
});

check('DARE exposes incomplete and converged iteration status', () => {
  const m=Lab.linearModel(spec),Q=Lab.diag([2,.5,55,3]);
  const short=Lab.dare(m.A,m.B,Q,.12,1),full=Lab.dare(m.A,m.B,Q,.12);
  assert.strictEqual(short.converged,false);assert.strictEqual(short.iterations,1);
  assert.strictEqual(full.converged,true);assert(full.iterations>1 && full.iterations<=2000);
});

check('measurement covariance provenance discloses simulator privilege', () => {
  const a=Lab.runEpisode({steps:2}),b=Lab.runEpisode({steps:2,observerOpts:{R:[.01,.01]}});
  assert.strictEqual(a.informationProvenance.measurementCovariance,'injected-noise-oracle');
  assert.strictEqual(b.informationProvenance.measurementCovariance,'provided');
});

console.log(JSON.stringify({schema:'cartpole-commissioning-contracts/v1',results},null,2));
if(results.some(r=>!r.passed)) process.exitCode=1;
