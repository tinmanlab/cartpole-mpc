'use strict';
const fs=require('fs'),path=require('path'),Lab=require('../src/engine');
const actor=JSON.parse(fs.readFileSync(path.join(__dirname,'..','assets','ppo_actor.json'),'utf8'));
const residual=JSON.parse(fs.readFileSync(path.join(__dirname,'..','assets','residual_model.json'),'utf8'));
const controllers=['pid','lqr','linear_mpc','centroidal_mpc','full_nmpc','ppo'];
const observers=['truth','raw','kf','ekf','so2','residual','adaptive'];
const scenarios=['nominal','sensor','model','mixed','glitch'],seeds=[1,2,3];
const out={
  schema:'cartpole-mpc-audited-evaluation/v2',
  timingContract:'u_k -> x_(k+1) -> y_(k+1) -> xhat_(k+1)',
  measurementCovariance:'scenario sensor variance is passed to KF/EKF-family R',
  steps:300,push:{at:120,force:3,steps:10},seeds,rows:[]
};
for(const scenario of scenarios)for(const controller of controllers)for(const observer of observers){
  const rs=seeds.map(seed=>Lab.runEpisode({controller,observer,scenario,seed,steps:300,actor,residualModel:residual,pushAt:120,pushForce:3}));
  out.rows.push({
    scenario,controller,observer,
    failures:rs.filter(r=>r.failed).length,
    meanStateRmse:rs.reduce((s,r)=>s+r.rmseState,0)/rs.length,
    meanMaxAngleDeg:rs.reduce((s,r)=>s+r.maxAngle,0)/rs.length*180/Math.PI,
    meanSolveMs:rs.reduce((s,r)=>s+(r.meanSolveMs||0),0)/rs.length,
    meanSolverIterations:rs.reduce((s,r)=>s+(r.meanSolverIterations||0),0)/rs.length
  });
}
out.residualArtifact={schema:residual.schema,metrics:residual.metrics,timingContract:residual.timingContract};
fs.mkdirSync(path.join(__dirname,'..','evidence'),{recursive:true});
const dest=path.join(__dirname,'..','evidence','control_observer_v2_metrics.json');
fs.writeFileSync(dest,JSON.stringify(out,null,2));
console.log(dest,out.rows.length);
