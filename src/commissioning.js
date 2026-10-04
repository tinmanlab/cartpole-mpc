'use strict';
const Lab=require('./engine');

class RNG{
  constructor(seed=1){this.s=seed>>>0;this.spare=null;}
  uniform(){let t=this.s=(this.s+0x6D2B79F5)>>>0;t=Math.imul(t^t>>>15,t|1);t^=t+Math.imul(t^t>>>7,t|61);return((t^t>>>14)>>>0)/4294967296;}
  normal(){if(this.spare!==null){const v=this.spare;this.spare=null;return v;}let x,y,q;do{x=2*this.uniform()-1;y=2*this.uniform()-1;q=x*x+y*y;}while(q===0||q>=1);const k=Math.sqrt(-2*Math.log(q)/q);this.spare=y*k;return x*k;}
}
const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
function mean(a){return a.reduce((s,v)=>s+v,0)/Math.max(1,a.length);}
function cemOptimize({bounds,objective,seed=1,iterations=6,population=18,eliteFraction=.25,initial=null}){
  const rng=new RNG(seed),n=bounds.length,eliteN=Math.max(2,Math.round(population*eliteFraction));
  let mu=initial?initial.slice():bounds.map(([a,b])=>(a+b)/2),sd=bounds.map(([a,b])=>(b-a)/3),best=null,history=[];
  for(let it=0;it<iterations;it++){
    const pop=[];
    for(let k=0;k<population;k++){
      const x=mu.map((m,i)=>clamp(m+sd[i]*rng.normal(),bounds[i][0],bounds[i][1])),score=objective(x);
      pop.push({x,score});if(!best||score<best.score)best={x:x.slice(),score};
    }
    pop.sort((a,b)=>a.score-b.score);const elite=pop.slice(0,eliteN);
    mu=mu.map((_,i)=>mean(elite.map(e=>e.x[i])));
    sd=sd.map((old,i)=>Math.max((bounds[i][1]-bounds[i][0])*0.015,Math.sqrt(mean(elite.map(e=>(e.x[i]-mu[i])**2)))*.9+.1*old));
    history.push({iteration:it,best:best.score,eliteMean:mean(elite.map(e=>e.score)),mu:mu.slice(),sd:sd.slice()});
  }
  return {best,history};
}
function generateSysIdData({seed=7,steps=260}={}){
  const plant=new Lab.LabPlant({seed,scenario:'model'}),rng=new RNG(seed+99),data=[];
  plant.reset([0,0,.06,0]);
  for(let k=0;k<steps;k++){
    const before=plant.s.slice(),u=clamp(5*Math.sin(k*.11)+2.5*Math.sin(k*.037)+(rng.uniform()-.5)*3,-8,8),out=plant.step(u);
    data.push({x:before,u,next:out.state.slice()});
    if(out.failed||Math.abs(out.state[0])>1.8)plant.reset([0,0,(rng.uniform()-.5)*.12,0]);
  }
  return {spec:plant.spec,trueParams:{mc:plant.params.mc,mp:plant.params.mp,l:plant.params.l,friction:plant.params.friction},data};
}
function sysIdLoss(theta,bundle){
  const [mc,mp,l,friction]=theta,sc=[1,2,.3,3];let loss=0;
  const p={mc,mp,l,friction};
  for(const row of bundle.data){
    const pred=Lab.nonlinearStep(row.x,row.u,bundle.spec,p);
    for(let i=0;i<4;i++){let e=pred[i]-row.next[i];if(i===2)e=Lab.wrap(e);loss+=(e/sc[i])**2;}
  }
  return loss/(bundle.data.length*4);
}
function sysIdRolloutLoss(theta,bundle,horizon=20){
  const [mc,mp,l,friction]=theta,p={mc,mp,l,friction},sc=[1,2,.3,3];let loss=0,count=0;
  for(let start=0;start<bundle.data.length;start+=horizon){
    let z=bundle.data[start].x.slice();
    for(let k=start;k<Math.min(bundle.data.length,start+horizon);k++){
      const row=bundle.data[k];z=Lab.nonlinearStep(z,row.u,bundle.spec,p);
      for(let i=0;i<4;i++){let e=z[i]-row.next[i];if(i===2)e=Lab.wrap(e);loss+=(e/sc[i])**2;count++;}
      if(k+1<bundle.data.length){
        const gap=row.next.reduce((a,v,i)=>a+Math.abs(v-bundle.data[k+1].x[i]),0);
        if(gap>1e-8)break;
      }
    }
  }
  return loss/Math.max(1,count);
}
function sysIdSensitivity(bundle,theta){
  const sc=[1,2,.3,3],eps=[1e-3,2e-4,5e-4,5e-4],cols=theta.map(()=>[]);
  for(const row of bundle.data){
    for(let j=0;j<theta.length;j++){
      const tp=theta.slice(),tm=theta.slice();tp[j]+=eps[j];tm[j]-=eps[j];
      const pp=Lab.nonlinearStep(row.x,row.u,bundle.spec,{mc:tp[0],mp:tp[1],l:tp[2],friction:tp[3]});
      const pm=Lab.nonlinearStep(row.x,row.u,bundle.spec,{mc:tm[0],mp:tm[1],l:tm[2],friction:tm[3]});
      for(let i=0;i<4;i++){let d=pp[i]-pm[i];if(i===2)d=Lab.wrap(d);cols[j].push(d/(2*eps[j]*sc[i]));}
    }
  }
  const norms=cols.map(c=>Math.sqrt(c.reduce((a,v)=>a+v*v,0))),corr=theta.map(()=>theta.map(()=>0));let maxCorr=0;
  for(let i=0;i<theta.length;i++)for(let j=0;j<theta.length;j++){
    corr[i][j]=cols[i].reduce((a,v,k)=>a+v*cols[j][k],0)/Math.max(1e-15,norms[i]*norms[j]);
    if(i!==j)maxCorr=Math.max(maxCorr,Math.abs(corr[i][j]));
  }
  return {parameterOrder:['mc','mp','l','friction'],sensitivityNorms:norms,correlation:corr,maxAbsOffDiagonalCorrelation:maxCorr,
    warning:maxCorr>.98?'high parameter-sensitivity correlation: improve excitation or parameterization':null};
}
function identifyPlant(opts={}){
  const train=generateSysIdData({seed:opts.seed||7,steps:opts.steps||260});
  const result=cemOptimize({bounds:[[.7,1.5],[.05,.16],[.4,.68],[0,.25]],objective:x=>sysIdLoss(x,train),seed:17,iterations:7,population:22,initial:[1,.1,.5,.05]});
  const test=generateSysIdData({seed:71,steps:180}),base=[1,.1,.5,0],identified=result.best.x;
  return {trueParams:train.trueParams,identified:{mc:identified[0],mp:identified[1],l:identified[2],friction:identified[3]},trainLoss:result.best.score,
    heldout:{nominalLoss:sysIdLoss(base,test),identifiedLoss:sysIdLoss(identified,test),nominalRolloutLoss:sysIdRolloutLoss(base,test),identifiedRolloutLoss:sysIdRolloutLoss(identified,test)},
    identifiability:sysIdSensitivity(train,identified),history:result.history};
}
function generateActuatorIdData({seed=19,steps=280}={}){
  const plant=new Lab.LabPlant({seed,scenario:'actuator_id'}),rng=new RNG(seed+701),data=[];plant.reset([0,0,0,0]);
  for(let k=0;k<steps;k++){
    const u=1.6*Math.sin(.09*k)+.7*Math.sin(.031*k)+(rng.uniform()-.5)*.5,out=plant.step(u);
    data.push({u,applied:out.appliedCommand});
  }
  return {trueParams:{gain:plant.actuatorGain,alpha:plant.actuatorAlpha,delaySteps:plant.delaySteps},data};
}
function actuatorIdLoss(theta,delay,bundle){
  const [gain,alpha]=theta,queue=Array(delay).fill(0);let state=0,loss=0;
  for(const row of bundle.data){queue.push(row.u);const delayed=queue.shift();state+=alpha*(gain*delayed-state);loss+=(state-row.applied)**2;}
  return loss/Math.max(1,bundle.data.length);
}
function identifyActuator(){
  const train=generateActuatorIdData({seed:19,steps:280}),test=generateActuatorIdData({seed:29,steps:220});let best=null;
  for(let delay=0;delay<=3;delay++){
    const fit=cemOptimize({bounds:[[.7,1.05],[.2,.95]],objective:x=>actuatorIdLoss(x,delay,train),seed:90+delay,iterations:6,population:18,initial:[.9,.5]});
    const cand={delay,gain:fit.best.x[0],alpha:fit.best.x[1],trainLoss:fit.best.score,history:fit.history};
    if(!best||cand.trainLoss<best.trainLoss)best=cand;
  }
  const baseline=[1,1],baselineDelay=0;
  return {trueParams:train.trueParams,identified:{gain:best.gain,alpha:best.alpha,delaySteps:best.delay},
    trainLoss:best.trainLoss,heldout:{nominalLoss:actuatorIdLoss(baseline,baselineDelay,test),identifiedLoss:actuatorIdLoss([best.gain,best.alpha],best.delay,test)},history:best.history};
}

function episodeScore(r){
  const fail=r.failed?30:0,track=8*r.meanAbsAngle+1.2*r.maxPosition,effort=.025*r.rmsControl,est=.15*r.rmseState,act=.08*r.rmsCommandMismatch;
  return fail+track+effort+est+act;
}
function riskAggregate(scores){
  const m=mean(scores),worst=Math.max(...scores);return m+.35*(worst-m);
}
function estimatorConsistency(r){
  let nis=[],nees=[];
  for(const q of r.trace){
    if(q.S&&q.innovation){
      const a=q.S[0][0],b=q.S[0][1],c=q.S[1][0],d=q.S[1][1],det=a*d-b*c;
      if(det>1e-14){const i00=d/det,i01=-b/det,i10=-c/det,i11=a/det,v=q.innovation;nis.push(v[0]*(i00*v[0]+i01*v[1])+v[1]*(i10*v[0]+i11*v[1]));}
    }
    if(q.P){
      const e=q.truth.map((v,i)=>i===2?Lab.wrap(v-q.estimate[i]):v-q.estimate[i]);
      try{const z=Lab.solveLinearSystem(q.P,e);nees.push(e.reduce((ss,v,i)=>ss+v*z[i],0));}catch(_){}
    }
  }
  const nisMean=nis.length?mean(nis):null,neesMean=nees.length?mean(nees):null;
  const available=nis.length>0&&nees.length>0;
  const nisBounds95=[0.0506356,7.3777589],neesBounds95=[0.4844186,11.1432868];
  const coverage=(a,b)=>a.length?a.filter(v=>v>=b[0]&&v<=b[1]).length/a.length:null;
  return {available,status:available?'available':'unavailable',nisMean,neesMean,nisSamples:nis.length,neesSamples:nees.length,nisCoverage95:coverage(nis,nisBounds95),neesCoverage95:coverage(nees,neesBounds95),
    penalty:available?Math.abs(Math.log(Math.max(1e-9,nisMean/2)))+Math.abs(Math.log(Math.max(1e-9,neesMean/4))):null};
}
function controllerEval(params,split){
  const Qdiag=params.slice(0,4).map(v=>10**v),R=10**params[4],scores=[];
  for(const s of split.scenarios)for(const seed of split.seeds){
    const r=Lab.runEpisode({controller:'lqr',controllerOpts:{Qdiag,R},observer:'ekf',scenario:s,seed,steps:260,pushAt:100,pushForce:s==='nominal'?2:3});
    scores.push(episodeScore(r));
  }
  return riskAggregate(scores);
}
function tuneController(){
  const train={scenarios:['nominal','model','sensor','sim2real'],seeds:[11,12,13,14]},validation={scenarios:['mixed','sim2real'],seeds:[101,102,103]},test={scenarios:['sim2real'],seeds:[1001,1002,1003,1004,1005]};
  const baseline=[Math.log10(2),Math.log10(.5),Math.log10(55),Math.log10(3),Math.log10(.12)];
  const result=cemOptimize({bounds:[[-.3,1],[ -1.2,.5],[1,2.3],[-.3,1.2],[-1.5,.3]],objective:x=>controllerEval(x,train),seed:31,iterations:7,population:20,initial:baseline});
  const decode=x=>({Qdiag:x.slice(0,4).map(v=>10**v),R:10**x[4]});
  const scores={train:{baseline:controllerEval(baseline,train),tuned:controllerEval(result.best.x,train)},
    validation:{baseline:controllerEval(baseline,validation),tuned:controllerEval(result.best.x,validation)},test:{baseline:controllerEval(baseline,test),tuned:controllerEval(result.best.x,test)}};
  return {baseline:decode(baseline),tuned:decode(result.best.x),accepted:scores.validation.tuned<scores.validation.baseline&&scores.test.tuned<scores.test.baseline,scores,history:result.history};
}
const BASE_Q=[2e-5,2e-3,2e-5,4e-3];
function estimatorEval(params,split){
  const Q=BASE_Q.map((q,i)=>q*10**params[i]),RScale=10**params[4],scores=[];
  for(const s of split.scenarios)for(const seed of split.seeds){
    const r=Lab.runEpisode({controller:'lqr',observer:'ekf',observerOpts:{Q,RScale},scenario:s,seed,steps:240,pushAt:999,pushForce:0});
    const consistency=estimatorConsistency(r);
    scores.push((r.failed?20:0)+r.rmseState+.012*(consistency.penalty??Infinity));
  }
  return riskAggregate(scores);
}
function calibrateEstimator(){
  const train={scenarios:['model','sensor','sim2real'],seeds:[21,22,23,24]},validation={scenarios:['mixed','sim2real'],seeds:[201,202,203]},test={scenarios:['sim2real'],seeds:[2001,2002,2003,2004,2005]};
  const baseline=[0,0,0,0,0],bounds=Array.from({length:5},()=>[-1.5,1.5]);
  const result=cemOptimize({bounds,objective:x=>estimatorEval(x,train),seed:41,iterations:7,population:20,initial:baseline});
  const decode=x=>({Q:BASE_Q.map((q,i)=>q*10**x[i]),RScale:10**x[4]});
  const scores={train:{baseline:estimatorEval(baseline,train),calibrated:estimatorEval(result.best.x,train)},
    validation:{baseline:estimatorEval(baseline,validation),calibrated:estimatorEval(result.best.x,validation)},test:{baseline:estimatorEval(baseline,test),calibrated:estimatorEval(result.best.x,test)}};
  return {baseline:decode(baseline),calibrated:decode(result.best.x),accepted:scores.validation.calibrated<scores.validation.baseline&&scores.test.calibrated<scores.test.baseline,scores,history:result.history};
}


function coDesignEval(params,split){
  const posScale=10**params[0],angleScale=10**params[1],Rc=.12*10**params[2],Qscale=10**params[3],Rscale=10**params[4],
    Qc=[2*posScale,.5*posScale,55*angleScale,3*angleScale],Qe=BASE_Q.map(q=>q*Qscale),scores=[];
  for(const scenario of split.scenarios)for(const seed of split.seeds){
    const r=Lab.runEpisode({controller:'lqr',controllerOpts:{Qdiag:Qc,R:Rc},observer:'ekf',observerOpts:{Q:Qe,RScale:Rscale},
      scenario,seed,steps:230,pushAt:90,pushForce:scenario==='sensor'?0:3});
    scores.push(episodeScore(r)+.012*(estimatorConsistency(r).penalty??Infinity));
  }
  return riskAggregate(scores);
}
function coTuneControlEstimation(){
  const train={scenarios:['nominal','model','sensor','actuator'],seeds:[61,62,63]},
    validation={scenarios:['mixed','latency'],seeds:[161,162,163]},
    test={scenarios:['sim2real'],seeds:[1601,1602,1603,1604]};
  const baseline=[0,0,0,0,0],result=cemOptimize({bounds:[[-.5,.7],[-.5,.7],[-.8,.8],[-1,1],[-1,1]],
    objective:x=>coDesignEval(x,train),seed:67,iterations:5,population:14,initial:baseline});
  const decode=x=>({controller:{Qdiag:[2*10**x[0],.5*10**x[0],55*10**x[1],3*10**x[1]],R:.12*10**x[2]},
    estimator:{Q:BASE_Q.map(q=>q*10**x[3]),RScale:10**x[4]}});
  const scores={train:{baseline:coDesignEval(baseline,train),candidate:coDesignEval(result.best.x,train)},
    validation:{baseline:coDesignEval(baseline,validation),candidate:coDesignEval(result.best.x,validation)},
    test:{baseline:coDesignEval(baseline,test),candidate:coDesignEval(result.best.x,test)}};
  return {baseline:decode(baseline),candidate:decode(result.best.x),
    accepted:scores.validation.candidate<scores.validation.baseline&&scores.test.candidate<scores.test.baseline,
    scores,history:result.history,
    boundary:'low-dimensional black-box co-tuning bridge; replace with differentiable/bilevel co-design at robot scale when tractable'};
}

function decodeMpcCandidate(c){
  const [N,rScale,posScale,angleScale]=c;
  return {
    N,
    R:.14*rScale,
    Qdiag:[2*posScale,.45*posScale,68*angleScale,3.5*angleScale],
    Qfdiag:[8*posScale,1.5*posScale,110*angleScale,7*angleScale]
  };
}
function mpcEval(c,split){
  const opts=decodeMpcCandidate(c),scores=[];
  for(const scenario of split.scenarios)for(const seed of split.seeds){
    const r=Lab.runEpisode({controller:'linear_mpc',controllerOpts:opts,observer:'ekf',scenario,seed,steps:180,pushAt:75,pushForce:scenario==='nominal'?2:3});
    scores.push(episodeScore(r));
  }
  return riskAggregate(scores);
}
function tuneMpcStructure(){
  const train={scenarios:['nominal','model','actuator'],seeds:[51,52]},
    validation={scenarios:['mixed','latency'],seeds:[151,152]},
    test={scenarios:['sim2real'],seeds:[1501,1502,1503]};
  const candidates=[];
  for(const N of [15,30,45])for(const rScale of [.6,1,1.7])for(const angleScale of [.8,1.2]){
    const c=[N,rScale,1,angleScale];candidates.push({candidate:c,score:mpcEval(c,train)});
  }
  candidates.sort((a,b)=>a.score-b.score);
  const base=[30,1,1,1],best=candidates[0].candidate;
  const scores={
    train:{baseline:mpcEval(base,train),tuned:mpcEval(best,train)},
    validation:{baseline:mpcEval(base,validation),tuned:mpcEval(best,validation)},
    test:{baseline:mpcEval(base,test),tuned:mpcEval(best,test)}
  };
  return {baseline:decodeMpcCandidate(base),tuned:decodeMpcCandidate(best),
    accepted:scores.validation.tuned<scores.validation.baseline&&scores.test.tuned<scores.test.baseline,
    scores,candidates:candidates.map(x=>({params:decodeMpcCandidate(x.candidate),trainScore:x.score}))};
}


function safetySupervisorProbe(){
  const angles=[.35,.45,.5],seeds=[721,722,723],rows=[];
  for(const theta0 of angles)for(const controller of ['full_nmpc','supervised_nmpc']){
    const rs=seeds.map(seed=>Lab.runEpisode({controller,observer:'truth',scenario:'nominal',seed,steps:300,pushAt:999,pushForce:0,initialState:[0,0,theta0,0]}));
    rows.push({controller,theta0,thetaDeg:theta0*180/Math.PI,...summarizeRuns(rs)});
  }
  const better=angles.every(theta=>{
    const a=rows.find(r=>r.controller==='full_nmpc'&&r.theta0===theta),b=rows.find(r=>r.controller==='supervised_nmpc'&&r.theta0===theta);
    return b.failures<=a.failures&&b.maxPosition<=a.maxPosition;
  });
  return {accepted:better,rows,boundary:'runtime backup bridge only; no CBF/reachability/invariant-set safety guarantee'};
}

function robustMpcProbe(){
  const scenarios=['model','sim2real'],seeds=[701,702,703],rows=[];
  for(const scenario of scenarios)for(const controller of ['linear_mpc','scenario_mpc']){
    const rs=seeds.map(seed=>Lab.runEpisode({controller,observer:'ekf',scenario,seed,steps:220,pushAt:90,pushForce:3}));
    rows.push({controller,scenario,...summarizeRuns(rs)});
  }
  const by=(c,s)=>rows.find(r=>r.controller===c&&r.scenario===s);
  const modelBase=by('linear_mpc','model'),modelRob=by('scenario_mpc','model'),simBase=by('linear_mpc','sim2real'),simRob=by('scenario_mpc','sim2real');
  const accepted=modelRob.failures<=modelBase.failures&&simRob.failures<=simBase.failures&&modelRob.maxPosition<=modelBase.maxPosition&&simRob.maxPosition<=simBase.maxPosition;
  return {accepted,rows,interpretation:accepted?
    'finite-model risk formulation improves or preserves the paired held-out stress metrics used by this probe':
    'finite-model risk formulation does not improve the paired stress probe; uncertainty-set/objective/constraint/runtime design remains inadequate'};
}

function modelHierarchySweep(){
  const controllers=['lqr','linear_mpc','state_mpc','ltv_mpc','full_nmpc'],angles=[.05,.15,.3,.45],rows=[];
  for(const controller of controllers)for(const theta0 of angles){
    const rs=[601,602,603].map(seed=>Lab.runEpisode({
      controller,observer:'truth',scenario:'nominal',seed,steps:260,pushAt:999,pushForce:0,
      initialState:[0,0,theta0,0]
    }));
    rows.push({controller,theta0,thetaDeg:theta0*180/Math.PI,...summarizeRuns(rs)});
  }
  return rows;
}

function summarizeRuns(rs){
  return {failures:rs.filter(r=>r.failed).length,rmseState:mean(rs.map(r=>r.rmseState)),maxPosition:mean(rs.map(r=>r.maxPosition)),
    rmsControl:mean(rs.map(r=>r.rmsControl)),rmsCommandMismatch:mean(rs.map(r=>r.rmsCommandMismatch)),meanSolveMs:mean(rs.map(r=>r.meanSolveMs)),
    p95SolveMs:mean(rs.map(r=>r.p95SolveMs)),deadlineMissRate:mean(rs.map(r=>r.deadlineMissRate))};
}
function stressReport(){
  const controllers=['lqr','linear_mpc','state_mpc','ltv_mpc','full_nmpc'],observers=['ekf'],
    scenarios=['nominal','model','sensor','bias','actuator','latency','mixed','sim2real'],rows=[];
  for(const controller of controllers)for(const observer of observers)for(const scenario of scenarios){
    const rs=[301,302,303].map(seed=>Lab.runEpisode({controller,observer,scenario,seed,steps:260,pushAt:100,pushForce:scenario==='nominal'?2:3}));
    rows.push({controller,observer,scenario,...summarizeRuns(rs)});
  }
  return rows;
}
function ablationReport(){
  const pipelines=[['lqr','ekf'],['full_nmpc','ekf']],scenarios=['nominal','model','sensor','bias','actuator','latency','glitch','sim2real'],out=[];
  for(const [controller,observer] of pipelines){
    const rows=scenarios.map(scenario=>{
      const rs=[401,402,403,404,405].map(seed=>Lab.runEpisode({controller,observer,scenario,seed,steps:260,pushAt:100,pushForce:scenario==='glitch'?0:3}));
      return {scenario,...summarizeRuns(rs)};
    });
    const base=rows.find(r=>r.scenario==='nominal');
    out.push({controller,observer,rows:rows.map(r=>({...r,deltaRmse:r.rmseState-base.rmseState,deltaMaxPosition:r.maxPosition-base.maxPosition,
      deltaCommandMismatch:r.rmsCommandMismatch-base.rmsCommandMismatch}))});
  }
  return out;
}

function lag1(values){
  const valid=values.filter(Number.isFinite);if(valid.length<3)return 0;const m=mean(valid);let num=0,den=0;
  for(let i=1;i<values.length;i++)if(Number.isFinite(values[i])&&Number.isFinite(values[i-1]))num+=(values[i]-m)*(values[i-1]-m);
  for(const v of valid)den+=(v-m)*(v-m);
  return den>1e-15?num/den:0;
}
function advancedFailureReport(){
  const sensorRows=[],actuatorRows=[];
  for(const scenario of ['colored','dropout','stuck'])for(const observer of ['ekf','ukf','mhe']){
    const rs=[801,802,803].map(seed=>Lab.runEpisode({controller:'lqr',observer,scenario,seed,steps:300,pushAt:120,pushForce:3}));
    const innovations=rs.flatMap(r=>[...r.trace.map(q=>q.innovation?.[0]??null),null]);
    sensorRows.push({scenario,observer,...summarizeRuns(rs),innovationLag1:lag1(innovations),
      staleSamples:rs.reduce((ss,r)=>ss+r.trace.filter(q=>q.sensorFresh===false).length,0),
      faultSamples:rs.reduce((ss,r)=>ss+r.trace.filter(q=>q.sensorFault).length,0)});
  }
  for(const scenario of ['jitter','torque_speed','thermal'])for(const controller of ['lqr','full_nmpc']){
    const steps=scenario==='thermal'?700:300;
    const rs=[811,812,813].map(seed=>Lab.runEpisode({controller,observer:'ekf',scenario,seed,steps,pushAt:120,pushForce:3}));
    actuatorRows.push({scenario,controller,...summarizeRuns(rs),
      minForceLimit:Math.min(...rs.flatMap(r=>r.trace.map(q=>q.forceLimit??10))),
      maxThermalState:Math.max(...rs.flatMap(r=>r.trace.map(q=>q.thermalState??0)))});
  }
  return {sensorRows,actuatorRows,interpretation:{
    colored:'innovation autocorrelation reveals a wrong white-noise assumption even when RMSE remains moderate',
    dropout:'stale packet metadata must be diagnosed separately from Gaussian measurement noise',
    stuck:'a plausible-but-frozen sensor value is a fault-isolation problem, not just an R tuning problem',
    jitter:'command-vs-applied mismatch exposes transport timing variation',
    torque_speed:'available input authority is state-dependent and belongs in the actuator/input constraint model',
    thermal:'available input authority becomes history-dependent and requires an actuator thermal state or derating model'}};
}
function discretizationProbe(){
  const plant=new Lab.LabPlant({seed:901,scenario:'nominal'}),spec=plant.spec,controls=Array.from({length:140},(_,k)=>4*Math.sin(.07*k)+1.5*Math.sin(.19*k)),
    levels=[1,2,4,8,16],trajectories={};
  for(const n of levels){
    let x=[0,0,.18,0],traj=[];
    for(const u of controls){x=Lab.nonlinearStepSubsteps(x,u,spec,null,n);traj.push(x.slice());}
    trajectories[n]=traj;
  }
  const ref=trajectories[16],rows=levels.slice(0,-1).map(n=>{
    let se=0,max=0,count=0;
    for(let k=0;k<ref.length;k++)for(let i=0;i<4;i++){
      let e=trajectories[n][k][i]-ref[k][i];if(i===2)e=Lab.wrap(e);se+=e*e;max=Math.max(max,Math.abs(e));count++;
    }
    return {substeps:n,dt:Lab.DT/n,rmseVs16:Math.sqrt(se/count),maxAbsErrorVs16:max};
  });
  return {referenceSubsteps:16,rows,warning:rows[0].rmseVs16>rows.at(-1).rmseVs16?
    'coarse integration materially changes the trajectory; validate integration before retuning control':
    'no monotonic refinement signal in this probe; inspect excitation/model before drawing conclusions'};
}

module.exports={cemOptimize,generateSysIdData,sysIdLoss,sysIdRolloutLoss,sysIdSensitivity,identifyPlant,generateActuatorIdData,actuatorIdLoss,identifyActuator,tuneController,coTuneControlEstimation,tuneMpcStructure,safetySupervisorProbe,robustMpcProbe,modelHierarchySweep,calibrateEstimator,stressReport,ablationReport,advancedFailureReport,discretizationProbe,estimatorConsistency};
