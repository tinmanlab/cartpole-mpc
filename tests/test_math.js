'use strict';
const assert=require('assert'),fs=require('fs'),Lab=require('../src/engine');
const actor=JSON.parse(fs.readFileSync('./assets/ppo_actor.json','utf8'));
const residual=JSON.parse(fs.readFileSync('./assets/residual_model.json','utf8'));

function maxAbs(a,b){let m=0;for(let i=0;i<a.length;i++)m=Math.max(m,Math.abs(a[i]-b[i]));return m;}
function rank(M,tol=1e-10){
  const A=M.map(r=>r.slice());let r=0;
  for(let c=0;c<A[0].length&&r<A.length;c++){
    let p=r;for(let i=r+1;i<A.length;i++)if(Math.abs(A[i][c])>Math.abs(A[p][c]))p=i;
    if(Math.abs(A[p][c])<tol)continue;
    [A[p],A[r]]=[A[r],A[p]];const d=A[r][c];for(let j=c;j<A[0].length;j++)A[r][j]/=d;
    for(let i=0;i<A.length;i++)if(i!==r){const f=A[i][c];for(let j=c;j<A[0].length;j++)A[i][j]-=f*A[r][j];}
    r++;
  }
  return r;
}
function mean(xs){return xs.reduce((s,x)=>s+x,0)/xs.length;}

const plant=new Lab.LabPlant({seed:2,scenario:'nominal'});
const model=Lab.linearModel(plant.spec);
assert.equal(model.discretization,'shared-plant-jacobian');

// Independent SciPy solve_discrete_are audit reference generated from the same A/B/Q/R.
const expectedK=[-3.52560714,-5.65028500,-50.97093669,-12.80462115];
const lqr=new Lab.LQRController(plant.spec);
assert(maxAbs(lqr.K,expectedK)<3e-7,'DARE gain drifted from independent SciPy reference');

// Direct measurements are position and angle. The four-state local model must be observable.
const H=[[1,0,0,0],[0,0,1,0]], A=model.A;
const HA=Lab.mul(H,A),HA2=Lab.mul(HA,A),HA3=Lab.mul(HA2,A);
assert.equal(rank(H.concat(HA,HA2,HA3)),4,'upright [p,theta] measurement model should have rank-4 observability');

// The same local pair must be controllable for the LQR design to stabilize all four states.
const B=model.B,AB=Lab.mul(A,B),A2B=Lab.mul(A,AB),A3B=Lab.mul(A,A2B);
const C=A.map((_,i)=>[B[i][0],AB[i][0],A2B[i][0],A3B[i][0]]);
assert.equal(rank(C),4,'upright (A,B) should have rank-4 controllability');

// Analytic adjoint gradient for the linear MPC cost must agree with central finite differences.
const U=Array.from({length:8},(_,i)=>.2*Math.sin(.7*i)),x0=[.12,-.08,.045,.03];
const Q=Lab.diag([2,.45,68,3.5]),Qf=Lab.diag([8,1.5,110,7]),R=.14;
const X=Lab.linearRollout(model.A,model.B,x0,U),g=Lab.linearQuadraticGradient(model.A,model.B,X,U,Q,R,Qf),eps=1e-6,gn=[];
for(let i=0;i<U.length;i++){
  const up=U.slice(),um=U.slice();up[i]+=eps;um[i]-=eps;
  const jp=Lab.linearQuadraticCost(Lab.linearRollout(model.A,model.B,x0,up),up,Q,R,Qf);
  const jm=Lab.linearQuadraticCost(Lab.linearRollout(model.A,model.B,x0,um),um,Q,R,Qf);
  gn.push((jp-jm)/(2*eps));
}
assert(maxAbs(g,gn)<2e-5,'linear MPC adjoint gradient disagrees with finite differences');

// Box-constrained linear MPC must decrease cost from a zero warm start and obey bounds.
const linear=new Lab.LinearMPCController(plant.spec);linear.reset();
const xlin=[.3,.1,.08,-.05],zero=Array(linear.N).fill(0);
const j0=Lab.linearQuadraticCost(Lab.linearRollout(linear.A,linear.B,xlin,zero),zero,linear.Q,linear.R,linear.Qf);
linear.act(xlin,0);
assert(linear.lastCost<=j0+1e-9);
assert(linear.lastControls.every(u=>Math.abs(u)<=linear.limit+1e-12));

// Reduced planner must solve a real bounded horizon and produce a downstream reference.
const cent=new Lab.CentroidalMPCController(plant.spec);cent.reset();cent.act([.2,.1,.06,-.02],0);
assert.equal(cent.lastPredictionReduced.length,33);
assert.equal(cent.lastReference.length,2);
assert(cent.lastControls.every(u=>Math.abs(u)<=cent.limit+1e-12));

// Full NMPC must reduce the zero-sequence nonlinear cost and obey its input bound.
const full=new Lab.FullNMPCController(plant.spec);full.reset();const xn=[.2,.05,.07,-.04],uz=Array(full.N).fill(0),xz=full.rollout(xn,uz),jn0=full.cost(xz,uz);
full.act(xn,0);
assert(full.lastCost<=jn0+1e-8,'full NMPC failed to reduce its local horizon cost');
assert(full.lastControls.every(u=>Math.abs(u)<=full.limit+1e-12));
assert(full.lastIterations>=1&&full.lastIterations<=2);

// Raw finite-difference output has no calibrated covariance model.
const raw=new Lab.RawObserver();raw.reset([0,0]);raw.step([.01,.001]);
assert.equal(raw.P,null);
assert.equal(raw.outputCovariance(),null);

// Adaptive-R reset must restore its baseline covariance.
const arPlant=new Lab.LabPlant({seed:4,scenario:'glitch'}),ar=new Lab.AdaptiveRObserver(arPlant.spec,{R:arPlant.measurementVariance()});
ar.reset(arPlant.sensor());ar.R=[[99,0],[0,99]];ar.reset([0,0]);
assert(Math.abs(ar.R[0][0]-ar.baseR[0][0])<1e-15);
assert(Math.abs(ar.R[1][1]-ar.baseR[1][1])<1e-15);

// Sensor R must match the actually injected Gaussian standard deviation in each scenario.
for(const scenario of ['nominal','sensor','model','mixed','glitch']){
  const p=new Lab.LabPlant({seed:4,scenario}),r=p.measurementVariance(),kf=new Lab.KFObserver(p.spec,{R:r});
  assert(Math.abs(kf.R[0][0]-p.sensorStd[0]**2)<1e-15);
  assert(Math.abs(kf.R[1][1]-p.sensorStd[1]**2)<1e-15);
}

// Joseph covariance update should remain symmetric and non-negative in sampled quadratic forms.
const pk=new Lab.LabPlant({seed:11,scenario:'sensor'}),kf=new Lab.KFObserver(pk.spec,{R:pk.measurementVariance()});
kf.reset(pk.sensor());let u=0;
for(let k=0;k<120;k++){const out=pk.step(u),y=pk.sensor();kf.step(y,u);u=lqr.act(kf.x,0);}
for(let i=0;i<4;i++)for(let j=0;j<4;j++)assert(Math.abs(kf.P[i][j]-kf.P[j][i])<1e-10);
for(const z of [[1,0,0,0],[0,1,0,0],[1,-2,.5,.3],[-.4,.8,-1,2]]){
  const q=z.reduce((s,zi,i)=>s+zi*kf.P[i].reduce((t,pij,j)=>t+pij*z[j],0),0);
  assert(q>=-1e-10,'P lost positive-semidefinite behavior');
}

// Timestamp alignment regression: the oracle estimate and truth must be exactly the same time sample.
const truthRun=Lab.runEpisode({controller:'lqr',observer:'truth',scenario:'nominal',seed:3,steps:120,actor,residualModel:residual,pushAt:60,pushForce:2});
assert.equal(truthRun.rmseState,0,'oracle RMSE exposes a timestamp mismatch');

// Residual correction is output-only. Its base EKF covariance is not a corrected-output covariance.
const rr=Lab.runEpisode({controller:'lqr',observer:'residual',scenario:'glitch',seed:7,steps:120,actor,residualModel:residual,pushAt:60,pushForce:2});
assert(rr.trace.every(q=>q.P===null));
assert(rr.trace.every(q=>Array.isArray(q.baseP)));
assert.equal(residual.schema,'cartpole-control-observer-residual/v2');
assert(residual.metrics.residualRmse<residual.metrics.baseRmse);

// Adaptive-R is an outlier bridge, not a generic high-noise improvement claim.
const seeds=[1,2,3,4,5];
const glitchKF=seeds.map(seed=>Lab.runEpisode({controller:'lqr',observer:'kf',scenario:'glitch',seed,steps:300,actor,residualModel:residual,pushAt:120,pushForce:3}).rmseState);
const glitchAdaptive=seeds.map(seed=>Lab.runEpisode({controller:'lqr',observer:'adaptive',scenario:'glitch',seed,steps:300,actor,residualModel:residual,pushAt:120,pushForce:3}).rmseState);
assert(mean(glitchAdaptive)<mean(glitchKF),'Adaptive-R fixed glitch probe should down-weight the injected outlier');

console.log(JSON.stringify({
  dareMaxAbsError:maxAbs(lqr.K,expectedK),
  observabilityRank:rank(H.concat(HA,HA2,HA3)),
  controllabilityRank:rank(C),
  linearMpcGradientMaxAbsError:maxAbs(g,gn),
  truthRmse:truthRun.rmseState,
  residualOffline:residual.metrics,
  glitch:{kf:mean(glitchKF),adaptiveR:mean(glitchAdaptive)}
},null,2));
