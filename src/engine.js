'use strict';
const ControlLab = (() => {
  const PlantRef = typeof module !== 'undefined' ? require('./plant') : Plant;
  const DT = 0.02;
  const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
  const wrap=a=>{while(a>Math.PI)a-=2*Math.PI;while(a<=-Math.PI)a+=2*Math.PI;return a;};
  const zeros=(r,c)=>Array.from({length:r},()=>Array(c).fill(0));
  const eye=n=>Array.from({length:n},(_,i)=>Array.from({length:n},(_,j)=>i===j?1:0));
  const diag=a=>a.map((v,i)=>a.map((_,j)=>i===j?v:0));
  const T=A=>A[0].map((_,j)=>A.map(r=>r[j]));
  const add=(A,B)=>A.map((r,i)=>r.map((v,j)=>v+B[i][j]));
  const sub=(A,B)=>A.map((r,i)=>r.map((v,j)=>v-B[i][j]));
  const scale=(A,s)=>A.map(r=>r.map(v=>v*s));
  const mul=(A,B)=>A.map(r=>B[0].map((_,j)=>r.reduce((s,v,k)=>s+v*B[k][j],0)));
  const mv=(A,x)=>A.map(r=>r.reduce((s,v,j)=>s+v*x[j],0));
  const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
  const quad=(x,Q)=>dot(x,mv(Q,x));
  const inv2=M=>{const d=M[0][0]*M[1][1]-M[0][1]*M[1][0];if(Math.abs(d)<1e-12)throw Error('singular 2x2');return [[M[1][1]/d,-M[0][1]/d],[-M[1][0]/d,M[0][0]/d]];};
  const cloneM=A=>A.map(r=>r.slice());
  function outer(a,b){return a.map(x=>b.map(y=>x*y));}
  function choleskyPSD(A){
    const n=A.length;
    for(const jitter of [0,1e-12,1e-10,1e-8,1e-6]){
      const L=zeros(n,n);let ok=true;
      for(let i=0;i<n&&ok;i++)for(let j=0;j<=i;j++){
        let v=A[i][j]+(i===j?jitter:0);
        for(let k=0;k<j;k++)v-=L[i][k]*L[j][k];
        if(i===j){if(v<=0||!Number.isFinite(v)){ok=false;break;}L[i][j]=Math.sqrt(v);}
        else L[i][j]=v/L[j][j];
      }
      if(ok)return L;
    }
    throw Error('covariance is not positive definite');
  }
  function circularMean(values,weights){
    let c=0,s=0;for(let i=0;i<values.length;i++){c+=weights[i]*Math.cos(values[i]);s+=weights[i]*Math.sin(values[i]);}
    return Math.atan2(s,c);
  }
  function stabilizeCovariance(P,minDiag=1e-12){
    const S=P.map((r,i)=>r.map((v,j)=>.5*(v+P[j][i])));
    for(let i=0;i<S.length;i++)S[i][i]=Math.max(minDiag,S[i][i]);
    return S;
  }
  function maxAbsDiff(A,B){let m=0;for(let i=0;i<A.length;i++)for(let j=0;j<A[i].length;j++)m=Math.max(m,Math.abs(A[i][j]-B[i][j]));return m;}
  function solveLinearSystem(A,b){
    const n=A.length,M=A.map((r,i)=>r.slice().concat([b[i]]));
    for(let k=0;k<n;k++){
      let piv=k;for(let i=k+1;i<n;i++)if(Math.abs(M[i][k])>Math.abs(M[piv][k]))piv=i;
      if(Math.abs(M[piv][k])<1e-12)throw Error('singular linear system');
      [M[k],M[piv]]=[M[piv],M[k]];
      const d=M[k][k];for(let j=k;j<=n;j++)M[k][j]/=d;
      for(let i=0;i<n;i++)if(i!==k){const f=M[i][k];for(let j=k;j<=n;j++)M[i][j]-=f*M[k][j];}
    }
    return M.map(r=>r[n]);
  }
  class RNG{constructor(seed=1){this.s=seed>>>0;this.spare=null;}uniform(){let t=this.s=(this.s+0x6D2B79F5)>>>0;t=Math.imul(t^t>>>15,t|1);t^=t+Math.imul(t^t>>>7,t|61);return((t^t>>>14)>>>0)/4294967296;}normal(){if(this.spare!==null){const v=this.spare;this.spare=null;return v;}let x,y,q;do{x=2*this.uniform()-1;y=2*this.uniform()-1;q=x*x+y*y;}while(q===0||q>=1);const k=Math.sqrt(-2*Math.log(q)/q);this.spare=y*k;return x*k;}}
  function linearModel(spec=PlantRef.DEFAULT_SPEC,dt=DT){
    const mc=spec.mc,mp=spec.mp,l=spec.l,g=spec.gravity,D=mc+.25*mp;
    const ac=zeros(4,4),bc=[[0],[1/D],[0],[-3/(4*l*D)]];
    ac[0][1]=1;ac[1][2]=-.75*mp*g/D;ac[2][3]=1;ac[3][2]=3*(g+.75*mp*g/D)/(4*l);
    // The lab runs a 20 ms controller around a 4 x 5 ms semi-implicit plant.
    // At the actual control period, use the discrete Jacobian of that SAME
    // nonlinear transition so LQR/KF are not quietly designed for a different
    // Euler-discretized plant. Keep Ac/Bc for the educational continuous model.
    if(Math.abs(dt-DT)<1e-12){
      const xeq=[0,0,0,0],A=numericJacobian(xeq,0,spec),B=numericInputJacobian(xeq,0,spec);
      return {A,B,Ac:ac,Bc:bc,discretization:'shared-plant-jacobian'};
    }
    const A=add(eye(4),scale(ac,dt)),B=scale(bc,dt);
    return {A,B,Ac:ac,Bc:bc,discretization:'forward-euler'};
  }
  function dare(A,B,Q,R,maxIter=2000,tol=1e-10){
    let P=cloneM(Q),K=[[0,0,0,0]],iterations=0,converged=false;
    for(let it=0;it<maxIter;it++){
      const Bt=T(B),BtP=mul(Bt,P),den=R+mul(BtP,B)[0][0],Knew=scale(mul(BtP,A),1/den);
      const At=T(A),Pnew=add(Q,sub(mul(mul(At,P),A),mul(mul(mul(At,P),B),Knew)));
      iterations=it+1;if(maxAbsDiff(Pnew,P)<tol){P=Pnew;K=Knew;converged=true;break;}P=Pnew;K=Knew;
    }
    const BtP=mul(T(B),P);K=scale(mul(BtP,A),1/(R+mul(BtP,B)[0][0]));
    return {P,K,iterations,converged};
  }
  function nominalParams(spec){return {mc:spec.mc,mp:spec.mp,l:spec.l,gain:1,tau:0,delay:0,friction:0,bias:0,pushAmp:0,pushDuration:0,pushOffset:1e9,pushPeriod:1e9,pushSign:1,noise:[0,0,0,0],thetaBias:0,maxForce:spec.force,actuator:'ideal'};}
  function nonlinearStepSubsteps(state,u,spec=PlantRef.DEFAULT_SPEC,params=null,substeps=4){
    let s=state.slice(),p=params||nominalParams(spec),n=Math.max(1,Math.round(substeps)),dt=DT/n;
    for(let k=0;k<n;k++)s=PlantRef.integrate(s,spec,p,u,0,dt,0).state;
    s[2]=wrap(s[2]);return s;
  }
  function nonlinearStep(state,u,spec=PlantRef.DEFAULT_SPEC,params=null){return nonlinearStepSubsteps(state,u,spec,params,4);}
  function numericJacobian(state,u,spec,eps=1e-5,params=null){
    const F=zeros(4,4);
    for(let j=0;j<4;j++){
      const xp=state.slice(),xm=state.slice();xp[j]+=eps;xm[j]-=eps;
      const fp=nonlinearStep(xp,u,spec,params),fm=nonlinearStep(xm,u,spec,params);
      for(let i=0;i<4;i++){let d=fp[i]-fm[i];if(i===2)d=wrap(d);F[i][j]=d/(2*eps);}
    }
    return F;
  }
  class LabPlant{
    constructor(opts={}){this.spec={...PlantRef.DEFAULT_SPEC,actuator:'ideal',force:10,friction:0,...(opts.spec||{})};this.rng=new RNG(opts.seed||17);this.sensorRng=new RNG((opts.seed||17)+991);this.scenario=opts.scenario||'nominal';this.goal=0;this.reset();}
    setScenario(name){this.scenario=name;this.configure();}
    configure(){
      const s=this.spec;this.params=nominalParams(s);this.sensorStd=[.008,.004];this.delaySteps=0;this.actuatorAlpha=1;this.actuatorGain=1;this.biasWalk=[0,0];this.coloredRho=0;this.dropoutPeriod=0;this.dropoutLength=0;this.stuckStart=-1;this.stuckLength=0;this.jitterProb=0;this.torqueSpeedNoLoad=Infinity;this.minForceFraction=1;this.thermalHeat=0;this.thermalCool=0;this.thermalDerate=0;
      if(this.scenario==='sensor'){this.sensorStd=[.06,.025];}
      if(this.scenario==='model'){this.params={...this.params,mc:s.mc*1.25,mp:s.mp*.8,l:s.l*1.15,friction:.15};this.sensorStd=[.012,.006];}
      if(this.scenario==='mixed'){this.params={...this.params,mc:s.mc*1.2,mp:s.mp*.85,l:s.l*1.1,friction:.12};this.sensorStd=[.04,.018];}
      if(this.scenario==='glitch'){this.sensorStd=[.012,.006];}
      if(this.scenario==='nonlinear'){this.sensorStd=[.0005,.00025];}
      if(this.scenario==='bias'){this.biasWalk=[4e-4,1.5e-4];}
      if(this.scenario==='actuator'){this.actuatorAlpha=.42;this.actuatorGain=.9;}
      if(this.scenario==='latency'){this.delaySteps=2;}
      if(this.scenario==='colored'){this.sensorStd=[.025,.01];this.coloredRho=.94;}
      if(this.scenario==='dropout'){this.dropoutPeriod=70;this.dropoutLength=8;}
      if(this.scenario==='stuck'){this.stuckStart=90;this.stuckLength=35;}
      if(this.scenario==='jitter'){this.actuatorAlpha=.72;this.actuatorGain=.96;this.jitterProb=.18;}
      if(this.scenario==='torque_speed'){this.torqueSpeedNoLoad=2.2;this.minForceFraction=.25;}
      if(this.scenario==='thermal'){this.thermalHeat=.02;this.thermalCool=.0015;this.thermalDerate=.55;}
      if(this.scenario==='actuator_id'){this.actuatorAlpha=.52;this.actuatorGain=.88;this.delaySteps=2;}
      if(this.scenario==='sim2real'){
        const u=()=>this.rng.uniform();
        this.params={...this.params,mc:s.mc*(.8+.4*u()),mp:s.mp*(.75+.5*u()),l:s.l*(.9+.2*u()),friction:.04+.18*u()};
        this.sensorStd=[.012+.025*u(),.005+.012*u()];this.delaySteps=Math.floor(3*u());this.actuatorAlpha=.35+.45*u();this.actuatorGain=.82+.26*u();
        this.biasWalk=[2e-4*(.5+u()),8e-5*(.5+u())];
      }
    }
    defaultState(){if(this.scenario==='nonlinear')return [0,0,.5,0];return [0,0,(this.rng.uniform()-.5)*.08,0];}
    reset(state=null){this.configure();this.s=state?state.slice():this.defaultState();this.steps=0;this.pushLeft=0;this.pushForce=0;this.glitch=0;this.lastSensor=null;this.sensorBias=[0,0];this.coloredNoise=[0,0];this.delayQueue=Array(this.delaySteps).fill(0);this.actuatorState=0;this.lastDelayedCommand=0;this.lastGoodMeasurement=null;this.stuckMeasurement=null;this.sensorMeta={fresh:true,fault:null};this.thermalState=0;this.forceLimit=this.spec.force;return this.s.slice();}
    applyPush(force,steps=10){this.pushForce=force;this.pushLeft=steps;}
    measurementVariance(){return this.sensorStd.map(s=>s*s);}
    sensor(){
      this.sensorBias[0]+=this.biasWalk[0]*this.sensorRng.normal();this.sensorBias[1]+=this.biasWalk[1]*this.sensorRng.normal();
      let nx,nt;
      if(this.coloredRho>0){
        const q=Math.sqrt(Math.max(0,1-this.coloredRho*this.coloredRho));
        this.coloredNoise[0]=this.coloredRho*this.coloredNoise[0]+q*this.sensorStd[0]*this.sensorRng.normal();
        this.coloredNoise[1]=this.coloredRho*this.coloredNoise[1]+q*this.sensorStd[1]*this.sensorRng.normal();
        nx=this.coloredNoise[0];nt=this.coloredNoise[1];
      }else{nx=this.sensorStd[0]*this.sensorRng.normal();nt=this.sensorStd[1]*this.sensorRng.normal();}
      let y=[this.s[0]+this.sensorBias[0]+nx,wrap(this.s[2]+this.sensorBias[1]+nt)],fresh=true,fault=null;
      if(this.scenario==='glitch' && (this.steps%180)>=90 && (this.steps%180)<108){y[0]+=.35;fault='outlier';}
      if(this.dropoutPeriod>0 && (this.steps%this.dropoutPeriod)<this.dropoutLength && this.lastGoodMeasurement){y=this.lastGoodMeasurement.slice();fresh=false;fault='dropout-hold';}
      else if(this.stuckStart>=0 && this.steps>=this.stuckStart && this.steps<this.stuckStart+this.stuckLength){
        if(!this.stuckMeasurement)this.stuckMeasurement=y.slice();y=this.stuckMeasurement.slice();fault='stuck';
      }else this.lastGoodMeasurement=y.slice();
      this.sensorMeta={fresh,fault};this.lastSensor=y;return y;
    }
    step(u){
      const ext=this.pushLeft>0?this.pushForce:0;if(this.pushLeft>0)this.pushLeft--;
      this.delayQueue.push(clamp(u,-this.spec.force,this.spec.force));let delayed=this.delayQueue.shift();
      if(this.jitterProb>0){
        if(this.rng.uniform()<this.jitterProb)delayed=this.lastDelayedCommand;
        else this.lastDelayedCommand=delayed;
      }else this.lastDelayedCommand=delayed;
      this.actuatorState+=this.actuatorAlpha*(this.actuatorGain*delayed-this.actuatorState);
      let speedFraction=1;
      if(Number.isFinite(this.torqueSpeedNoLoad))speedFraction=Math.max(this.minForceFraction,1-Math.abs(this.s[1])/this.torqueSpeedNoLoad);
      if(this.thermalHeat>0){
        const effort=Math.min(1,Math.abs(this.actuatorState)/Math.max(1e-9,this.spec.force));
        this.thermalState=clamp(this.thermalState+this.thermalHeat*effort*effort-this.thermalCool*this.thermalState,0,1);
      }
      const thermalFraction=1-this.thermalDerate*this.thermalState;
      this.forceLimit=this.spec.force*speedFraction*thermalFraction;
      const applied=clamp(this.actuatorState,-this.forceLimit,this.forceLimit);
      let s=this.s.slice(),peak=0;for(let k=0;k<4;k++){const r=PlantRef.integrate(s,this.spec,this.params,applied,ext,.005,0);s=r.state;peak=Math.max(peak,r.drive.tractionRatio||0);}
      s[2]=wrap(s[2]);this.s=s;this.steps++;return {state:s.slice(),external:ext,appliedCommand:applied,forceLimit:this.forceLimit,thermalState:this.thermalState,peakTractionRatio:peak,failed:Math.abs(s[0])>2.4||Math.abs(s[2])>35*Math.PI/180};
    }
  }
  class PIDController{
    constructor(spec=PlantRef.DEFAULT_SPEC){this.limit=spec.force;this.i=0;this.name='PID';}
    reset(){this.i=0;}
    act(x,goal=0){const ex=x[0]-goal;this.i=clamp(this.i+ex*DT,-1,1);const u=46*x[2]+10*x[3]+2.0*ex+3.2*x[1]+.18*this.i;return clamp(u,-this.limit,this.limit);}
  }
  class LQRController{
    constructor(spec=PlantRef.DEFAULT_SPEC,opts={}){this.limit=spec.force;const {A,B}=linearModel(spec),Q=diag(opts.Qdiag||[2,.5,55,3]),R=opts.R??.12;const r=dare(A,B,Q,R);this.A=A;this.B=B;this.Q=Q;this.R=R;this.K=r.K[0];this.name='LQR';}
    reset(){}
    act(x,goal=0){const e=[x[0]-goal,x[1],wrap(x[2]),x[3]];return clamp(-dot(this.K,e),-this.limit,this.limit);}
  }

  function linearRollout(A,B,x0,U){
    const X=[x0.slice()];
    for(let k=0;k<U.length;k++)X.push(mv(A,X[k]).map((v,i)=>v+B[i][0]*U[k]));
    return X;
  }
  function linearQuadraticCost(X,U,Q,R,Qf){
    let J=quad(X.at(-1),Qf);
    for(let k=0;k<U.length;k++)J+=quad(X[k],Q)+R*U[k]*U[k];
    return J;
  }
  function linearQuadraticGradient(A,B,X,U,Q,R,Qf){
    const g=Array(U.length).fill(0),At=T(A),Bt=T(B);
    let lam=mv(Qf,X.at(-1)).map(v=>2*v);
    for(let k=U.length-1;k>=0;k--){
      g[k]=2*R*U[k]+dot(Bt[0],lam);
      const qx=mv(Q,X[k]).map(v=>2*v);
      lam=mv(At,lam).map((v,i)=>v+qx[i]);
    }
    return g;
  }
  function solveBoxLinearMpc(A,B,Q,R,Qf,x0,U0,limit,maxIter=30){
    let U=U0.slice(),X=linearRollout(A,B,x0,U),J=linearQuadraticCost(X,U,Q,R,Qf),iterations=0,step=1;
    for(let it=0;it<maxIter;it++){
      const g=linearQuadraticGradient(A,B,X,U,Q,R,Qf);let accepted=false,bestU=U,bestX=X,bestJ=J,bestStep=step;
      for(let bt=0;bt<14;bt++){
        const a=step*(.5**bt),Un=U.map((u,k)=>clamp(u-a*g[k],-limit,limit));
        const move=Un.reduce((ss,u,k)=>ss+(u-U[k])**2,0);
        if(move<1e-14)return {U,X,J,iterations,converged:true};
        const Xn=linearRollout(A,B,x0,Un),Jn=linearQuadraticCost(Xn,Un,Q,R,Qf);
        if(Number.isFinite(Jn)&&Jn<J-1e-12){accepted=true;bestU=Un;bestX=Xn;bestJ=Jn;bestStep=a;break;}
      }
      iterations++;
      if(!accepted)break;
      const improvement=J-bestJ;U=bestU;X=bestX;J=bestJ;step=Math.min(1,bestStep*1.5);
      if(improvement<1e-8)return {U,X,J,iterations,converged:true};
    }
    return {U,X,J,iterations,converged:false};
  }
  class LinearMPCController{
    constructor(spec=PlantRef.DEFAULT_SPEC,opts={}){
      const m=linearModel(spec);this.A=m.A;this.B=m.B;this.limit=spec.force;this.N=opts.N||30;
      this.Q=diag(opts.Qdiag||[2,.45,68,3.5]);this.Qf=diag(opts.Qfdiag||[8,1.5,110,7]);this.R=opts.R??.14;
      this.U=Array(this.N).fill(0);this.name='Linear MPC';this.lastSolveMs=0;this.lastIterations=0;
    }
    reset(){this.U.fill(0);this.lastPrediction=[];this.lastControls=[];this.lastIterations=0;}
    act(x,goal=0){
      const clock=typeof performance!=='undefined'?performance:Date,t0=clock.now(),x0=[x[0]-goal,x[1],wrap(x[2]),x[3]];
      const sol=solveBoxLinearMpc(this.A,this.B,this.Q,this.R,this.Qf,x0,this.U,this.limit,100);
      const out=sol.U[0];this.U=sol.U.slice(1).concat(sol.U.at(-1));
      this.lastPrediction=sol.X.map(s=>[s[0]+goal,s[1],s[2],s[3]]);this.lastControls=sol.U.slice();
      this.lastCost=sol.J;this.lastIterations=sol.iterations;this.lastConverged=sol.converged;this.lastSolveMs=clock.now()-t0;return out;
    }
  }
  function ensembleLinearObjective(models,x0,U,Q,R,Qf,risk=.5,bound=null){
    const rows=models.map(m=>{
      const X=linearRollout(m.A,m.B,x0,U);
      if(bound){const cg=constrainedLinearCost(m.A,m.B,x0,U,Q,R,Qf,bound);return {X:cg.X,J:cg.J,g:constrainedLinearGradient(m.A,m.B,cg.X,U,Q,R,Qf,bound)};}
      return {X,J:linearQuadraticCost(X,U,Q,R,Qf),g:linearQuadraticGradient(m.A,m.B,X,U,Q,R,Qf)};
    });
    const n=rows.length,meanJ=rows.reduce((s,r)=>s+r.J,0)/n,worst=rows.reduce((a,b)=>a.J>=b.J?a:b),meanG=Array(U.length).fill(0);
    for(const row of rows)for(let k=0;k<U.length;k++)meanG[k]+=row.g[k]/n;
    const g=meanG.map((v,k)=>v+risk*(worst.g[k]-v)),J=meanJ+risk*(worst.J-meanJ);
    return {rows,J,g,worstCost:worst.J,meanCost:meanJ};
  }
  class ScenarioMPCController{
    constructor(spec=PlantRef.DEFAULT_SPEC,opts={}){
      this.spec=spec;this.limit=spec.force;this.N=opts.N||30;this.Q=diag(opts.Qdiag||[3,.6,72,4]);this.Qf=diag(opts.Qfdiag||[10,2,120,8]);this.R=opts.R??.16;this.risk=opts.risk??.55;
      const variants=opts.variants||[
        {mc:1,mp:1,l:1,friction:0},
        {mc:1.25,mp:.8,l:1.15,friction:.15},
        {mc:.8,mp:1.25,l:.9,friction:.18},
        {mc:1.2,mp:.75,l:1.1,friction:.22},
        {mc:.85,mp:1.2,l:.9,friction:.05}
      ];
      this.models=variants.map(v=>{
        const p={...nominalParams(spec),mc:spec.mc*v.mc,mp:spec.mp*v.mp,l:spec.l*v.l,friction:v.friction};
        return {A:numericJacobian([0,0,0,0],0,spec,1e-5,p),B:numericInputJacobian([0,0,0,0],0,spec,1e-4,p),params:p};
      });
      this.nominalIndex=0;this.bound={index:0,soft:opts.softPosition??1.55,weight:opts.boundWeight??260,hard:opts.hardPosition??2.4};
      this.U=Array(this.N).fill(0);this.name='Scenario-risk linear MPC';this.lastSolveMs=0;this.lastIterations=0;
    }
    reset(){this.U.fill(0);this.lastPrediction=[];this.lastControls=[];this.lastIterations=0;this.lastScenarioCosts=[];}
    act(x,goal=0){
      const clock=typeof performance!=='undefined'?performance:Date,t0=clock.now(),x0=[x[0]-goal,x[1],wrap(x[2]),x[3]];
      let U=this.U.slice(),cur=ensembleLinearObjective(this.models,x0,U,this.Q,this.R,this.Qf,this.risk,this.bound),step=.6,iterations=0;
      for(let it=0;it<70;it++){
        let accepted=false;
        for(let bt=0;bt<14;bt++){
          const a=step*(.5**bt),Un=U.map((u,k)=>clamp(u-a*cur.g[k],-this.limit,this.limit));
          const move=Un.reduce((ss,u,k)=>ss+(u-U[k])**2,0);if(move<1e-14)break;
          const cand=ensembleLinearObjective(this.models,x0,Un,this.Q,this.R,this.Qf,this.risk,this.bound);
          if(Number.isFinite(cand.J)&&cand.J<cur.J-1e-12){U=Un;cur=cand;step=Math.min(1,a*1.5);accepted=true;break;}
        }
        iterations++;if(!accepted)break;
      }
      const out=U[0];this.U=U.slice(1).concat(U.at(-1));const nom=cur.rows[this.nominalIndex];
      this.lastPrediction=nom.X.map(q=>[q[0]+goal,q[1],q[2],q[3]]);this.lastControls=U.slice();this.lastScenarioCosts=cur.rows.map(r=>r.J);
      this.lastCost=cur.J;this.lastWorstCost=cur.worstCost;this.lastPredictedStateViolation=Math.max(0,...nom.X.map(q=>Math.abs(q[0])-this.bound.hard));this.lastIterations=iterations;this.lastSolveMs=clock.now()-t0;return out;
    }
  }

  function boundPenalty(x,bound){
    const idx=bound.index??0,soft=bound.soft??1.6,w=bound.weight??250,d=Math.abs(x[idx])-soft;
    if(d<=0)return {cost:0,grad:Array(x.length).fill(0)};
    const grad=Array(x.length).fill(0);grad[idx]=2*w*d*Math.sign(x[idx]);return {cost:w*d*d,grad};
  }
  function constrainedLinearCost(A,B,x0,U,Q,R,Qf,bound){
    const X=linearRollout(A,B,x0,U);let J=quad(X.at(-1),Qf)+boundPenalty(X.at(-1),bound).cost;
    for(let k=0;k<U.length;k++)J+=quad(X[k],Q)+R*U[k]*U[k]+boundPenalty(X[k],bound).cost;
    return {X,J};
  }
  function constrainedLinearGradient(A,B,X,U,Q,R,Qf,bound){
    const g=Array(U.length).fill(0),At=T(A),Bt=T(B),pgf=boundPenalty(X.at(-1),bound).grad;
    let lam=mv(Qf,X.at(-1)).map((v,i)=>2*v+pgf[i]);
    for(let k=U.length-1;k>=0;k--){
      g[k]=2*R*U[k]+dot(Bt[0],lam);
      const pg=boundPenalty(X[k],bound).grad,qx=mv(Q,X[k]).map((v,i)=>2*v+pg[i]);
      lam=mv(At,lam).map((v,i)=>v+qx[i]);
    }
    return g;
  }
  class StateAwareLinearMPCController extends LinearMPCController{
    constructor(spec=PlantRef.DEFAULT_SPEC,opts={}){
      super(spec,opts);this.bound={index:0,soft:opts.softPosition??1.55,weight:opts.boundWeight??300,hard:opts.hardPosition??2.4};
      this.name='Linear MPC + soft state bound';
    }
    act(x,goal=0){
      const clock=typeof performance!=='undefined'?performance:Date,t0=clock.now(),x0=[x[0]-goal,x[1],wrap(x[2]),x[3]];
      let U=this.U.slice(),cur=constrainedLinearCost(this.A,this.B,x0,U,this.Q,this.R,this.Qf,this.bound),iterations=0,step=.5;
      for(let it=0;it<100;it++){
        const g=constrainedLinearGradient(this.A,this.B,cur.X,U,this.Q,this.R,this.Qf,this.bound);let accepted=false;
        for(let bt=0;bt<14;bt++){
          const a=step*(.5**bt),Un=U.map((u,k)=>clamp(u-a*g[k],-this.limit,this.limit)),cand=constrainedLinearCost(this.A,this.B,x0,Un,this.Q,this.R,this.Qf,this.bound);
          if(cand.J<cur.J-1e-12){U=Un;cur=cand;step=Math.min(1,a*1.5);accepted=true;break;}
        }
        iterations++;if(!accepted)break;
      }
      const out=U[0];this.U=U.slice(1).concat(U.at(-1));this.lastPrediction=cur.X.map(q=>[q[0]+goal,q[1],q[2],q[3]]);this.lastControls=U.slice();
      this.lastCost=cur.J;this.lastIterations=iterations;this.lastConverged=true;this.lastPredictedStateViolation=Math.max(0,...cur.X.map(q=>Math.abs(q[0])-this.bound.hard));
      this.lastSolveMs=clock.now()-t0;return out;
    }
  }

  class CentroidalMPCController{
    constructor(spec=PlantRef.DEFAULT_SPEC){
      this.spec=spec;this.limit=spec.force;this.N=32;this.mass=spec.mc+spec.mp;this.alpha=spec.mp*spec.l/this.mass;
      this.A=[[1,DT],[0,1]];this.B=[[.5*DT*DT/this.mass],[DT/this.mass]];
      this.Q=diag([14,2.2]);this.Qf=diag([28,5]);this.R=.045;this.U=Array(this.N).fill(0);
      const inner=new LQRController(spec);this.innerK=inner.K.slice();this.lookahead=12;
      this.name='Centroidal-style reduced MPC';this.lastSolveMs=0;this.lastIterations=0;
    }
    reset(){this.U.fill(0);this.lastPredictionReduced=[];this.lastPrediction=[];this.lastControls=[];this.lastReference=[0,0];this.lastIterations=0;}
    reducedState(x,goal){const c=x[0]+this.alpha*Math.sin(x[2]),cd=x[1]+this.alpha*Math.cos(x[2])*x[3];return [c-goal,cd];}
    act(x,goal=0){
      const clock=typeof performance!=='undefined'?performance:Date,t0=clock.now(),z0=this.reducedState(x,goal);
      const sol=solveBoxLinearMpc(this.A,this.B,this.Q,this.R,this.Qf,z0,this.U,this.limit,60);
      const plannerForce=sol.U[0];this.U=sol.U.slice(1).concat(sol.U.at(-1));
      const look=sol.X[Math.min(this.lookahead,sol.X.length-1)],xRef=look[0]+goal,vRef=look[1];
      const e=[x[0]-xRef,x[1]-vRef,wrap(x[2]),x[3]],u=clamp(-dot(this.innerK,e),-this.limit,this.limit);
      this.lastPlannerForce=plannerForce;this.lastReference=[xRef,vRef];this.lastControls=sol.U.slice();this.lastPredictionReduced=sol.X.map(z=>[z[0]+goal,z[1]]);
      this.lastPrediction=sol.X.map((z,k)=>{const a=Math.min(1,k/Math.max(1,this.lookahead)),th=x[2]*(1-a),om=x[3]*(1-a);return [z[0]+goal,z[1],th,om];});
      this.lastCost=sol.J;this.lastIterations=sol.iterations;this.lastConverged=sol.converged;this.lastSolveMs=clock.now()-t0;return u;
    }
  }
  function numericInputJacobian(state,u,spec,eps=1e-4,params=null){
    const fp=nonlinearStep(state,u+eps,spec,params),fm=nonlinearStep(state,u-eps,spec,params),B=zeros(4,1);
    for(let i=0;i<4;i++){let d=fp[i]-fm[i];if(i===2)d=wrap(d);B[i][0]=d/(2*eps);}return B;
  }
  function symmetrize(A){return A.map((r,i)=>r.map((v,j)=>.5*(v+A[j][i])));}
  class FullNMPCController{
    constructor(spec=PlantRef.DEFAULT_SPEC){
      this.spec=spec;this.limit=spec.force;this.N=30;this.Q=diag([8,1,80,4]);this.Qf=diag([24,3,130,8]);this.R=.12;
      this.U=Array(this.N).fill(0);this.maxIterations=2;this.reg=1e-4;this.name='Full nonlinear NMPC';
    }
    reset(){this.U.fill(0);this.lastPrediction=[];this.lastControls=[];this.lastSolveMs=0;this.lastIterations=0;}
    rollout(x0,U){const X=[x0.slice()];for(let k=0;k<this.N;k++)X.push(nonlinearStep(X[k],U[k],this.spec));return X;}
    cost(X,U){let J=0;for(let k=0;k<this.N;k++)J+=quad(X[k],this.Q)+this.R*U[k]*U[k];return J+quad(X[this.N],this.Qf);}
    backward(X,U){
      let Vx=mv(this.Qf,X[this.N]).map(v=>2*v),Vxx=scale(this.Qf,2);const ks=Array(this.N),Ks=Array(this.N);
      for(let k=this.N-1;k>=0;k--){
        const A=numericJacobian(X[k],U[k],this.spec),B=numericInputJacobian(X[k],U[k],this.spec),At=T(A),Bt=T(B);
        const lx=mv(this.Q,X[k]).map(v=>2*v),lu=2*this.R*U[k];
        const Qx=mv(At,Vx).map((v,i)=>v+lx[i]),Qu=lu+dot(Bt[0],Vx);
        const Qxx=add(scale(this.Q,2),mul(mul(At,Vxx),A));
        const BV=mul(Bt,Vxx),Quu=2*this.R+mul(BV,B)[0][0]+this.reg,Qux=mul(BV,A)[0];
        const kff=-Qu/Quu,K=Qux.map(v=>-v/Quu);ks[k]=kff;Ks[k]=K;
        const newVx=Qx.map((v,i)=>v+K[i]*Quu*kff+K[i]*Qu+Qux[i]*kff);
        const outerKK=K.map(a=>K.map(b=>a*Quu*b)),outerKQ=K.map(a=>Qux.map(b=>a*b)),outerQK=Qux.map(a=>K.map(b=>a*b));
        Vxx=symmetrize(add(Qxx,add(outerKK,add(outerKQ,outerQK))));Vx=newVx;
      }
      return {ks,Ks};
    }
    act(x,goal=0){
      const clock=typeof performance!=='undefined'?performance:Date,t0=clock.now();const x0=[x[0]-goal,x[1],wrap(x[2]),x[3]];
      let U=this.U.slice(),X=this.rollout(x0,U),best=this.cost(X,U),iterations=0;
      for(let it=0;it<this.maxIterations;it++){
        const {ks,Ks}=this.backward(X,U);let accepted=false,bestLocal=best,bestU=U,bestX=X;
        for(const alpha of [1,.5,.25,.1]){
          const Un=Array(this.N),Xn=[x0.slice()];
          for(let k=0;k<this.N;k++){const dx=Xn[k].map((v,i)=>v-X[k][i]);Un[k]=clamp(U[k]+alpha*ks[k]+dot(Ks[k],dx),-this.limit,this.limit);Xn.push(nonlinearStep(Xn[k],Un[k],this.spec));}
          const J=this.cost(Xn,Un);if(Number.isFinite(J)&&J<bestLocal){bestLocal=J;bestU=Un;bestX=Xn;accepted=true;}
        }
        iterations++;if(!accepted)break;const improvement=best-bestLocal;U=bestU;X=bestX;best=bestLocal;if(improvement<1e-5)break;
      }
      const out=clamp(U[0],-this.limit,this.limit);this.U=U.slice(1).concat(U.at(-1));this.lastPrediction=X.map(s=>[s[0]+goal,s[1],s[2],s[3]]);
      this.lastControls=U.slice();this.lastCost=best;this.lastIterations=iterations;this.lastSolveMs=clock.now()-t0;return out;
    }
  }

  class SupervisedNMPCController{
    constructor(spec=PlantRef.DEFAULT_SPEC,opts={}){
      this.primary=new FullNMPCController(spec);this.backup=new LQRController(spec,opts.backup||{});
      this.positionMargin=opts.positionMargin??2.05;this.angleMargin=opts.angleMargin??(.48);
      this.limit=spec.force;this.name='NMPC + backup supervisor';this.lastBackup=false;
    }
    reset(){this.primary.reset();this.backup.reset();this.lastBackup=false;this.lastPrediction=[];this.lastControls=[];this.lastSolveMs=0;this.lastIterations=0;}
    act(x,goal=0){
      const up=this.primary.act(x,goal),unsafe=(this.primary.lastPrediction||[]).some(q=>Math.abs(q[0])>this.positionMargin||Math.abs(q[2])>this.angleMargin);
      const ub=unsafe?this.backup.act(x,goal):up;this.lastBackup=unsafe;this.lastPrediction=this.primary.lastPrediction;this.lastControls=this.primary.lastControls;
      this.lastSolveMs=this.primary.lastSolveMs;this.lastIterations=this.primary.lastIterations;this.lastCost=this.primary.lastCost;return clamp(ub,-this.limit,this.limit);
    }
  }

  class LTVMPCController extends FullNMPCController{
    constructor(spec=PlantRef.DEFAULT_SPEC,opts={}){
      super(spec);this.N=opts.N||30;this.U=Array(this.N).fill(0);
      this.Q=diag(opts.Qdiag||[6,.8,76,4]);this.Qf=diag(opts.Qfdiag||[18,2.4,120,8]);this.R=opts.R??.12;
      this.reg=opts.reg??1e-4;this.name='LTV MPC · one-step SQP/RTI bridge';
    }
    act(x,goal=0){
      const clock=typeof performance!=='undefined'?performance:Date,t0=clock.now(),x0=[x[0]-goal,x[1],wrap(x[2]),x[3]];
      let U=this.U.slice(),X=this.rollout(x0,U),best=this.cost(X,U),accepted=false,bestU=U,bestX=X,bestCost=best;
      const {ks,Ks}=this.backward(X,U);
      // One real-time-iteration style local quadratic update per control tick.
      for(const alpha of [1,.5,.25,.1]){
        const Un=Array(this.N),Xn=[x0.slice()];
        for(let k=0;k<this.N;k++){
          const dx=Xn[k].map((v,i)=>v-X[k][i]);
          Un[k]=clamp(U[k]+alpha*ks[k]+dot(Ks[k],dx),-this.limit,this.limit);
          Xn.push(nonlinearStep(Xn[k],Un[k],this.spec));
        }
        const J=this.cost(Xn,Un);
        if(Number.isFinite(J)&&J<bestCost){accepted=true;bestCost=J;bestU=Un;bestX=Xn;break;}
      }
      if(accepted){U=bestU;X=bestX;best=bestCost;}
      const out=clamp(U[0],-this.limit,this.limit);this.U=U.slice(1).concat(U.at(-1));
      this.lastPrediction=X.map(q=>[q[0]+goal,q[1],q[2],q[3]]);this.lastControls=U.slice();this.lastCost=best;
      this.lastIterations=1;this.lastConverged=accepted;this.lastSolveMs=clock.now()-t0;return out;
    }
  }

  function ppoForward(snapshot,obs){const p=snapshot.p,n=snapshot.n,h=snapshot.h,o=snapshot.o,b1=h*n,w2=b1+h,b2=w2+o*h,hidden=Array(h).fill(0),y=Array(o).fill(0);for(let i=0;i<h;i++){let z=p[b1+i];for(let j=0;j<n;j++)z+=p[i*n+j]*obs[j];hidden[i]=Math.tanh(z);}for(let i=0;i<o;i++){let z=p[b2+i];for(let j=0;j<h;j++)z+=p[w2+i*h+j]*hidden[j];y[i]=z;}const m=Math.max(...y),ex=y.map(v=>Math.exp(v-m)),sum=ex.reduce((a,b)=>a+b,0);return ex.map(v=>v/sum);}
  class PPOController{
    constructor(actor,spec=PlantRef.DEFAULT_SPEC){this.actor=actor;this.limit=spec.force;this.name='PPO';}
    reset(){}
    act(x,goal=0){const obs=[(goal-x[0])/2.4,x[1]/2.5,wrap(x[2])/(Math.PI/15),x[3]/2.5,goal/2.4],p=ppoForward(this.actor,obs);this.lastProb=p;return (p[1]>=p[0]?1:-1)*this.limit;}
  }
  const H=[[1,0,0,0],[0,0,1,0]];
  class BaseKalman{
    constructor(spec,opts={}){this.spec=spec;const m=linearModel(spec);this.A=m.A;this.B=m.B;this.R=diag(opts.R||[.008**2,.004**2]);this.Q=diag(opts.Q||[2e-5,2e-3,2e-5,4e-3]);this.P=diag([.1,1,.04,1]);this.x=[0,0,0,0];this.name='KF';this.wrapInnovation=false;}
    reset(y=[0,0]){this.x=[y[0],0,y[1],0];this.P=diag([.1,1,.04,1]);this.prevInnovation=[0,0];}
    predictLinear(u){this.x=mv(this.A,this.x).map((v,i)=>v+this.B[i][0]*u);this.P=add(mul(mul(this.A,this.P),T(this.A)),this.Q);}
    update(y){if(y===null){this.last={innovation:null,S:null,measurementUsed:false};return this.x.slice();}const yhat=mv(H,this.x),innov=[y[0]-yhat[0],y[1]-yhat[1]];if(this.wrapInnovation)innov[1]=wrap(innov[1]);const S=add(mul(mul(H,this.P),T(H)),this.R),K=mul(mul(this.P,T(H)),inv2(S));this.x=this.x.map((v,i)=>v+K[i][0]*innov[0]+K[i][1]*innov[1]);if(this.wrapInnovation)this.x[2]=wrap(this.x[2]);const I=eye(4),IKH=sub(I,mul(K,H));this.P=add(mul(mul(IKH,this.P),T(IKH)),mul(mul(K,this.R),T(K)));this.last={innovation:innov,K,S,measurementUsed:true};this.prevInnovation=innov.slice();return this.x.slice();}
    step(y,u){this.predictLinear(u);return this.update(y);}
    outputCovariance(){return this.P;}
  }
  class KFObserver extends BaseKalman{constructor(spec,opts){super(spec,opts);this.name='KF';}}
  class EKFObserver extends BaseKalman{
    constructor(spec,opts){super(spec,opts);this.name='EKF';}
    step(y,u){const before=this.x.slice(),F=numericJacobian(before,u,this.spec);this.x=nonlinearStep(before,u,this.spec);this.P=add(mul(mul(F,this.P),T(F)),this.Q);return this.update(y);}
  }
  class SO2Observer extends EKFObserver{
    constructor(spec,opts){super(spec,opts);this.name='SO(2) invariant-error bridge';this.wrapInnovation=true;}
  }
  class RawObserver{
    constructor(){this.name='Raw sensor';this.prev=null;this.x=[0,0,0,0];this.P=null;}
    reset(y){this.prev=y.slice();this.x=[y[0],0,y[1],0];this.elapsedSteps=0;}
    step(y){this.elapsedSteps++;if(y===null){this.last={innovation:null,S:null,measurementUsed:false};return this.x.slice();}const elapsed=DT*this.elapsedSteps,dx=(y[0]-this.prev[0])/elapsed,dth=wrap(y[1]-this.prev[1])/elapsed;this.x=[y[0],dx,y[1],dth];this.prev=y.slice();this.elapsedSteps=0;this.P=null;this.last={innovation:null,measurementUsed:true};return this.x.slice();}
    outputCovariance(){return null;}
  }
  class TruthObserver{constructor(){this.name='Truth / oracle';this.x=[0,0,0,0];this.P=diag([0,0,0,0]);}reset(_y,truth){this.x=truth.slice();}step(_y,_u,truth){this.x=truth.slice();return this.x.slice();}outputCovariance(){return this.P;}}
  function mlpResidual(model,features){if(!model)return [0,0,0,0];const z=features.map((v,i)=>(v-model.mean[i])/(model.std[i]||1)),h=model.b1.map((b,i)=>Math.tanh(b+dot(model.w1[i],z)));const raw=model.b2.map((b,i)=>b+dot(model.w2[i],h));return raw.map((v,i)=>v*(model.outStd?.[i]||1)+(model.outMean?.[i]||0));}
  class ResidualObserver extends SO2Observer{
    constructor(spec,opts={}){super(spec,opts);this.model=opts.model||null;this.name='Learned residual (InNKF-style)';this.prevInnov=[0,0];this.outputX=this.x.slice();}
    reset(y){super.reset(y);this.prevInnov=[0,0];this.outputX=this.x.slice();}
    step(y,u){
      // Mirror InNKF's authority boundary: the neural compensation is the
      // OUTPUT estimate only. It is not fed back into the base EKF state.
      const base=super.step(y,u);if(y===null){this.prevInnov=[0,0];this.outputX=base.slice();return base;}const inn=this.last.innovation||[0,0],f=[inn[0],inn[1],this.prevInnov[0],this.prevInnov[1],u/10,base[1]/3,base[2]/.25,base[3]/3,y[0]/2.4,y[1]/.25],r=mlpResidual(this.model,f);
      const corrected=[base[0]+r[0],base[1]+r[1],wrap(base[2]+r[2]),base[3]+r[3]];
      this.prevInnov=inn.slice();this.outputX=corrected.slice();this.last.residual=r;this.last.corrected=corrected.slice();return corrected;
    }
    // P belongs to the base EKF posterior, not to the neural-corrected output.
    outputCovariance(){return null;}
  }
  class AdaptiveRObserver extends SO2Observer{
    constructor(spec,opts={}){super(spec,opts);this.name='Adaptive R · outlier/reliability bridge';this.baseR=cloneM(this.R);}
    reset(y=[0,0]){super.reset(y);if(this.baseR)this.R=cloneM(this.baseR);}
    update(y){if(y===null)return super.update(y);const yhat=mv(H,this.x),ix=y[0]-yhat[0],it=wrap(y[1]-yhat[1]),sx=Math.sqrt(this.baseR[0][0])+1e-9,st=Math.sqrt(this.baseR[1][1])+1e-9;const wx=Math.exp(-.5*(ix/(5*sx))**2),wt=Math.exp(-.5*(it/(5*st))**2);this.R=[[this.baseR[0][0]*(1+99*(1-wx)),0],[0,this.baseR[1][1]*(1+99*(1-wt))]];const out=super.update(y);this.last.reliability=[wx,wt];return out;}
  }
  class UKFObserver{
    constructor(spec,opts={}){
      this.spec=spec;this.n=4;this.alpha=opts.alpha??.7;this.beta=opts.beta??2;this.kappa=opts.kappa??0;
      this.lambda=this.alpha*this.alpha*(this.n+this.kappa)-this.n;
      const den=this.n+this.lambda;this.Wm=[this.lambda/den];this.Wc=[this.lambda/den+(1-this.alpha*this.alpha+this.beta)];
      for(let i=0;i<2*this.n;i++){this.Wm.push(1/(2*den));this.Wc.push(1/(2*den));}
      this.R=diag(opts.R||[.008**2,.004**2]);this.Q=diag(opts.Q||[2e-5,2e-3,2e-5,4e-3]);
      this.P=diag([.1,1,.04,1]);this.x=[0,0,0,0];this.name='UKF · sigma-point nonlinear filter';
    }
    reset(y=[0,0]){this.x=[y[0],0,y[1],0];this.P=diag([.1,1,.04,1]);this.last={innovation:[0,0]};}
    sigmaPoints(){
      const L=choleskyPSD(this.P),scaleFactor=Math.sqrt(this.n+this.lambda),pts=[this.x.slice()];
      for(let j=0;j<this.n;j++){
        const d=L.map(r=>scaleFactor*r[j]),p=this.x.map((v,i)=>v+d[i]),m=this.x.map((v,i)=>v-d[i]);
        p[2]=wrap(p[2]);m[2]=wrap(m[2]);pts.push(p,m);
      }
      return pts;
    }
    step(y,u){
      const prop=this.sigmaPoints().map(x=>nonlinearStep(x,u,this.spec));
      const xm=[0,0,0,0];
      for(const i of [0,1,3])for(let k=0;k<prop.length;k++)xm[i]+=this.Wm[k]*prop[k][i];
      xm[2]=circularMean(prop.map(q=>q[2]),this.Wm);
      let Pm=cloneM(this.Q);
      for(let k=0;k<prop.length;k++){
        const d=prop[k].map((v,i)=>i===2?wrap(v-xm[i]):v-xm[i]);
        Pm=add(Pm,scale(outer(d,d),this.Wc[k]));
      }
      Pm=stabilizeCovariance(Pm);
      // The sensor is h(x)=[position, angle]. In the local angle chart,
      // project the complete predicted covariance (including additive Q).
      // Reusing only propagated pre-Q sigma points drops HQH' and QH'.
      if(y===null){this.x=xm;this.P=Pm;this.last={innovation:null,S:null,measurementUsed:false};return this.x.slice();}
      const zm=[xm[0],xm[2]],Pxz=mul(Pm,T(H)),S=add(mul(H,Pxz),this.R);
      const K=mul(Pxz,inv2(S)),innov=[y[0]-zm[0],wrap(y[1]-zm[1])];
      this.x=xm.map((v,i)=>v+K[i][0]*innov[0]+K[i][1]*innov[1]);this.x[2]=wrap(this.x[2]);
      const IKH=sub(eye(4),mul(K,H));
      this.P=stabilizeCovariance(add(mul(mul(IKH,Pm),T(IKH)),mul(mul(K,this.R),T(K))));
      this.last={innovation:innov,K,S,measurementUsed:true};return this.x.slice();
    }
    outputCovariance(){return this.P;}
  }

  class ShootingMHEObserver{
    constructor(spec,opts={}){
      this.spec=spec;this.window=opts.window||8;this.Rdiag=opts.R||[.008**2,.004**2];this.name='Nonlinear shooting MHE';
      this.base=new EKFObserver(spec,opts);this.records=[];this.x=[0,0,0,0];this.P=null;this.last={innovation:[0,0],iterations:0,cost:0};
    }
    reset(y=[0,0]){
      this.base.reset(y);this.x=this.base.x.slice();this.P=this.base.P;this.records=[{y:y.slice(),u:null,baseX:this.base.x.slice(),baseP:cloneM(this.base.P)}];
    }
    residualVector(z0){
      const first=this.records[0],error=z0.map((v,i)=>i===2?wrap(v-first.baseX[i]):v-first.baseX[i]);
      // Full whitening: ||L^-1 error||^2 = error' P^-1 error; retain correlations.
      const r=solveLinearSystem(choleskyPSD(first.baseP),error);
      let z=z0.slice(),lastInnov=null;
      for(let i=1;i<this.records.length;i++){
        z=nonlinearStep(z,this.records[i].u,this.spec);
        const yy=this.records[i].y;if(yy===null)continue;const ep=(z[0]-yy[0])/Math.sqrt(this.Rdiag[0]),et=wrap(z[2]-yy[1])/Math.sqrt(this.Rdiag[1]);
        r.push(ep,et);lastInnov=[yy[0]-z[0],wrap(yy[1]-z[2])];
      }
      return {r,state:z,innovation:lastInnov};
    }
    optimize(){
      let z=this.records[0].baseX.slice(),best=this.residualVector(z),bestCost=dot(best.r,best.r),iterations=0;
      const eps=[1e-4,1e-4,1e-5,1e-4];
      for(let it=0;it<6;it++){
        const base=this.residualVector(z),m=base.r.length,J=Array.from({length:m},()=>Array(4).fill(0));
        for(let j=0;j<4;j++){
          const zp=z.slice(),zm=z.slice();zp[j]+=eps[j];zm[j]-=eps[j];
          const rp=this.residualVector(zp).r,rm=this.residualVector(zm).r;
          for(let i=0;i<m;i++)J[i][j]=(rp[i]-rm[i])/(2*eps[j]);
        }
        const Hn=Array.from({length:4},()=>Array(4).fill(0)),g=Array(4).fill(0);
        for(let a=0;a<4;a++)for(let b=0;b<4;b++)for(let i=0;i<m;i++)Hn[a][b]+=J[i][a]*J[i][b];
        for(let a=0;a<4;a++){for(let i=0;i<m;i++)g[a]+=J[i][a]*base.r[i];Hn[a][a]+=1e-3;}
        let delta;try{delta=solveLinearSystem(Hn,g.map(v=>-v));}catch(_){break;}
        let accepted=false;
        for(const alpha of [1,.5,.25,.1]){
          const zn=z.map((v,i)=>v+alpha*delta[i]);zn[2]=wrap(zn[2]);const rr=this.residualVector(zn),c=dot(rr.r,rr.r);
          if(Number.isFinite(c)&&c<bestCost){z=zn;best=rr;bestCost=c;accepted=true;break;}
        }
        iterations++;if(!accepted||Math.sqrt(dot(delta,delta))<1e-5)break;
      }
      return {state:best.state,cost:bestCost,iterations,innovation:best.innovation};
    }
    step(y,u){
      const bx=this.base.step(y,u);this.P=this.base.P;this.records.push({y:y===null?null:y.slice(),u,baseX:bx.slice(),baseP:cloneM(this.base.P)});
      while(this.records.length>this.window+1)this.records.shift();
      if(this.records.length<3){this.x=bx.slice();this.last={innovation:this.base.last.innovation?.slice()??null,iterations:0,cost:0,measurementUsed:y!==null};return this.x.slice();}
      const sol=this.optimize();this.x=sol.state.slice();this.last={innovation:y===null?null:sol.innovation,iterations:sol.iterations,cost:sol.cost,window:this.records.length-1,measurementUsed:y!==null};return this.x.slice();
    }
    outputCovariance(){return null;}
  }

  function makeController(name,spec,actor,opts={}){if(name==='pid')return new PIDController(spec);if(name==='lqr')return new LQRController(spec,opts);if(name==='mpc'||name==='linear_mpc')return new LinearMPCController(spec,opts);if(name==='scenario_mpc')return new ScenarioMPCController(spec,opts);if(name==='state_mpc')return new StateAwareLinearMPCController(spec,opts);if(name==='ltv_mpc')return new LTVMPCController(spec,opts);if(name==='supervised_nmpc')return new SupervisedNMPCController(spec,opts);if(name==='centroidal_mpc')return new CentroidalMPCController(spec);if(name==='full_nmpc')return new FullNMPCController(spec);if(name==='ppo')return new PPOController(actor,spec);throw Error('unknown controller');}
  function makeObserver(name,spec,opts={}){if(name==='truth')return new TruthObserver();if(name==='raw')return new RawObserver();if(name==='kf')return new KFObserver(spec,opts);if(name==='ekf')return new EKFObserver(spec,opts);if(name==='so2')return new SO2Observer(spec,opts);if(name==='ukf')return new UKFObserver(spec,opts);if(name==='mhe')return new ShootingMHEObserver(spec,opts);if(name==='residual')return new ResidualObserver(spec,opts);if(name==='adaptive')return new AdaptiveRObserver(spec,opts);throw Error('unknown observer');}
  function runEpisode({controller='lqr',observer='kf',scenario='nominal',seed=1,steps=500,goal=0,actor=null,residualModel=null,controllerOpts={},observerOpts={},pushAt=120,pushForce=3,initialState=null}={}){
    const plant=new LabPlant({seed,scenario});if(initialState)plant.reset(initialState);const ctl=makeController(controller,plant.spec,actor,controllerOpts);
    const baseR=plant.measurementVariance().map(v=>v*(observerOpts.RScale??1)),obs=makeObserver(observer,plant.spec,{...observerOpts,model:residualModel,R:observerOpts.R||baseR});
    plant.goal=goal;const y0=plant.sensor();obs.reset(y0,plant.s);ctl.reset();
    let xh=obs.outputX?obs.outputX.slice():obs.x.slice(),se=0,ae=0,maxAngle=0,maxPosition=0,failed=false,solveMs=0,solveN=0,iterSum=0,solveTimes=[],sumU2=0,maxU=0,sumActErr2=0;
    const trace=[];
    for(let k=0;k<steps;k++){
      const uu=ctl.act(xh,goal);sumU2+=uu*uu;maxU=Math.max(maxU,Math.abs(uu));
      if(Number.isFinite(ctl.lastSolveMs)){solveMs+=ctl.lastSolveMs;solveN++;solveTimes.push(ctl.lastSolveMs);iterSum+=(ctl.lastIterations??0);}
      if(k===pushAt)plant.applyPush(pushForce,10);
      const out=plant.step(uu),y=plant.sensor();sumActErr2+=(uu-(out.appliedCommand??uu))**2;
      xh=obs.step(plant.sensorMeta.fresh?y:null,uu,out.state);
      const err=out.state.map((v,i)=>i===2?wrap(v-xh[i]):v-xh[i]);
      se+=err.reduce((ss,v)=>ss+v*v,0);ae+=Math.abs(out.state[2]);maxAngle=Math.max(maxAngle,Math.abs(out.state[2]));maxPosition=Math.max(maxPosition,Math.abs(out.state[0]));
      const Pout=obs.outputCovariance?obs.outputCovariance():obs.P;
      trace.push({k,truth:out.state.slice(),estimate:xh.slice(),measurement:y.slice(),u:uu,innovation:obs.last?.innovation?.slice?.()??null,measurementUsed:obs.last?.measurementUsed??plant.sensorMeta.fresh,
        P:Pout?cloneM(Pout):null,baseP:obs.P?cloneM(obs.P):null,S:obs.last?.S?cloneM(obs.last.S):null,reliability:obs.last?.reliability?.slice?.()||null,observerIterations:obs.last?.iterations??0,observerCost:obs.last?.cost??0,observerWindow:obs.last?.window??0,external:out.external,appliedCommand:out.appliedCommand??uu,forceLimit:out.forceLimit??plant.spec.force,thermalState:out.thermalState??0,sensorFresh:plant.sensorMeta?.fresh??true,sensorFault:plant.sensorMeta?.fault??null});
      if(out.failed){failed=true;break;}
    }
    const sortedSolve=solveTimes.slice().sort((a,b)=>a-b),p95SolveMs=sortedSolve.length?sortedSolve[Math.min(sortedSolve.length-1,Math.ceil(.95*sortedSolve.length)-1)]:0;
    const deadlineMs=DT*1000,deadlineMisses=solveTimes.filter(v=>v>deadlineMs).length;
    return {controller,observer,scenario,informationProvenance:{measurementCovariance:observerOpts.R?'provided':'injected-noise-oracle',runtimeState:observer==='truth'?'simulation-truth':'sensor-estimate',predictionModel:'nominal-model'},steps:trace.length,failed,rmseState:Math.sqrt(se/Math.max(1,trace.length*4)),meanAbsAngle:ae/Math.max(1,trace.length),maxAngle,maxPosition,rmsControl:Math.sqrt(sumU2/Math.max(1,trace.length)),maxControl:maxU,rmsCommandMismatch:Math.sqrt(sumActErr2/Math.max(1,trace.length)),meanSolveMs:solveN?solveMs/solveN:0,p95SolveMs,deadlineMs,deadlineMisses,deadlineMissRate:solveN?deadlineMisses/solveN:0,meanSolverIterations:solveN?iterSum/solveN:0,trace};
  }

  return {DT,wrap,diag,eye,mul,mv,T,add,sub,scale,linearModel,dare,linearRollout,linearQuadraticCost,linearQuadraticGradient,solveBoxLinearMpc,ensembleLinearObjective,solveLinearSystem,nonlinearStep,nonlinearStepSubsteps,numericJacobian,numericInputJacobian,LabPlant,PIDController,LQRController,LinearMPCController,ScenarioMPCController,StateAwareLinearMPCController,SupervisedNMPCController,LTVMPCController,CentroidalMPCController,FullNMPCController,PPOController,KFObserver,EKFObserver,SO2Observer,UKFObserver,ShootingMHEObserver,RawObserver,TruthObserver,ResidualObserver,AdaptiveRObserver,makeController,makeObserver,runEpisode,ppoForward,mlpResidual};
})();
if(typeof module!=='undefined')module.exports=ControlLab;
