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
  function maxAbsDiff(A,B){let m=0;for(let i=0;i<A.length;i++)for(let j=0;j<A[i].length;j++)m=Math.max(m,Math.abs(A[i][j]-B[i][j]));return m;}
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
  function dare(A,B,Q,R,maxIter=500,tol=1e-10){
    let P=cloneM(Q),K=[[0,0,0,0]];
    for(let it=0;it<maxIter;it++){
      const Bt=T(B),BtP=mul(Bt,P),den=R+mul(BtP,B)[0][0],Knew=scale(mul(BtP,A),1/den);
      const At=T(A),Pnew=add(Q,sub(mul(mul(At,P),A),mul(mul(mul(At,P),B),Knew)));
      if(maxAbsDiff(Pnew,P)<tol){P=Pnew;K=Knew;break;}P=Pnew;K=Knew;
    }
    return {P,K};
  }
  function nominalParams(spec){return {mc:spec.mc,mp:spec.mp,l:spec.l,gain:1,tau:0,delay:0,friction:0,bias:0,pushAmp:0,pushDuration:0,pushOffset:1e9,pushPeriod:1e9,pushSign:1,noise:[0,0,0,0],thetaBias:0,maxForce:spec.force,actuator:'ideal'};}
  function nonlinearStep(state,u,spec=PlantRef.DEFAULT_SPEC,params=null){
    let s=state.slice(),p=params||nominalParams(spec);
    for(let k=0;k<4;k++)s=PlantRef.integrate(s,spec,p,u,0,.005,0).state;
    s[2]=wrap(s[2]);return s;
  }
  function numericJacobian(state,u,spec,eps=1e-5){
    const F=zeros(4,4);
    for(let j=0;j<4;j++){
      const xp=state.slice(),xm=state.slice();xp[j]+=eps;xm[j]-=eps;
      const fp=nonlinearStep(xp,u,spec),fm=nonlinearStep(xm,u,spec);
      for(let i=0;i<4;i++){let d=fp[i]-fm[i];if(i===2)d=wrap(d);F[i][j]=d/(2*eps);}
    }
    return F;
  }
  class LabPlant{
    constructor(opts={}){this.spec={...PlantRef.DEFAULT_SPEC,actuator:'ideal',force:10,friction:0,...(opts.spec||{})};this.rng=new RNG(opts.seed||17);this.sensorRng=new RNG((opts.seed||17)+991);this.scenario=opts.scenario||'nominal';this.goal=0;this.reset();}
    setScenario(name){this.scenario=name;this.configure();}
    configure(){
      const s=this.spec;this.params=nominalParams(s);this.sensorStd=[.008,.004];
      if(this.scenario==='sensor'){this.sensorStd=[.06,.025];}
      if(this.scenario==='model'){this.params={...this.params,mc:s.mc*1.25,mp:s.mp*.8,l:s.l*1.15,friction:.15};this.sensorStd=[.012,.006];}
      if(this.scenario==='mixed'){this.params={...this.params,mc:s.mc*1.2,mp:s.mp*.85,l:s.l*1.1,friction:.12};this.sensorStd=[.04,.018];}
      if(this.scenario==='glitch'){this.sensorStd=[.012,.006];}
    }
    reset(state=null){this.configure();this.s=state?state.slice():[0,0,(this.rng.uniform()-.5)*.08,0];this.steps=0;this.pushLeft=0;this.pushForce=0;this.glitch=0;this.lastSensor=null;return this.s.slice();}
    applyPush(force,steps=10){this.pushForce=force;this.pushLeft=steps;}
    measurementVariance(){return this.sensorStd.map(s=>s*s);}
    sensor(){
      let x=this.s[0]+this.sensorStd[0]*this.sensorRng.normal(),th=wrap(this.s[2]+this.sensorStd[1]*this.sensorRng.normal());
      if(this.scenario==='glitch' && (this.steps%180)>=90 && (this.steps%180)<108)x+=.35;
      const y=[x,th];this.lastSensor=y;return y;
    }
    step(u){
      const ext=this.pushLeft>0?this.pushForce:0;if(this.pushLeft>0)this.pushLeft--;
      let s=this.s.slice(),peak=0;for(let k=0;k<4;k++){const r=PlantRef.integrate(s,this.spec,this.params,clamp(u,-this.spec.force,this.spec.force),ext,.005,0);s=r.state;peak=Math.max(peak,r.drive.tractionRatio||0);}
      s[2]=wrap(s[2]);this.s=s;this.steps++;return {state:s.slice(),external:ext,peakTractionRatio:peak,failed:Math.abs(s[0])>2.4||Math.abs(s[2])>35*Math.PI/180};
    }
  }
  class PIDController{
    constructor(spec=PlantRef.DEFAULT_SPEC){this.limit=spec.force;this.i=0;this.name='PID';}
    reset(){this.i=0;}
    act(x,goal=0){const ex=x[0]-goal;this.i=clamp(this.i+ex*DT,-1,1);const u=46*x[2]+10*x[3]+2.0*ex+3.2*x[1]+.18*this.i;return clamp(u,-this.limit,this.limit);}
  }
  class LQRController{
    constructor(spec=PlantRef.DEFAULT_SPEC){this.limit=spec.force;const {A,B}=linearModel(spec),Q=diag([2,.5,55,3]);const r=dare(A,B,Q,.12);this.A=A;this.B=B;this.K=r.K[0];this.name='LQR';}
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
    constructor(spec=PlantRef.DEFAULT_SPEC){
      const m=linearModel(spec);this.A=m.A;this.B=m.B;this.limit=spec.force;this.N=30;
      this.Q=diag([2,.45,68,3.5]);this.Qf=diag([8,1.5,110,7]);this.R=.14;
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
  function numericInputJacobian(state,u,spec,eps=1e-4){
    const fp=nonlinearStep(state,u+eps,spec),fm=nonlinearStep(state,u-eps,spec),B=zeros(4,1);
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
    update(y){const yhat=mv(H,this.x),innov=[y[0]-yhat[0],y[1]-yhat[1]];if(this.wrapInnovation)innov[1]=wrap(innov[1]);const S=add(mul(mul(H,this.P),T(H)),this.R),K=mul(mul(this.P,T(H)),inv2(S));this.x=this.x.map((v,i)=>v+K[i][0]*innov[0]+K[i][1]*innov[1]);if(this.wrapInnovation)this.x[2]=wrap(this.x[2]);const I=eye(4),IKH=sub(I,mul(K,H));this.P=add(mul(mul(IKH,this.P),T(IKH)),mul(mul(K,this.R),T(K)));this.last={innovation:innov,K,S};this.prevInnovation=innov.slice();return this.x.slice();}
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
    reset(y){this.prev=y.slice();this.x=[y[0],0,y[1],0];}
    step(y){const dx=(y[0]-this.prev[0])/DT,dth=wrap(y[1]-this.prev[1])/DT;this.x=[y[0],dx,y[1],dth];this.prev=y.slice();this.P=null;this.last={innovation:[0,0]};return this.x.slice();}
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
      const base=super.step(y,u),inn=this.last.innovation||[0,0],f=[inn[0],inn[1],this.prevInnov[0],this.prevInnov[1],u/10,base[1]/3,base[2]/.25,base[3]/3,y[0]/2.4,y[1]/.25],r=mlpResidual(this.model,f);
      const corrected=[base[0]+r[0],base[1]+r[1],wrap(base[2]+r[2]),base[3]+r[3]];
      this.prevInnov=inn.slice();this.outputX=corrected.slice();this.last.residual=r;this.last.corrected=corrected.slice();return corrected;
    }
    // P belongs to the base EKF posterior, not to the neural-corrected output.
    outputCovariance(){return null;}
  }
  class AdaptiveRObserver extends SO2Observer{
    constructor(spec,opts={}){super(spec,opts);this.name='Adaptive R (FOCUS-style observation bridge)';this.baseR=cloneM(this.R);}
    reset(y=[0,0]){super.reset(y);if(this.baseR)this.R=cloneM(this.baseR);}
    update(y){const yhat=mv(H,this.x),ix=y[0]-yhat[0],it=wrap(y[1]-yhat[1]),sx=Math.sqrt(this.baseR[0][0])+1e-9,st=Math.sqrt(this.baseR[1][1])+1e-9;const wx=Math.exp(-.5*(ix/(5*sx))**2),wt=Math.exp(-.5*(it/(5*st))**2);this.R=[[this.baseR[0][0]*(1+99*(1-wx)),0],[0,this.baseR[1][1]*(1+99*(1-wt))]];const out=super.update(y);this.last.reliability=[wx,wt];return out;}
  }
  function makeController(name,spec,actor){if(name==='pid')return new PIDController(spec);if(name==='lqr')return new LQRController(spec);if(name==='mpc'||name==='linear_mpc')return new LinearMPCController(spec);if(name==='centroidal_mpc')return new CentroidalMPCController(spec);if(name==='full_nmpc')return new FullNMPCController(spec);if(name==='ppo')return new PPOController(actor,spec);throw Error('unknown controller');}
  function makeObserver(name,spec,opts={}){if(name==='truth')return new TruthObserver();if(name==='raw')return new RawObserver();if(name==='kf')return new KFObserver(spec,opts);if(name==='ekf')return new EKFObserver(spec,opts);if(name==='so2')return new SO2Observer(spec,opts);if(name==='residual')return new ResidualObserver(spec,opts);if(name==='adaptive')return new AdaptiveRObserver(spec,opts);throw Error('unknown observer');}
  function runEpisode({controller='lqr',observer='kf',scenario='nominal',seed=1,steps=500,goal=0,actor=null,residualModel=null,pushAt=120,pushForce=3}={}){
    const plant=new LabPlant({seed,scenario}),ctl=makeController(controller,plant.spec,actor);
    const obs=makeObserver(observer,plant.spec,{model:residualModel,R:plant.measurementVariance()});
    plant.goal=goal;const y0=plant.sensor();obs.reset(y0,plant.s);ctl.reset();
    let xh=obs.outputX?obs.outputX.slice():obs.x.slice(),se=0,ae=0,maxAngle=0,failed=false,solveMs=0,solveN=0,iterSum=0;
    const trace=[];
    for(let k=0;k<steps;k++){
      const uu=ctl.act(xh,goal);
      if(Number.isFinite(ctl.lastSolveMs)){solveMs+=ctl.lastSolveMs;solveN++;iterSum+=(ctl.lastIterations??0);}
      if(k===pushAt)plant.applyPush(pushForce,10);
      const out=plant.step(uu),y=plant.sensor();
      xh=obs.step(y,uu,out.state);
      const err=out.state.map((v,i)=>i===2?wrap(v-xh[i]):v-xh[i]);
      se+=err.reduce((ss,v)=>ss+v*v,0);ae+=Math.abs(out.state[2]);maxAngle=Math.max(maxAngle,Math.abs(out.state[2]));
      const Pout=obs.outputCovariance?obs.outputCovariance():obs.P;
      trace.push({k,truth:out.state.slice(),estimate:xh.slice(),measurement:y.slice(),u:uu,innovation:obs.last?.innovation?.slice?.()||[0,0],
        P:Pout?cloneM(Pout):null,baseP:obs.P?cloneM(obs.P):null,reliability:obs.last?.reliability?.slice?.()||null,external:out.external});
      if(out.failed){failed=true;break;}
    }
    return {controller,observer,scenario,steps:trace.length,failed,rmseState:Math.sqrt(se/Math.max(1,trace.length*4)),meanAbsAngle:ae/Math.max(1,trace.length),maxAngle,meanSolveMs:solveN?solveMs/solveN:0,meanSolverIterations:solveN?iterSum/solveN:0,trace};
  }

  return {DT,wrap,diag,eye,mul,mv,T,add,sub,scale,linearModel,dare,linearRollout,linearQuadraticCost,linearQuadraticGradient,solveBoxLinearMpc,nonlinearStep,numericJacobian,numericInputJacobian,LabPlant,PIDController,LQRController,LinearMPCController,CentroidalMPCController,FullNMPCController,PPOController,KFObserver,EKFObserver,SO2Observer,RawObserver,TruthObserver,ResidualObserver,AdaptiveRObserver,makeController,makeObserver,runEpisode,ppoForward,mlpResidual};
})();
if(typeof module!=='undefined')module.exports=ControlLab;
