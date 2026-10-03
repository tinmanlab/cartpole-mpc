'use strict';
const fs=require('fs'),path=require('path'),Lab=require('../src/engine');
const OUT=path.join(__dirname,'..','assets','residual_model.json');

class RNG{
  constructor(seed=1){this.s=seed>>>0;this.spare=null;}
  uniform(){let t=this.s=(this.s+0x6D2B79F5)>>>0;t=Math.imul(t^t>>>15,t|1);t^=t+Math.imul(t^t>>>7,t|61);return((t^t>>>14)>>>0)/4294967296;}
  normal(){if(this.spare!==null){const v=this.spare;this.spare=null;return v;}let x,y,q;do{x=2*this.uniform()-1;y=2*this.uniform()-1;q=x*x+y*y;}while(q===0||q>=1);const k=Math.sqrt(-2*Math.log(q)/q);this.spare=y*k;return x*k;}
}
class MLP{
  constructor(rng,n=10,h=20,o=4){this.n=n;this.h=h;this.o=o;this.w1=0;this.b1=h*n;this.w2=this.b1+h;this.b2=this.w2+o*h;this.p=new Float64Array(this.b2+o);this.m=new Float64Array(this.p.length);this.v=new Float64Array(this.p.length);this.t=0;for(let i=0;i<h*n;i++)this.p[i]=rng.normal()*Math.sqrt(2/(n+h));for(let i=this.w2;i<this.b2;i++)this.p[i]=rng.normal()*.02;}
  forward(x){const h=new Float64Array(this.h),y=new Float64Array(this.o);for(let i=0;i<this.h;i++){let z=this.p[this.b1+i];for(let j=0;j<this.n;j++)z+=this.p[i*this.n+j]*x[j];h[i]=Math.tanh(z);}for(let i=0;i<this.o;i++){let z=this.p[this.b2+i];for(let j=0;j<this.h;j++)z+=this.p[this.w2+i*this.h+j]*h[j];y[i]=z;}return{x,h,y};}
  backward(f,dy,G,scale=1){const dz=new Float64Array(this.h);for(let i=0;i<this.o;i++){G[this.b2+i]+=dy[i]*scale;for(let j=0;j<this.h;j++)G[this.w2+i*this.h+j]+=dy[i]*f.h[j]*scale;}for(let j=0;j<this.h;j++){let d=0;for(let i=0;i<this.o;i++)d+=this.p[this.w2+i*this.h+j]*dy[i];dz[j]=d*(1-f.h[j]*f.h[j]);G[this.b1+j]+=dz[j]*scale;for(let k=0;k<this.n;k++)G[j*this.n+k]+=dz[j]*f.x[k]*scale;}}
  adam(G,lr=.001,maxNorm=2){let norm=0;for(const g of G)norm+=g*g;norm=Math.sqrt(norm);const scale=Math.min(1,maxNorm/(norm+1e-12));this.t++;const bc1=1-.9**this.t,bc2=1-.999**this.t;for(let i=0;i<this.p.length;i++){const g=G[i]*scale;this.m[i]=.9*this.m[i]+.1*g;this.v[i]=.999*this.v[i]+.001*g*g;this.p[i]-=lr*(this.m[i]/bc1)/(Math.sqrt(this.v[i]/bc2)+1e-8);}}
}
function features(inn,prev,u,base,y){return [inn[0],inn[1],prev[0],prev[1],u/10,base[1]/3,base[2]/.25,base[3]/3,y[0]/2.4,y[1]/.25];}
function collect(seed0,count,scenarios){
  const rows=[];
  for(let e=0;e<count;e++){
    const scenario=scenarios[e%scenarios.length],plant=new Lab.LabPlant({seed:seed0+e,scenario}),obs=new Lab.SO2Observer(plant.spec,{R:plant.measurementVariance()}),ctl=new Lab.LQRController(plant.spec);
    let y=plant.sensor();obs.reset(y);ctl.reset();let base=obs.x.slice(),prev=[0,0];
    for(let k=0;k<360;k++){
      const u=ctl.act(base,0);
      if(k===90+(e%7)*6)plant.applyPush((e%2?1:-1)*(2+(e%4)),10);
      const out=plant.step(u);y=plant.sensor();base=obs.step(y,u,out.state);
      const inn=obs.last.innovation.slice(),target=[out.state[0]-base[0],out.state[1]-base[1],Lab.wrap(out.state[2]-base[2]),out.state[3]-base[3]];
      if(k>12&&!target.some(v=>!Number.isFinite(v))&&Math.abs(out.state[2])<.55)rows.push({f:features(inn,prev,u,base,y),t:target});
      prev=inn;if(out.failed)break;
    }
  }
  return rows;
}
function stat(rows,key,n){const mean=Array(n).fill(0),std=Array(n).fill(0);for(const r of rows)for(let i=0;i<n;i++)mean[i]+=r[key][i];for(let i=0;i<n;i++)mean[i]/=rows.length;for(const r of rows)for(let i=0;i<n;i++)std[i]+=(r[key][i]-mean[i])**2;for(let i=0;i<n;i++)std[i]=Math.sqrt(std[i]/rows.length)+1e-9;return{mean,std};}
function evaluate(m,rows,xs,ys){let base=0,net=0;for(const r of rows){const x=r.f.map((v,i)=>(v-xs.mean[i])/xs.std[i]),pred=m.forward(x).y;for(let i=0;i<4;i++){const p=pred[i]*ys.std[i]+ys.mean[i];base+=r.t[i]*r.t[i];net+=(r.t[i]-p)**2;}}return{baseRmse:Math.sqrt(base/(rows.length*4)),residualRmse:Math.sqrt(net/(rows.length*4))};}

const trainRows=collect(1000,80,['nominal','sensor','model','mixed','glitch']);
const valRows=collect(9000,20,['nominal','sensor','model','mixed','glitch']);
const xs=stat(trainRows,'f',10),ys=stat(trainRows,'t',4),rng=new RNG(4242),m=new MLP(rng,10,20,4),order=Array.from({length:trainRows.length},(_,i)=>i),batch=128;
let best=null,bestParams=null;
for(let epoch=0;epoch<55;epoch++){
  for(let i=order.length-1;i>0;i--){const j=Math.floor(rng.uniform()*(i+1)),t=order[i];order[i]=order[j];order[j]=t;}
  for(let s=0;s<order.length;s+=batch){const G=new Float64Array(m.p.length),end=Math.min(order.length,s+batch),scale=1/(end-s);for(let kk=s;kk<end;kk++){const r=trainRows[order[kk]],x=r.f.map((v,i)=>(v-xs.mean[i])/xs.std[i]),y=r.t.map((v,i)=>(v-ys.mean[i])/ys.std[i]),f=m.forward(x),dy=f.y.map((v,i)=>2*(v-y[i]));m.backward(f,dy,G,scale);}m.adam(G,.0012,2);}
  const metrics=evaluate(m,valRows,xs,ys);
  if(!best||metrics.residualRmse<best.residualRmse){best=metrics;bestParams=Array.from(m.p);}
  if(epoch%10===0||epoch===54)console.log('epoch',epoch,metrics);
}
m.p.set(bestParams);
const w1=Array.from({length:m.h},(_,i)=>Array.from(m.p.slice(i*m.n,(i+1)*m.n))),b1=Array.from(m.p.slice(m.b1,m.b1+m.h)),w2=Array.from({length:m.o},(_,i)=>Array.from(m.p.slice(m.w2+i*m.h,m.w2+(i+1)*m.h))),b2=Array.from(m.p.slice(m.b2,m.b2+m.o)),metrics=evaluate(m,valRows,xs,ys);
const artifact={schema:'cartpole-control-observer-residual/v2',kind:'InNKF-style educational output residual MLP',timingContract:'post-update residual for x_(k+1) after y_(k+1); correction is output-only and is not fed back into the base EKF',input:['innovation_x','innovation_theta','prev_innovation_x','prev_innovation_theta','u_div_10','vhat_div_3','theta_hat_div_.25','omega_hat_div_3','measurement_x_div_2.4','measurement_theta_div_.25'],output:['dx','dv','dtheta','domega'],hidden:m.h,trainEpisodes:80,validationEpisodes:20,trainSamples:trainRows.length,validationSamples:valRows.length,trainScenarios:['nominal','sensor','model','mixed','glitch'],validationSeeds:'9000..9019',mean:xs.mean,std:xs.std,outMean:ys.mean,outStd:ys.std,w1,b1,w2,b2,metrics};
fs.writeFileSync(OUT,JSON.stringify(artifact,null,2));console.log('wrote',OUT,metrics);
