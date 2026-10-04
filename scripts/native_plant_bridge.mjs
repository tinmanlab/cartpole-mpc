// Optional offline stdin/stdout experiment driver. No network port or hardware I/O.
// MuJoCo WASM, LabPlant, sensor noise and EKF remain the existing implementations.
import readline from 'node:readline';
import {createRequire} from 'node:module';
import {createMujocoBackend} from '../src/mujoco_backend.mjs';
const require=createRequire(import.meta.url),L=require('../src/engine');
const backend=await createMujocoBackend();L.setPhysicsBackend(backend);
let plant,observer,controller,estimate,goal;
function snapshot(){return {estimate:estimate.slice(),truth:plant.s.slice(),k:plant.steps,
  measurementCovarianceSource:'injected-noise-oracle',physics:L.physicsInfo()};}
const lines=readline.createInterface({input:process.stdin,crlfDelay:Infinity});
for await(const line of lines){
 try{
  const q=JSON.parse(line);let out;
  if(q.op==='reset'){
   if(!['nominal','mixed','sim2real'].includes(q.scenario)||!Number.isInteger(q.seed)||!Number.isFinite(q.goal))throw Error('Invalid case');
   plant=new L.LabPlant({seed:q.seed,scenario:q.scenario});goal=q.goal;plant.goal=goal;
   observer=new L.EKFObserver(plant.spec,{R:plant.measurementVariance()});controller=L.makeController('hard_mpc',plant.spec);
   if(q.terminalMatrix){
    if(q.terminalMatrix.length!==4||q.terminalMatrix.some(row=>row.length!==4||!row.every(Number.isFinite)))throw Error('Invalid terminal matrix');
    controller.Qf=q.terminalMatrix.map(row=>row.slice());
   }
   observer.reset(plant.sensor());controller.reset();estimate=observer.x.slice();out=snapshot();
  }else if(q.op==='step'){
   if(!plant)throw Error('Reset first');const before=performance.now();let u;
   if(q.baseline)u=controller.act(estimate,goal);else u=q.u;
   if(!Number.isFinite(u)||Math.abs(u)>10+1e-7)throw Error('Invalid force');
   const solveMs=performance.now()-before;
   if(plant.steps===96)plant.applyPush(3,10);
   const p=plant.step(u),y=plant.sensor();estimate=observer.step(plant.sensorMeta.fresh?y:null,u,p.state);
   out={...snapshot(),u,applied:p.appliedCommand,failed:p.failed,baselineSolveMs:q.baseline?solveMs:null};
  }else if(q.op==='models'){
   const spec=new L.LabPlant().spec;const cases=[];
   for(const p of [{mc:1,mp:.1,l:.5},{mc:1.25,mp:.08,l:.575},{mc:.8,mp:.14,l:.42}])
    for(const x of [[0,0,0,0],[.2,.5,.4,-.7],[-.3,-.4,-.6,.8],[0,.8,1.2,1.4]])for(const u of [-7,0,5]){
     const s={...spec,...p};cases.push({p,x,u,next:L.nonlinearStep(x,u,s),A:L.numericJacobian(x,u,s),B:L.numericInputJacobian(x,u,s)});
    }
   out={cases,physics:L.physicsInfo()};
  }else if(q.op==='close'){process.stdout.write(JSON.stringify({ok:true})+'\n');break;}
  else throw Error('Unsupported operation');
  process.stdout.write(JSON.stringify({ok:true,...out})+'\n');
 }catch(error){process.stdout.write(JSON.stringify({ok:false,error:String(error.message),k:plant?.steps??null,estimate:estimate??null,truth:plant?.s??null})+'\n');}
}
backend.dispose();
