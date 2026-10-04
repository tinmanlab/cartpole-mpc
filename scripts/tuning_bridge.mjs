// Local stdin/stdout adapter to the existing DesignStudy evaluator; no port/service.
// Each request runs the actual pinned MuJoCo WASM for a full task or explicit failure.
import fs from 'node:fs';import crypto from 'node:crypto';import readline from 'node:readline';import{createRequire}from'node:module';
import{createMujocoBackend}from'../src/mujoco_backend.mjs';
const require=createRequire(import.meta.url),L=require('../src/engine'),D=require('../src/design_study');
const m=JSON.parse(fs.readFileSync('tests/fixtures/design_study.json'));const p=JSON.parse(fs.readFileSync('tests/fixtures/sequential_tuning.json'));
const hash=s=>crypto.createHash('sha256').update(s).digest('hex');
D.validateManifest(m);const backend=await createMujocoBackend();L.setPhysicsBackend(backend);
L.LabPlant.prototype.measurementVariance=()=>{throw Error('Noise oracle forbidden');};
const fit=D.calibrate(m).fit;
const tests=m.test.flatMap((t,i)=>p.freshTestSeedRows[i].map(seed=>({...t,id:'fresh-'+i+'-'+seed,seed})));
let locked=null;let actualCalls=0;
const lines=readline.createInterface({input:process.stdin,crlfDelay:Infinity});
try{for await(const line of lines){
 try{
  const q=JSON.parse(line);let out;
  if(q.op==='info')out={physics:backend.diagnostics(),fit,taskManifestSha256:hash(fs.readFileSync(p.taskManifest)),steps:m.steps,actualCalls};
  else if(q.op==='lock'){
   if(locked)throw Error('Already locked');if(!Array.isArray(q.configurations)||!q.configurations.length)throw Error('Empty lock');
   locked=new Set(q.configurations.map(c=>JSON.stringify(Object.entries(c).sort())));out={locked:true,configurations:locked.size};
  }else if(q.op==='evaluate'){
   const c=q.configuration;if(!c||Object.keys(c).some(k=>!['controller','observer','effort','process','horizon'].includes(k)))throw Error('Invalid configuration fields');
   const pair=m.pairs.find(z=>z.controller===c.controller&&z.observer===c.observer);if(!pair)throw Error('Unknown controller/observer');
   const pool=q.phase==='training'?m.training:q.phase==='validation'?m.validation:q.phase==='test'?tests:null;
   if(!pool)throw Error('Unknown evaluation phase');const test=pool.find(t=>t.id===q.caseId);if(!test)throw Error('Unknown phase-specific case');
   if(q.phase==='test'&&(!locked||!locked.has(JSON.stringify(Object.entries(c).sort()))))throw Error('Test before configuration lock');
   if(q.phase!=='test'&&locked)throw Error('Search/validation prohibited after final test lock');
   const candidate={id:JSON.stringify(c),effortMultiplier:c.effort,processMultiplier:c.process,...(c.horizon!==undefined?{horizon:c.horizon}:{})};
   const start=performance.now(),run=D.createRun(m,test,pair,candidate,fit,{searchDomain:p.domain,record:false});
   while(!run.done)run.step();const result=run.result();actualCalls++;
   if(result.outcome==='execution-error')throw Error('Evaluator implementation failure: '+result.reason);
   const {trace,...rest}=result;out={...rest,configuration:c,phase:q.phase,bridgeComputeMs:performance.now()-start,actualCalls};
  }else if(q.op==='close'){process.stdout.write(JSON.stringify({ok:true})+'\n');break;}
  else throw Error('Unsupported bridge operation');
  process.stdout.write(JSON.stringify({ok:true,result:out})+'\n');
 }catch(e){process.stdout.write(JSON.stringify({ok:false,error:String(e.message)})+'\n');}
}}finally{backend.dispose();}
