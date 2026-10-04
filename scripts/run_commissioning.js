'use strict';
const fs=require('fs'),path=require('path'),crypto=require('crypto');
const Lab=require('../src/engine'),C=require('../src/commissioning');
const root=path.join(__dirname,'..');
const sources=['src/engine.js','src/plant.js','src/qp.js','src/commissioning.js','src/mujoco_backend.mjs','assets/cartpole.xml','vendor/manifest.json','package-lock.json','scripts/run_commissioning.js'];
function hashes(){return Object.fromEntries(sources.map(p=>[p,crypto.createHash('sha256').update(fs.readFileSync(path.join(root,p))).digest('hex')]));}
async function main(){
  const {createMujocoBackend}=await import('../src/mujoco_backend.mjs');
  const backend=await createMujocoBackend();Lab.setPhysicsBackend(backend);
  const sourceSha256=hashes();
  // Reuse completed sections only under exactly the same numerical source identity.
  // This small runner checkpoint is transient test output, never a second SSOT.
  const transient=path.join(root,'test-results');fs.mkdirSync(transient,{recursive:true});
  const checkpointPath=path.join(transient,'commissioning-checkpoint.json'),progressPath=path.join(transient,'commissioning-progress.json');
  let checkpoint={sourceSha256,completed:{}};
  if(fs.existsSync(checkpointPath)){
    const saved=JSON.parse(fs.readFileSync(checkpointPath,'utf8'));
    if(JSON.stringify(saved.sourceSha256)===JSON.stringify(sourceSha256))checkpoint=saved;
  }
  const atomic=(dest,data)=>{fs.writeFileSync(dest+'.tmp',JSON.stringify(data,null,2));fs.renameSync(dest+'.tmp',dest);};
  function run(name,fn){
    if(Object.hasOwn(checkpoint.completed,name))return checkpoint.completed[name].result;
    atomic(progressPath,{section:name,state:'running',startedAt:new Date().toISOString(),completed:Object.keys(checkpoint.completed)});
    const begin=performance.now(),result=fn();
    checkpoint.completed[name]={result,computeMs:performance.now()-begin,completedAt:new Date().toISOString()};
    atomic(checkpointPath,checkpoint);atomic(progressPath,{section:name,state:'completed',computeMs:checkpoint.completed[name].computeMs,completed:Object.keys(checkpoint.completed)});
    return result;
  }
  const out={
    schema:'cartpole-commissioning-evidence/v1',
    generatedAt:new Date().toISOString(),
    sourceSha256,
    physics:backend.diagnostics(),
    informationBoundary:{systemIdentification:'simulation-truth-state',actuatorIdentification:'simulation-realized-force',measurementCovariance:'injected-noise-oracle',hardwareVerified:false},
    notes:[
      'Actual official MuJoCo WASM engine and canonical MJCF; no silent JS physics fallback.',
      'CartPole-only commissioning evidence; do not promote to humanoid/hardware capability.',
      'Controller tuner candidates are accepted only if both held-out validation and held-out test improve; do not repeatedly optimize against this fixed test set.',
      'Estimator calibration objective includes RMSE plus NIS/NEES consistency penalties.',
      'Single-factor ablations precede combined stress. This is simulated commissioning, not measured-data hardware calibration.',
      'Reported solver-call times are host measurements, not hard-real-time or sensor-to-hardware-actuator guarantees.'
    ],
    systemIdentification:run('systemIdentification',()=>C.identifyPlant()),
    actuatorIdentification:run('actuatorIdentification',()=>C.identifyActuator()),
    estimatorCalibration:run('estimatorCalibration',()=>C.calibrateEstimator()),
    controllerTuning:run('controllerTuning',()=>C.tuneController()),
    controlEstimationCoTuning:run('controlEstimationCoTuning',()=>C.coTuneControlEstimation()),
    mpcTuning:run('mpcTuning',()=>C.tuneMpcStructure()),
    safetySupervisor:run('safetySupervisor',()=>C.safetySupervisorProbe()),
    robustMpcProbe:run('robustMpcProbe',()=>C.robustMpcProbe()),
    modelHierarchy:run('modelHierarchy',()=>C.modelHierarchySweep()),
    singleFactorAblation:run('singleFactorAblation',()=>C.ablationReport()),
    advancedFailures:run('advancedFailures',()=>C.advancedFailureReport()),
    discretization:run('discretization',()=>C.discretizationProbe()),
    stress:run('stress',()=>C.stressReport())
  };
  if(JSON.stringify(sourceSha256)!==JSON.stringify(hashes()))throw Error('Numerical sources changed during commissioning; refusing mixed-source receipt');
  out.completedAt=new Date().toISOString();out.physics=backend.diagnostics();out.sourcesStable=true;
  out.sectionComputeMs=Object.fromEntries(Object.entries(checkpoint.completed).map(([k,v])=>[k,v.computeMs]));
  out.checkpointBoundary='Sections may resume only at identical source hashes; timing remains host-specific, not exclusive WCET.';
  const dest=path.join(root,'evidence','commissioning.json');
  fs.writeFileSync(dest,JSON.stringify(out,null,2)+'\n');
  console.log(dest);
  console.log(JSON.stringify({physics:out.physics,sourcesStable:true,
    estimatorAccepted:out.estimatorCalibration.accepted,estimatorScores:out.estimatorCalibration.scores,
    controllerAccepted:out.controllerTuning.accepted,controllerScores:out.controllerTuning.scores,
    coTuningAccepted:out.controlEstimationCoTuning.accepted,coTuningScores:out.controlEstimationCoTuning.scores,
    mpcAccepted:out.mpcTuning.accepted,mpcScores:out.mpcTuning.scores,
    safetySupervisorAccepted:out.safetySupervisor.accepted,robustMpcAccepted:out.robustMpcProbe.accepted,
    stressRows:out.stress.length},null,2));
  backend.dispose();
}
main().catch(error=>{console.error(error);process.exitCode=1;});
