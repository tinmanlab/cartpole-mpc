'use strict';
const fs=require('fs'),path=require('path');
const C=require('../src/commissioning');

function main(){
  const out={
    schema:'cartpole-commissioning-evidence/v1',
    generatedAt:new Date().toISOString(),
    notes:[
      'CartPole-only commissioning evidence; do not promote to humanoid/hardware capability.',
      'Controller tuner candidates are accepted only if both held-out validation and held-out test improve.',
      'Estimator calibration objective includes RMSE plus NIS/NEES consistency penalties.',
      'Single-factor ablations precede the combined sim2real stress so failure causes remain attributable.'
    ],
    systemIdentification:C.identifyPlant(),
    actuatorIdentification:C.identifyActuator(),
    estimatorCalibration:C.calibrateEstimator(),
    controllerTuning:C.tuneController(),
    mpcTuning:C.tuneMpcStructure(),
    robustMpcProbe:C.robustMpcProbe(),
    modelHierarchy:C.modelHierarchySweep(),
    singleFactorAblation:C.ablationReport(),
    stress:C.stressReport()
  };
  const dest=path.join(__dirname,'..','evidence','commissioning.json');
  fs.writeFileSync(dest,JSON.stringify(out,null,2));
  console.log(dest);
  console.log(JSON.stringify({
    systemIdentification:out.systemIdentification,
    actuatorIdentification:out.actuatorIdentification,
    estimatorAccepted:out.estimatorCalibration.accepted,
    estimatorScores:out.estimatorCalibration.scores,
    controllerAccepted:out.controllerTuning.accepted,
    controllerScores:out.controllerTuning.scores,
    mpcAccepted:out.mpcTuning.accepted,
    mpcScores:out.mpcTuning.scores,
    robustMpcAccepted:out.robustMpcProbe.accepted,
    ablationPipelines:out.singleFactorAblation.length,
    stressRows:out.stress.length
  },null,2));
}
main();
