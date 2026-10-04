// Task-scoped experiment adapter: existing MuJoCo, LQR/quadprog MPC, KF/EKF and R fitting.
// No new optimizer/controller/filter. Training objective and test admission are separate.
(function(root,factory){const node=typeof module==='object'&&module.exports;const api=factory(node?require('./engine'):ControlLab,node?require('./calibration_lab'):CalibrationLab);if(node)module.exports=api;else root.DesignStudy=api;})(globalThis,function(L,C){
 'use strict';
 const copy=x=>JSON.parse(JSON.stringify(x)),mean=a=>a.length?a.reduce((s,v)=>s+v,0)/a.length:null;
 const finite=a=>Array.isArray(a)&&a.every(Number.isFinite),freeze=x=>{if(x&&typeof x==='object'){Object.values(x).forEach(freeze);Object.freeze(x);}return x;};
 const clock=()=>typeof performance!=='undefined'?performance.now():Date.now();
 const scalar=(v,name)=>{if(!Number.isFinite(v)||v<=0)throw Error('Positive finite '+name+' required');};
 function candidates(m){
  return m.candidateGrid.effortMultipliers.flatMap(e=>m.candidateGrid.processMultipliers.map(q=>({id:'r'+e+'-q'+q,effortMultiplier:e,processMultiplier:q,baseline:e===m.candidateGrid.baseline.effortMultiplier&&q===m.candidateGrid.baseline.processMultiplier})));
 }
 function validateManifest(m){
  if(m.controlDt!==L.DT||!Number.isInteger(m.steps)||m.steps<1||!Number.isInteger(m.theoryDesign.horizon)||m.theoryDesign.horizon<1)throw Error('Invalid sample/horizon contract');
  for(const [name,a,n] of [['state scales',m.theoryDesign.stateScales,4],['Qe shape',m.theoryDesign.QeBase,4],['P0',m.theoryDesign.P0diag,4]]){if(!finite(a)||a.length!==n)throw Error('Invalid '+name);a.forEach(v=>scalar(v,name));}
  scalar(m.theoryDesign.forceScale_N,'force scale');scalar(m.task.forceLimit_N,'force limit');
  if(m.task.forceLimit_N!==10||m.task.railLimit_m!==2.4||Math.abs(m.task.angleEnvelope_rad-35*Math.PI/180)>1e-12)throw Error('Task physical bounds must match the verified plant');
  for(const q of Object.values(m.task.objective)){scalar(q.scale,'objective scale');scalar(q.weight,'objective weight');}
  if(m.pairs.length!==4||m.pairs.some(p=>!['lqr','hard_mpc'].includes(p.controller)||!['kf','ekf'].includes(p.observer))||new Set(m.pairs.map(p=>p.controller+'/'+p.observer)).size!==4)throw Error('Expected four admitted controller/observer pairs');
  const grid=candidates(m);if(grid.length!==9||grid.filter(c=>c.baseline).length!==1||new Set(grid.map(c=>c.id)).size!==grid.length)throw Error('Invalid equal candidate budget');
  grid.forEach(c=>{scalar(c.effortMultiplier,'effort factor');scalar(c.processMultiplier,'process factor');});
  const all=[...m.training,...m.validation,...m.test];
  if(new Set(all.map(c=>c.id)).size!==all.length||new Set([...all.map(c=>c.seed),m.calibration.seed]).size!==all.length+1)throw Error('Calibration/train/validation/test must be disjoint');
  if(m.training.length!==3||m.validation.length!==3||m.test.length!==6||m.test.filter(c=>c.group==='primary').length!==4)throw Error('Invalid frozen study split');
  for(const c of all){if(!finite(c.initialState)||c.initialState.length!==4||!Number.isFinite(c.goal)||Math.abs(c.goal)>m.task.railLimit_m)throw Error('Invalid task initial state/goal');}
  return true;
 }
 function checkPair(m,pair){const p=m.pairs.find(p=>p.id===pair.id);if(!p||p.controller!==pair.controller||p.observer!==pair.observer)throw Error('Unsupported study pair');return p;}
 function checkCandidate(m,c){const candidate=candidates(m).find(q=>q.id===c.id);if(!candidate||candidate.effortMultiplier!==c.effortMultiplier||candidate.processMultiplier!==c.processMultiplier)throw Error('Invalid candidate outside fixed positive grid');return candidate;}
 function calibrate(m){
  const p=new L.LabPlant({seed:m.calibration.seed,scenario:m.calibration.scenario});p.reset(m.calibration.fixtureState);
  p.sensorStd=[m.sensorFixture.sigmaPosition_m,m.sensorFixture.sigmaAngle_rad];
  const measurements=Array.from({length:m.calibration.samples},()=>p.sensor().slice());
  const fit=C.estimateMeasurementNoise(measurements,m.calibrationScreen);if(!fit.usable)throw Error('Stationary R measurement screen rejected');
  return {measurements,fit,scope:'simulated clamped repeatability; fitter receives only readings, not true variance or bias'};
 }
 function searchCandidate(pair,c,domain){
  if(!domain.controllers?.includes(pair.controller)||!domain.observers?.includes(pair.observer))throw Error('Unsupported search pair');
  for(const [name,bounds] of [['effortMultiplier',domain.effortBounds],['processMultiplier',domain.processBounds]]){
   if(!Array.isArray(bounds)||bounds.length!==2||!bounds.every(Number.isFinite)||bounds[0]<=0||bounds[1]<bounds[0]||!Number.isFinite(c[name])||c[name]<bounds[0]||c[name]>bounds[1])throw Error('Candidate outside declared search domain');
  }
  if(pair.controller==='hard_mpc'){if(!Number.isInteger(c.horizon)||!domain.mpcHorizons.includes(c.horizon))throw Error('Unsupported search horizon');}
  else if(c.horizon!==undefined)throw Error('MPC horizon is inactive for LQR');
  return {...c,baseline:c.effortMultiplier===domain.baselineEffort&&c.processMultiplier===domain.baselineProcess&&(pair.controller==='lqr'||c.horizon===domain.baselineHorizon)};
 }
 function design(m,pair,candidate,fit,searchDomain=null){
  checkPair(m,pair);const c=searchDomain?searchCandidate(pair,candidate,searchDomain):checkCandidate(m,candidate);if(!fit?.usable)throw Error('R calibration rejected');
  const spec=new L.LabPlant().spec,Qc=m.theoryDesign.stateScales.map(v=>1/v**2),Rc=c.effortMultiplier/m.theoryDesign.forceScale_N**2,Qe=m.theoryDesign.QeBase.map(v=>c.processMultiplier*v),Re=fit.Rdiag.slice();
  const {A,B}=L.linearModel(spec),terminal=L.riccatiTerminal(A,B,L.diag(Qc),Rc);
  const BtP=L.mul(L.T(B),terminal.P),den=Rc+L.mul(BtP,B)[0][0],K=L.mul(BtP,A)[0].map(v=>v/den);
  return {pairId:pair.id,controller:pair.controller,observer:pair.observer,candidate:c,Qc,Rc,Qe,Re,P0:m.theoryDesign.P0diag.slice(),A,B,Pf:terminal.P,K,terminalInfo:terminal.info,horizon:searchDomain&&pair.controller==='hard_mpc'?c.horizon:m.theoryDesign.horizon,
   information:{Qc:'declared inverse-squared state scales',Rc:'declared effort multiplier / forceScale^2',Qe:'effective task-tuned scale, not calibrated physical process noise',Re:'stationary measured repeatability',Pf:'nominal full discrete Riccati tail cost; no terminal-set guarantee'}};
 }
 function buildControllers(m,pair,d,spec){
  const opts={Qdiag:d.Qc,R:d.Rc};
  const controller=pair.controller==='lqr'?new L.LQRController(spec,opts):new L.LinearMPCController(spec,{...opts,N:d.horizon,positionLimit:m.task.railLimit_m,terminalCost:'dare'});
  const observer=L.makeObserver(pair.observer,spec,{Q:d.Qe,R:d.Re});
  if(controller.limit!==m.task.forceLimit_N)throw Error('Controller force authority differs from task');
  if(pair.controller==='lqr'&&Math.max(...controller.K.map((v,i)=>Math.abs(v-d.K[i])))>1e-8)throw Error('LQR gain differs from verified Riccati design');
  return {controller,observer};
 }
 function stageCost(m,{truth,goal,u,previousU}){
  if(!finite(truth)||truth.length!==4||![goal,u,previousU].every(Number.isFinite))throw Error('Invalid finite task state/input');
  const o=m.task.objective,term=(v,q)=>q.weight*(v/q.scale)**2;
  const parts={position:term(truth[0]-goal,o.position),angle:term(L.wrap(truth[2]),o.angle),force:term(u,o.force),deltaForce:term(u-previousU,o.deltaForce)};
  return {...parts,total:Object.values(parts).reduce((a,b)=>a+b,0)};
 }
 function distribution(a){if(!a.length)return null;const s=a.slice().sort((a,b)=>a-b),at=q=>s[Math.max(0,Math.ceil(s.length*q)-1)];return {samples:s.length,p50:at(.5),p95:at(.95),max:s.at(-1)};}
 function createRun(m,test,pair,candidate,fit,{record=false,searchDomain=null}={}){
  if(L.physicsInfo().backend!=='mujoco-wasm')throw Error('Actual MuJoCo WASM required');
  const started=clock(),d=design(m,pair,candidate,fit,searchDomain),plant=new L.LabPlant({seed:test.seed,scenario:test.scenario});plant.reset(test.initialState);plant.goal=test.goal;
  plant.sensorStd=[m.sensorFixture.sigmaPosition_m,m.sensorFixture.sigmaAngle_rad];
  const {controller,observer}=buildControllers(m,pair,d,plant.spec);let y=plant.sensor();observer.reset(y);observer.P=L.diag(d.P0);controller.reset();let xhat=observer.x.slice(),previousU=0,outcome='running',reason=null;
  const trace=[],series=[],times={solve:[],observer:[],step:[]},termSum={position:0,angle:0,force:0,deltaForce:0},constructionMs=clock()-started;
  let saturation=0,activeRail=0,maxKkt=0,maxPrimal=0;
  function step(){
   if(outcome!=='running')return false;const k=series.length,begin=clock(),truth=plant.s.slice(),estimate=xhat.slice();
   try{
    const row={k,measurementTime:k*L.DT,nextTime:(k+1)*L.DT,truth,estimate,measurement:y.slice(),P:copy(observer.P),goal:test.goal,previousU};
    const t=clock(),u=controller.act(estimate,test.goal);times.solve.push(clock()-t);
    if(!Number.isFinite(u)||Math.abs(u)>m.task.forceLimit_N+1e-8)throw Error('Command rejected by physical force gate');
    if(pair.controller==='hard_mpc'){
     if(!controller.lastConverged||controller.lastKktResidual>1e-7||controller.lastPrimalResidual>1e-8)throw Error('QP numerical admission failed');
     maxKkt=Math.max(maxKkt,controller.lastKktResidual);maxPrimal=Math.max(maxPrimal,controller.lastPrimalResidual);
     if(controller.lastPrediction.some(x=>Math.abs(x[0])>=m.task.railLimit_m-1e-6))activeRail++;
    }
    if(Math.abs(u)>=m.task.forceLimit_N-1e-8)saturation++;
    const costs=stageCost(m,{truth,goal:test.goal,u,previousU});for(const key in termSum)termSum[key]+=costs[key];
    if(test.push&&k===test.push.step)plant.applyPush(test.push.force,test.push.duration);
    const out=plant.step(u);y=plant.sensor();const ob=clock();xhat=observer.step(plant.sensorMeta.fresh?y:null,u);times.observer.push(clock()-ob);
    if(!xhat.every(Number.isFinite)||!observer.P.flat().every(Number.isFinite))throw Error('Nonfinite observer output');
    const err=estimate.map((v,i)=>i===2?L.wrap(v-truth[i]):v-truth[i]);
    // compact independent-audit series: all task terms can be recomputed without trusting costs.
    series.push([truth[0]-test.goal,L.wrap(truth[2]),u,u-previousU,...err,out.state[0],out.state[2],out.appliedCommand]);
    if(record)trace.push({...row,command:u,appliedForce:out.appliedCommand,external:out.external,nextTruth:out.state.slice(),nextMeasurement:y.slice(),nextEstimate:xhat.slice(),nextP:copy(observer.P),nextInnovation:observer.last?.innovation?.slice()??null,nextS:observer.last?.S?copy(observer.last.S):null,costs,solverKkt:controller.lastKktResidual??null,solverPrimal:controller.lastPrimalResidual??null});
    previousU=u;times.step.push(clock()-begin);
    if(Math.abs(out.state[0])>m.task.railLimit_m||Math.abs(L.wrap(out.state[2]))>m.task.angleEnvelope_rad){outcome='envelope-failure';reason='physical rail or angle envelope';}
    else if(series.length>=m.steps)outcome='completed';return outcome==='running';
   }catch(e){reason=String(e.message);outcome=reason.startsWith('QP ')?'solver-rejected':'execution-error';times.step.push(clock()-begin);return false;}
  }
  function result(){
   const n=series.length,window=series.slice(-m.task.finalWindowSteps),rms=(a,index)=>a.length?Math.sqrt(mean(a.map(r=>r[index]**2))):null;
   const tailPosition=window.length?Math.sqrt(mean(window.map(r=>(r[8]-test.goal)**2))):null,tailAngle=window.length?Math.sqrt(mean(window.map(r=>L.wrap(r[9])**2))):null;
   const passed=outcome==='completed'&&window.length===m.task.finalWindowSteps&&tailPosition<=m.task.finalPositionRms_m&&tailAngle<=m.task.finalAngleRms_rad;
   const components=Object.fromEntries(Object.entries(termSum).map(([k,v])=>[k,n?v/n:null]));
   return {caseId:test.id,seed:test.seed,scenario:test.scenario,pairId:pair.id,candidateId:candidate.id,design:d,outcome,reason,taskPassed:passed,appliedSteps:n,requestedSteps:m.steps,
    fullScore:outcome==='completed'?Object.values(components).reduce((a,b)=>a+b,0):null,scoreComponents:components,partialScore:n?Object.values(components).reduce((a,b)=>a+b,0):null,
    positionTrackingRmse:rms(series,0),angleRms:rms(series,1),forceRms:rms(series,2),deltaForceRms:rms(series,3),estimationRmseByState:[4,5,6,7].map(i=>rms(series,i)),tailPositionRms:tailPosition,tailAngleRms:tailAngle,
    saturatedSamples:saturation,predictedRailActiveSamples:activeRail,maxKktResidual:pair.controller==='hard_mpc'?maxKkt:null,maxPrimalResidual:pair.controller==='hard_mpc'?maxPrimal:null,
    finalTruth:plant.s.slice(),finalEstimate:xhat.slice(),timing:{constructionMs,solve:distribution(times.solve),observer:distribution(times.observer),synchronousStep:distribution(times.step),overBudgetSamples:times.step.filter(v=>v>m.task.executionBudget_ms).length,scope:m.task.timingScope},
    information:{controller:'observer-output only',observer:'current/past measurements and requested commands only',Re:'separate stationary readings',truth:'simulator task evaluator only',stateUnits:['m','m/s','rad','rad/s']},auditSeries:series,trace};
  }
  return {step,result,components:()=>({plant,controller,observer}),get done(){return outcome!=='running';},snapshot:()=>({steps:series.length,outcome,truth:plant.s.slice(),estimate:xhat.slice(),goal:test.goal,design:d,last:trace.at(-1)??null})};
 }
 function withoutSeries(r){const {trace,auditSeries,...summary}=r;return summary;}
 function aggregate(candidate,runs){const completed=runs.filter(r=>r.outcome==='completed');return {candidate,hardFailures:runs.length-completed.length,taskFailures:runs.filter(r=>!r.taskPassed).length,fullScore:completed.length===runs.length?mean(completed.map(r=>r.fullScore)):null,eligible:completed.length===runs.length&&runs.every(r=>r.taskPassed),runs:runs.map(withoutSeries)};}
 function rankCandidates(a,b){return a.hardFailures-b.hardFailures||a.taskFailures-b.taskFailures||((a.fullScore??Infinity)-(b.fullScore??Infinity))||Number(b.candidate.baseline)-Number(a.candidate.baseline)||a.candidate.id.localeCompare(b.candidate.id);}
 function settingsSignature(m){return JSON.stringify({controlDt:m.controlDt,steps:m.steps,theoryDesign:m.theoryDesign,sensorFixture:m.sensorFixture,calibration:m.calibration,task:m.task,candidateGrid:m.candidateGrid,pairs:m.pairs,selection:m.selection});}
 async function select(m,fit,{evaluate,onProgress=()=>{}}={}){
  if(typeof evaluate!=='function')throw Error('An explicit training/validation evaluator is required');
  const available=[...m.training,...m.validation];if(new Set(available.map(r=>r.id)).size!==available.length||new Set(available.map(r=>r.seed)).size!==available.length||available.some(r=>r.seed===m.calibration.seed))throw Error('Training/validation records must be disjoint');
  // This function intentionally never references m.test; test getter poisoning is a contract test.
  const candidatesAll=candidates(m),results=[];let trainingEvaluations=0,validationEvaluations=0;
  for(const pair of m.pairs){
   const training=[];for(const c of candidatesAll){const runs=await evaluate(pair,c,m.training,'training');training.push(aggregate(c,runs));trainingEvaluations+=runs.length;onProgress({stage:'training',pairId:pair.id,candidateId:c.id,done:trainingEvaluations,total:4*9*3});}
   const short=training.slice().sort(rankCandidates).slice(0,m.candidateGrid.shortlist).map(x=>x.candidate);
   const base=candidatesAll.find(c=>c.baseline);if(m.candidateGrid.includeBaseline&&!short.some(c=>c.baseline))short.push(base);
   const validation=[];for(const c of short){const runs=await evaluate(pair,c,m.validation,'validation');validation.push(aggregate(c,runs));validationEvaluations+=runs.length;onProgress({stage:'validation',pairId:pair.id,candidateId:c.id});}
   const winner=validation.slice().sort(rankCandidates)[0];results.push({pairId:pair.id,candidate:winner.candidate,eligible:winner.eligible,validationScore:winner.fullScore,training,validation});
  }
  const eligible=results.filter(r=>r.eligible),best=eligible.length?Math.min(...eligible.map(r=>r.validationScore)):null;
  const near=eligible.filter(r=>r.validationScore<=best*(1+m.selection.crossPairTieRelative));near.sort((a,b)=>m.pairs.find(p=>p.id===a.pairId).simplicityOrder-m.pairs.find(p=>p.id===b.pairId).simplicityOrder);
  const recommendedPairId=near[0]?.pairId??null,id=JSON.stringify({pairs:results.map(r=>[r.pairId,r.candidate.id]),recommendedPairId});
  return freeze({id,settingsSignature:settingsSignature(m),Re:fit.Rdiag.slice(),pairs:results,recommendedPairId,trainingEvaluations,validationEvaluations,testUsedForSelection:false,locked:true,tiePolicy:m.selection.crossPairRule,scope:'best within the declared candidates and validation task; not global optimum or hardware admission'});
 }
 function pairedMetrics(a,b,m){const n=Math.min(a.auditSeries.length,b.auditSeries.length),score=r=>n?mean(r.auditSeries.slice(0,n).map(v=>m.task.objective.position.weight*(v[0]/m.task.objective.position.scale)**2+m.task.objective.angle.weight*(v[1]/m.task.objective.angle.scale)**2+m.task.objective.force.weight*(v[2]/m.task.objective.force.scale)**2+m.task.objective.deltaForce.weight*(v[3]/m.task.objective.deltaForce.scale)**2)):null;
  return {commonPrefixSteps:n,baselineScore:score(a),candidateScore:score(b),bothCompleted:a.outcome==='completed'&&b.outcome==='completed',scope:'shared-prefix diagnostic only; failed runs never receive a full-duration score'};}
 function adjudicate(m,selection,rows){
  const id=selection.recommendedPairId,p=rows.filter(r=>r.pairId===id&&r.group==='primary'),expected=m.test.filter(t=>t.group==='primary'),reasons=[];
  if(selection.settingsSignature!==settingsSignature(m))reasons.push('Design settings changed after selection');
  if(!id)reasons.push('No validation-eligible pair; no recommendation to promote');
  if(p.length!==expected.length||new Set(p.map(r=>r.caseId)).size!==expected.length||expected.some(t=>!p.some(r=>r.caseId===t.id)))reasons.push('Incomplete or mismatched primary test coverage');
  if(p.some(r=>r.candidate.outcome!=='completed'||!r.candidate.taskPassed))reasons.push('Locked candidate fails at least one primary task');
  if(p.some(r=>r.baseline.taskPassed&&!r.candidate.taskPassed))reasons.push('Candidate-only primary task failure');
  const both=p.filter(r=>r.baseline.outcome==='completed'&&r.candidate.outcome==='completed'),bs=both.length===p.length&&p.length?mean(both.map(r=>r.baseline.fullScore)):null,cs=both.length===p.length&&p.length?mean(both.map(r=>r.candidate.fullScore)):null;
  if(cs===null||bs===null||!Number.isFinite(cs)||!Number.isFinite(bs)||cs>(1+m.selection.primaryNonRegressionTolerance)*bs)reasons.push('Primary full-duration non-regression gate not established');
  return freeze({selectionId:selection.id,settingsSignature:settingsSignature(m),testCaseSignature:JSON.stringify(m.test),recommendedPairId:id,accepted:reasons.length===0,reasons,primaryCases:p.length,baselineScore:bs,candidateScore:cs,relativeScore:bs>0&&cs!==null?cs/bs:null,testCanReselect:false,hardwareAdmission:false,scope:m.selection.adoptionScope});
 }
 function application(m,selection,admission,test,fit){
  if(!Object.isFrozen(selection)||!selection.locked||!admission.accepted)throw Error('Design not admitted; selection/application rejected');
  if(selection.settingsSignature!==settingsSignature(m)||admission.settingsSignature!==selection.settingsSignature||admission.testCaseSignature!==JSON.stringify(m.test))throw Error('Study settings/signature changed since evaluation');
  if(JSON.stringify(fit.Rdiag)!==JSON.stringify(selection.Re))throw Error('Measurement calibration covariance changed');
  if(admission.selectionId!==selection.id||admission.recommendedPairId!==selection.recommendedPairId)throw Error('Selection/admission identity mismatch');
  const actual=m.test.find(t=>t.id===test.id&&t.group==='primary');if(!actual||JSON.stringify(actual)!==JSON.stringify(test))throw Error('Application case outside admitted explicit task');
  const picked=selection.pairs.find(p=>p.pairId===selection.recommendedPairId),pair=m.pairs.find(p=>p.id===picked?.pairId);if(!pair)throw Error('Invalid locked selection');
  return freeze({study:'task-aware-design',case:copy(actual),pair:copy(pair),candidate:copy(picked.candidate),design:design(m,pair,picked.candidate,fit),sensorStd:[m.sensorFixture.sigmaPosition_m,m.sensorFixture.sigmaAngle_rad],startPaused:true,defaultChanged:false,scope:'Explicit replay of one admitted simulated task; not permission for arbitrary conditions or hardware'});
 }
 async function runStudy(m,{onProgress=()=>{},yieldControl=()=>Promise.resolve(),retainAudit=true}={}){
  validateManifest(m);const capture=calibrate(m),fit=capture.fit,allRuns=[];
  async function execute(pair,candidate,test,phase,record=false){const run=createRun(m,test,pair,candidate,fit,{record});let n=0;while(!run.done){run.step();if(++n%100===0)await yieldControl();}const r=run.result();if(r.outcome==='execution-error')throw Error('Study implementation error at '+test.id+': '+r.reason);if(retainAudit)allRuns.push({phase,...r});return r;}
  const selection=await select(m,fit,{onProgress,evaluate:async(pair,candidate,tests,phase)=>{const rows=[];for(const t of tests)rows.push(await execute(pair,candidate,t,phase));return rows;}});
  onProgress({stage:'selection-locked',recommendedPairId:selection.recommendedPairId,testEvaluations:0});
  const tests=[],base=candidates(m).find(c=>c.baseline);let evaluated=0;
  for(const pair of m.pairs){const selected=selection.pairs.find(p=>p.pairId===pair.id).candidate;
   for(const test of m.test){const baseline=await execute(pair,base,test,'test-baseline',true),candidate=await execute(pair,selected,test,'test-selected',true);tests.push({caseId:test.id,pairId:pair.id,group:test.group,baseline,candidate,paired:pairedMetrics(baseline,candidate,m)});onProgress({stage:'locked-test',pairId:pair.id,caseId:test.id,done:++evaluated,total:m.pairs.length*m.test.length});await yieldControl();}
  }
  const admission=adjudicate(m,selection,tests);
  const summary=m.pairs.map(pair=>({pairId:pair.id,selectedCandidateId:selection.pairs.find(p=>p.pairId===pair.id).candidate.id,groups:['primary','boundary'].map(group=>{
   const rr=tests.filter(t=>t.pairId===pair.id&&t.group===group),both=rr.filter(t=>t.paired.bothCompleted);
   return {group,cases:rr.length,bothCompleted:both.length,excludedFromFullDuration:rr.length-both.length,arms:['baseline','candidate'].map(arm=>({arm,completed:rr.filter(t=>t[arm].outcome==='completed').length,taskPassed:rr.filter(t=>t[arm].taskPassed).length,score:both.length?mean(both.map(t=>t[arm].fullScore)):null,trackingRmse:both.length?Math.sqrt(mean(both.map(t=>t[arm].positionTrackingRmse**2))):null,forceRms:both.length?Math.sqrt(mean(both.map(t=>t[arm].forceRms**2))):null,deltaForceRms:both.length?Math.sqrt(mean(both.map(t=>t[arm].deltaForceRms**2))):null,saturatedSamples:rr.reduce((s,t)=>s+t[arm].saturatedSamples,0)}))};
  })}));
  return {capture,selection,tests,summary,admission,auditRuns:allRuns,defaultsChanged:false,method:'Measured R + inverse-squared-scale Riccati baseline + identical two-factor candidate budgets + task-first rank + locked independent test'};
 }
 return {validateManifest,candidates,calibrate,design,buildControllers,stageCost,createRun,withoutSeries,aggregate,rankCandidates,select,adjudicate,application,runStudy};
});
