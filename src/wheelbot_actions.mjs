import {validateActionJump,actionJumpCommand,actionJumpObserver,jumpEntry} from './wheelbot_action_jump.mjs';
import {validateWheelbotProfile,observerUpdate} from './wheelbot_control.mjs';
import {CONTROLLED_INDICES as indices} from './wheelbot_backend.mjs';
const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
const keys=['x','z','pitch'];
export function createWheelbotActions(backend,baseProfile,poseBundle,{seed=17,jumpProfile=null}={}){
 if(!Number.isInteger(seed)||seed<0||seed>0xffffffff)throw Error('Seed must be an unsigned 32-bit integer');
 validateWheelbotProfile(backend,baseProfile);
 const bundle=structuredClone(poseBundle),base=structuredClone(baseProfile);
 const jumpData=jumpProfile?structuredClone(jumpProfile):null;if(jumpData)validateActionJump(backend,base,jumpData);
 if(bundle?.schema!=='wheelbot-pose-atlas/v1'||bundle.assetSha256!==backend.assetSha256||!Array.isArray(bundle.profiles)||bundle.profiles.length!==25||typeof backend.stepWrench!=='function')throw Error('Invalid posture atlas/backend');
 for(const k of keys)if(!Array.isArray(bundle.ranges?.[k])||bundle.ranges[k].length!==2||!bundle.ranges[k].every(Number.isFinite)||bundle.ranges[k][0]>=bundle.ranges[k][1])throw Error('Invalid atlas ranges');
 for(const name of ['velocity','acceleration'])if(!Array.isArray(bundle.referenceLimits?.[name])||bundle.referenceLimits[name].length!==3||!bundle.referenceLimits[name].every(v=>Number.isFinite(v)&&v>0))throw Error('Invalid reference limits');
 bundle.profiles.forEach(p=>validateWheelbotProfile(backend,p));
 const zs=[...new Set(bundle.profiles.map(p=>p.target.z))].sort((a,b)=>a-b),ps=[...new Set(bundle.profiles.map(p=>p.target.pitch))].sort((a,b)=>a-b);
 if(zs.length!==5||ps.length!==5||!zs.every(Number.isFinite)||!ps.every(Number.isFinite)||new Set(bundle.profiles.map(p=>[p.target.z,p.target.pitch].join(','))).size!==25)throw Error('Invalid atlas grid');
 const bracket=(a,v)=>{let i=0;while(i<a.length-2&&v>a[i+1])i++;return [i,clamp((v-a[i])/(a[i+1]-a[i]),0,1)];};
 function profile(z,pitch,x){
  const [zi,zt]=bracket(zs,z),[pi,pt]=bracket(ps,pitch);
  const corners=[[zi,pi,(1-zt)*(1-pt)],[zi+1,pi,zt*(1-pt)],[zi,pi+1,(1-zt)*pt],[zi+1,pi+1,zt*pt]].map(([i,j,w])=>[bundle.profiles.find(p=>p.target.z===zs[i]&&p.target.pitch===ps[j]),w]);
  const blend=(name)=>{const go=(v,path)=>Array.isArray(v)?v.map((a,i)=>go(a,[...path,i])):corners.reduce((s,[p,w])=>s+w*path.reduce((a,i)=>a[i],p[name]),0);return go(corners[0][0][name],[]);};
  const p={...base};for(const name of ['qref','uref','A','B','K','L'])p[name]=blend(name);p.qref[0]=x;return p;
 }
 let rng=seed>>>0;
 const uniform=()=>{rng=(Math.imul(1664525,rng)+1013904223)>>>0;return (rng+.5)/4294967296;};
 const noise=()=>Math.sqrt(-2*Math.log(uniform()))*Math.cos(2*Math.PI*uniform());
 const measure=x=>base.measurementIndices.map((j,i)=>x[j]+base.measurementSigma[i]*noise());
 let truth=[...base.qref,0,0,0,0,0,0],estimate=[...measure(truth).slice(0,5),0,0,0,0,0,0];
 let target={x:0,z:base.qref[1],pitch:base.qref[2]},appliedTarget={...target},velocity=[0,0,0],steps=0,last=null,phase='standing',failed=false,pulse=null,status='Ready',settled=0;
 let jumpState=null,lastJumpResult=null;
 const snapshot=()=>structuredClone({actions:true,truth,estimate,estimateIndices:indices,observerValid:!failed,steps,goal:target.x,mode:jumpState?.started?'tvlqr_kf':'lqr_kf',failed,done:false,target,appliedTarget,phase,actionStatus:status,capabilities:{jump:!!jumpData,recover:false},lastJumpResult,last,controlDt:.01});
 return {controlDt:.01,snapshot,setTarget(request){
  if(!request||Object.keys(request).some(k=>!keys.includes(k))||!Object.keys(request).length||Object.values(request).some(v=>!Number.isFinite(v)))throw Error('Target requires finite x, z or pitch');
  if(failed)throw Error('Robot is fallen; posture control unavailable. Reset is separate from recovery.');
  if(jumpState)throw Error('점프가 끝난 뒤 목표를 변경할 수 있습니다.');
  const next={...target,...request},reasons=[];
  for(const k of keys){const bounded=clamp(next[k],...bundle.ranges[k]);if(bounded!==next[k])reasons.push(k+' clamped to validated range');next[k]=bounded;}
  target=next;status=reasons.join('; ')||'Tracking target';return snapshot();
 },disturb({kind,direction,strength}={}){
  if(!['push','gust','twist'].includes(kind)||![-1,1].includes(direction)||!['light','medium','strong'].includes(strength))throw Error('Invalid disturbance kind, direction or strength');
  if(pulse)throw Error('A disturbance is already active');
  const level=['light','medium','strong'].indexOf(strength),duration=kind==='gust'?[.2,.25,.2][level]:[.1,.2,.2][level];
  pulse={kind,direction,strength,total:Math.round(duration/.01),remaining:Math.round(duration/.01),amplitude:(kind==='twist'?[.02,.05,.1]:[.5,1.5,4])[level],startTimeS:steps*.01};
  status=kind+' applied';return snapshot();
 },jump(){
  if(!jumpData)throw Error('Jump is not validated for this robot; state preserved');
  if(failed||jumpState||pulse)throw Error('균형 회복 후 외란이 끝났을 때 점프할 수 있습니다.');
  target={x:clamp(estimate[0],...bundle.ranges.x),z:base.qref[1],pitch:base.qref[2]};
  jumpState={started:false,wait:0,index:0,origin:target.x,flight:0,longestFlight:0,maxClearance:0,maxPenetration:0,maxJointExcursion:0,tail:[],entryConfirm:0};
  lastJumpResult=null;phase='preparing-jump';status='점프 준비: 같은 로봇을 기준 자세로 이동합니다.';return snapshot();
 },recover(){throw Error('Physical get-up is not validated for this robot; state preserved');},step(externalX=0){
  if(!Number.isFinite(externalX)||Math.abs(externalX)>4)throw Error('External force must be finite and within ±4 N');
  const wrench=[externalX,0,0];
  if(pulse){const scale=pulse.kind==='gust'?Math.sin(Math.PI*(pulse.total-pulse.remaining+.5)/pulse.total):1;wrench[pulse.kind==='twist'?2:0]+=pulse.direction*pulse.amplitude*scale;}
  if(Math.abs(wrench[0])>4)throw Error('Combined force exceeds 4 N');
  const controlStart=performance.now();
  for(let i=0;i<3;i++){
   const k=keys[i],error=target[k]-appliedTarget[k],vmax=bundle.referenceLimits.velocity[i],acc=bundle.referenceLimits.acceleration[i];
   const desired=Math.sign(error)*Math.min(vmax,Math.sqrt(2*acc*Math.abs(error)));
   velocity[i]+=clamp(desired-velocity[i],-acc*.01,acc*.01);
   const delta=velocity[i]*.01;if(Math.abs(delta)>=Math.abs(error)){appliedTarget[k]=target[k];velocity[i]=0;}else appliedTarget[k]+=delta;
  }
  if(jumpState&&!jumpState.started){
   jumpState.wait++;const ready=jumpEntry(estimate,base)&&Math.abs(estimate[0]-target.x)<.015&&keys.every(k=>Math.abs(target[k]-appliedTarget[k])<.003);
   jumpState.entryConfirm=ready?jumpState.entryConfirm+1:0;
   if(jumpState.entryConfirm>=15){jumpState.started=true;jumpState.origin=estimate[0];jumpState.index=0;phase='jumping';status='모터로 도약 · 착지 · 제동';}
   else if(jumpState.wait>2000){lastJumpResult={passed:false,reason:'entry-not-settled'};jumpState=null;status='기준 자세 정착 실패: 점프를 시작하지 않았습니다.';}
  }
  const activeJump=!!jumpState?.started;
  const p=profile(appliedTarget.z,appliedTarget.pitch,appliedTarget.x);
  const ref=indices.map(j=>j<6?p.qref[j]:0);ref[5]=velocity[0];ref[10]=velocity[0]/.05;
  const requested=failed?[0,0,0]:activeJump?actionJumpCommand(jumpData,base,jumpState.index,estimate,jumpState.origin).requested:p.uref.map((v,i)=>v-p.K[i].reduce((s,g,j)=>s+g*(estimate[j]-ref[j]),0));
  const u=requested.map((v,i)=>clamp(v,-backend.limits[i],backend.limits[i]));
  const controlMs=performance.now()-controlStart,plantStart=performance.now();
  truth=backend.stepWrench(truth,u,wrench);const plantMs=performance.now()-plantStart;
  const measurement=measure(truth);if(!failed)estimate=activeJump?actionJumpObserver(jumpData,base,jumpState.index,estimate,u,measurement,jumpState.origin):observerUpdate(p,estimate,u,measurement);
  steps++;
  if(!failed&&(activeJump?(Math.abs(truth[2]-base.qref[2])>.6||truth[1]<.15):(Math.abs(truth[2]-appliedTarget.pitch)>.5||truth[1]<.26||Math.abs(truth[3]-p.qref[3])>.6||Math.abs(truth[4]-p.qref[4])>.6))){
   failed=true;phase='falling';status='Outside validated balance domain; motors disengaged';
  }
  if(failed){settled=truth.slice(6).every(v=>Math.abs(v)<.15)?settled+1:0;if(settled>=50)phase='fallen';}
  else phase=activeJump?'jumping':jumpState?'preparing-jump':keys.some(k=>Math.abs(target[k]-appliedTarget[k])>1e-5)?'moving':'standing';
  if(activeJump){
   const g=backend.actionTelemetry(truth,u),interval=backend.lastActionStep?.();
   const air=g.wheelClearanceM>.005&&backend.contact(truth).wheelContacts===0;
   jumpState.flight=air?jumpState.flight+1:0;jumpState.longestFlight=Math.max(jumpState.longestFlight,jumpState.flight);jumpState.maxClearance=Math.max(jumpState.maxClearance,g.wheelClearanceM);
   jumpState.maxPenetration=Math.max(jumpState.maxPenetration,g.maximumPenetrationM,interval?.maximumPenetrationM??0);jumpState.maxJointExcursion=Math.max(jumpState.maxJointExcursion,g.jointLimitExcursionRad,interval?.jointLimitExcursionRad??0);
   if(jumpState.index>=jumpData.steps-50)jumpState.tail.push(truth.slice());jumpState.index++;
   if(jumpState.maxPenetration>.01||jumpState.maxJointExcursion>.02){failed=true;phase='falling';status='점프가 물리 허용 범위를 벗어났습니다.';}
   if(failed||jumpState.index>=jumpData.steps){
    const tail=jumpState.tail,errors=tail.length?Array.from({length:5},(_,j)=>Math.max(...tail.map(x=>Math.abs(x[j]-(j===0?jumpState.origin:base.qref[j]))))):Array(5).fill(null);
    const speed=tail.length?Math.max(...tail.flatMap(x=>x.slice(6).map(Math.abs))):null;
    const passed=!failed&&tail.length===50&&jumpState.longestFlight>=3&&errors[0]<=.03&&errors[2]<=.04&&Math.max(errors[3],errors[4])<=.06&&speed<=.3;
    lastJumpResult={passed,steps:jumpState.index,flightSeconds:jumpState.longestFlight*.01,maximumWheelClearanceM:jumpState.maxClearance,maximumPenetrationM:jumpState.maxPenetration,maximumJointExcursionRad:jumpState.maxJointExcursion,tailErrors:errors,tailMaxRate:speed};
    target={x:jumpState.origin,z:base.qref[1],pitch:base.qref[2]};appliedTarget={...target};velocity=[0,0,0];jumpState=null;
    status=passed?'점프 · 착지 완료':failed?'점프 실패 · 넘어짐':'착지 후 안정화 기준 미달';if(!failed)phase='standing';
   }
  }
  if(!truth.every(Number.isFinite)||Math.abs(truth[0])>10||Math.abs(truth[1])>5)throw Error('Physical state escaped the numerical workspace');
  const disturbance=pulse?{...pulse,timeS:steps*.01}:null;if(pulse&&--pulse.remaining===0)pulse=null;
  last={u,requested,saturated:u.some((v,i)=>v!==requested[i]),measurement,contact:backend.contact(truth),externalWrench:wrench,externalX:wrench[0],disturbance,controlMs,plantMs};return snapshot();
 }};
}
