import {createWheelbotContactBackend} from './wheelbot_contact_backend.mjs';
import {admitOptionalTargetJump} from './wheelbot_target_jump.mjs';
import {createWheelbotActions} from './wheelbot_actions.mjs';
import {createLiveView,createRuntimeMeter} from './wheelbot_live_view.mjs';
const $=id=>document.getElementById(id),view=createLiveView(),meter=createRuntimeMeter(),pureMeter=createRuntimeMeter();
let backend,profile,atlas,trial,playing=true,previous=0,accumulator=0,drag=null,requestId=0,planning=false,jumpReason='점프 검증 대기',jumpPlanner=null,lastStatusTime=-Infinity;
const say=message=>{$('status').textContent=message.replace(/^Unavailable:/,'사용 불가:').replace(/^Ready$/,'준비됨');};
function render(){if(!trial)return;const s=trial.snapshot();view.drawScene(backend.sceneGeometry(s.truth),s.truth,{live:true,target:s.appliedTarget,goal:s.target.x,wrench:s.last?.externalWrench,path:drag?{...s.path,requested:drag.points}:s.path});$('play').textContent=playing?'일시정지':'재생';}
async function request(points,options={}){if(options.action==='jump')options={...options,jumpUnavailableReason:options.mode==='base'?'몸통 꼭짓점 점프는 검증되지 않았습니다':jumpReason};const id=++requestId;planning=true;say('경로 확인 중 · 균형 제어는 계속됩니다…');try{await trial.requestPath(points,options);if(id===requestId)say(trial.snapshot().actionStatus);}catch(e){say(e.message);}finally{if(id===requestId)planning=false;render();}}
for(const [id,direction] of [['disturb-left',-1],['disturb-right',1]])$(id).onclick=()=>{try{requestId++;planning=false;trial.disturb({kind:$('disturb-kind').value,strength:$('disturb-strength').value,direction});say(trial.snapshot().actionStatus);}catch(e){say(e.message);}};
const options=()=>({mode:$('target-mode').value,action:$('target-action').value});
const canvas=$('view');
canvas.onpointerdown=e=>{if(!trial||e.button!==0)return;const p=view.pointer(e);if(!p)return;drag={id:e.pointerId,points:[p]};canvas.setPointerCapture(e.pointerId);};
canvas.onpointermove=e=>{if(drag?.id!==e.pointerId)return;const p=view.pointer(e),last=drag.points.at(-1);if(p&&Math.hypot(p.x-last.x,p.z-last.z)>.025&&drag.points.length<63)drag.points.push(p);};
canvas.onpointerup=e=>{if(drag?.id!==e.pointerId)return;const p=view.pointer(e),points=drag.points;drag=null;canvas.releasePointerCapture(e.pointerId);if(!p){say('Outside canvas; last path retained');return;}if(Math.hypot(p.x-points.at(-1).x,p.z-points.at(-1).z)>.005)points.push(p);request(points,options());};
canvas.onpointercancel=canvas.onlostpointercapture=()=>{drag=null;render();};
$('low-target').onclick=()=>request([{x:trial.snapshot().appliedTarget.x,z:.3925}],{mode:'base',action:'follow',pitch:0});
$('cancel').onclick=()=>{requestId++;planning=false;trial?.cancel();say('Future commands cancelled; balance retained');};
$('play').onclick=()=>{playing=!playing;previous=0;if(playing){meter.start(trial.snapshot().steps*.01);pureMeter.start(trial.snapshot().steps*.01);}else{meter.pause();pureMeter.pause();}render();};
$('reset').onclick=()=>{requestId++;trial?.cancel();planning=false;trial=createWheelbotActions(backend,profile,atlas,{jumpPlanner,seed:7});view.clear();meter.reset();meter.start(0);accumulator=0;say('Explicit reset');render();};
for(const key of ['x','z','pitch'])$('target-'+key).onchange=()=>request([{x:Number($('target-x').value),z:Number($('target-z').value)}],{mode:'base',action:$('target-action').value,pitch:Number($('target-pitch').value)*Math.PI/180});
function frame(time){if(trial){if(playing){accumulator+=Math.min(.05,previous?(time-previous)/1000:0);while(accumulator>=.01){const began=performance.now(),s=trial.step();meter.sampleControl(performance.now()-began,s.steps*.01);pureMeter.sampleControl(s.last.controlMs,s.steps*.01);accumulator-=.01;}}previous=time;meter.frame(time);const t=performance.now();render();meter.sampleRender(performance.now()-t);if(time-lastStatusTime>=200){lastStatusTime=time;const s=trial.snapshot();const admission=trial.jumpAdmission();$('jump-status').textContent=planning?'점프 검증 중':!jumpPlanner?jumpReason:($('target-mode').value==='base'?'몸통 꼭짓점 점프는 지원하지 않습니다 · ':'')+(admission.ready?'점프 준비됨 · ':'점프 사용 불가 · ')+admission.reason;if($('metrics').closest('details').open)$('metrics').textContent=JSON.stringify({fullLoop:meter.snapshot(),pureControl:pureMeter.snapshot(),plantMs:s.last?.plantMs,planningMs:s.path?.planningMs,absoluteWheelRate:s.truth.slice(8,12).reduce((a,b)=>a+b,0),modelHash:backend.assetSha256},null,2);if(!planning)say(s.actionStatus);}}requestAnimationFrame(frame);}
try{
 const get=async path=>{const r=await fetch(path);if(!r.ok)throw Error('Unavailable '+path);return r;};
 const xml=await (await get('assets/wheelbot/live_model.xml')).text();
 const profileText=await (await get('assets/wheelbot/live_profile.json')).text();profile=JSON.parse(profileText);atlas=await (await get('assets/wheelbot/pose_profiles.json')).json();
 backend=await createWheelbotContactBackend(xml);trial=createWheelbotActions(backend,profile,atlas,{jumpPlanner,seed:7});
 const version=backend.diagnostics().version;
 $('versions').textContent='box/'+version+' · WASM '+version+' · 자세 설계 '+(atlas.versions?.mujoco??'기록 없음')+' · 기본 설계 '+(profile.generation?.engineVersion??'기록 없음')+' · 요청 버전 3.15.0';
 if(atlas.versions?.mujoco!==version)throw Error('Atlas generation/runtime version mismatch');
 if(profile.generation&&profile.generation.engineVersion!==version)throw Error('Base generation/runtime version mismatch');
 const baselineSha256=Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256',new TextEncoder().encode(profileText))),v=>v.toString(16).padStart(2,'0')).join('');
 void(async()=>{try{const jump=await (await get('assets/wheelbot/target_jump.json')).json();const admission=admitOptionalTargetJump(backend,profile,jump,baselineSha256);jumpPlanner=admission.planner;jumpReason=admission.reason;trial.setJumpPlanner(jumpPlanner);}catch(e){jumpReason=e.message;}
 $('jump-status').textContent=jumpPlanner?'작은 점프 · 높이 0.3925 / 0.425 m, 기울기 0, 낮은 속도에서만 가능 · 기본 높이에서는 사용 불가':'점프 사용 불가: '+jumpReason;
 if(jumpPlanner)$('target-action').querySelector('option[value=jump]').textContent='작은 점프 (낮은 자세)';})();
 if(profile.modelMetadata.baseGeometryType!=='box')throw Error('Expected box model');
 window.wheelbotLab={ready:true,isPlanning:()=>planning,getState:()=>trial.snapshot(),getJumpAdmission:()=>trial.jumpAdmission(),getModelHash:()=>backend.assetSha256,getPerformance:()=>({fullLoop:meter.snapshot(),pureControl:pureMeter.snapshot()}),disturb:v=>trial.disturb(v),requestPath:request,setLiveTarget:v=>trial.setTarget(v),pause(){playing=false;meter.pause();pureMeter.pause();previous=0;accumulator=0;},run(n){if(!Number.isInteger(n)||n<0||n>2000)throw Error('run requires a finite integer 0–2000');playing=false;meter.pause();pureMeter.pause();previous=0;accumulator=0;for(let k=0;k<n;k++)trial.step();render();return trial.snapshot();},pointerWorld:(x,y)=>view.pointer({clientX:x,clientY:y})};
 meter.start(0);say('Ready');requestAnimationFrame(frame);
}catch(e){say('Model/profile rejected: '+e.message);}
