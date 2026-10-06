import {createGameWorld} from './wheelbot_game_world.mjs';
import {createGameController} from './wheelbot_game_control.mjs';
import {createGameInput} from './wheelbot_game_input.mjs';
import {createLiveView,createRuntimeMeter} from './wheelbot_live_view.mjs';
const $=id=>document.getElementById(id),canvas=$('view'),view=createLiveView(),meter=createRuntimeMeter();
let backend,controller,config,playing=false,loading=true,token=0,previous=0,accumulator=0,camera=0,lastText=0;
const input=createGameInput({onChargeStart:()=>{if(playing&&!loading)controller.beginCharge();},onChargeRelease:()=>{if(playing&&!loading)controller.releaseCharge();},onCancel:()=>controller?.cancelInput(),onPause:()=>pause(!playing),onReset:()=>reset(),onHelp:()=>{$('help').open=!$('help').open;}});
function pause(resume=false){input.clear();playing=resume&&!loading;previous=accumulator=0;if(playing)meter.start(controller.snapshot().steps*.01);else meter.pause();$('pause').textContent=playing?'일시정지 P':'계속 P';}
function reset(){if(loading)return;input.clear();controller=createGameController(backend,config.base,config.atlas,config.jump,{seed:7});camera=0;view.clear();meter.reset();pause(true);canvas.focus();render(0,true);}
window.addEventListener('keydown',e=>{if(e.code==='KeyP'||e.code==='KeyR'||e.code==='Escape'||(playing&&!loading&&document.activeElement===canvas))input.down(e);});
window.addEventListener('keyup',e=>input.up(e));
window.addEventListener('blur',()=>pause());
document.addEventListener('visibilitychange',()=>{if(document.hidden)pause();});
document.addEventListener('focusin',e=>{if(e.target!==canvas)input.clear();});
canvas.addEventListener('pointerdown',()=>canvas.focus());
$('pause').onclick=()=>{pause(!playing);canvas.focus();};$('reset').onclick=reset;
$('help').addEventListener('toggle',()=>input.clear());
$('course').onchange=()=>void changeCourse($('course').value);
function tick(){controller.setInput(input.axes());const start=performance.now(),s=controller.step();meter.sampleControl(performance.now()-start,s.steps*.01);return s;}
function render(dt,text=false){if(!controller)return;const s=controller.snapshot();camera+=(s.truth[0]-camera)*(1-Math.exp(-5*dt));view.drawScene(backend.sceneGeometry(s.truth),s.truth,{game:true,cameraX:camera,showCOM:false});if(!text)return;
 $('telemetry').textContent=`x ${s.truth[0].toFixed(2)} m · v ${s.truth[6].toFixed(2)} m/s · 높이 ${(s.truth[1]*100).toFixed(1)} cm`;
 $('charge').value=s.charge.fraction;$('charge-label').textContent=`${s.charge.seconds.toFixed(2)} s · ${(100*s.charge.requestedHeight).toFixed(1)} cm`;
 $('status').textContent=loading?'코스 불러오는 중…':!playing?'일시정지 · P로 계속':s.status;$('failure').hidden=!s.failed;
 if($('help').open)$('metrics').textContent=JSON.stringify({runtime:backend.diagnostics().version,robotAssetSha256:backend.robotAssetSha256,sceneAssetSha256:backend.assetSha256,knownTerrain:true,performance:meter.snapshot()},null,2);
}
function frame(time){const dt=previous?Math.min(.1,(time-previous)/1000):0;previous=time;if(controller&&!loading){if(playing){accumulator+=dt;while(accumulator>=.01){tick();accumulator-=.01;}}meter.frame(time);const t=performance.now();render(dt,time-lastText>=100);meter.sampleRender(performance.now()-t);if(time-lastText>=100)lastText=time;}requestAnimationFrame(frame);}
async function changeCourse(level){const request=++token;pause();loading=true;$('loading').hidden=false;input.clear();let next;
 try{next=await createGameWorld(config.xml,level);if(request!==token){next.dispose();return;}
 const version=next.diagnostics().version;if(version!=='3.15.0'||config.base.generation?.engineVersion!==version||config.atlas.versions?.mujoco!==version)throw Error('Model generation/runtime version mismatch');
 const trial=createGameController(next,config.base,config.atlas,config.jump,{seed:7});const old=backend;backend=next;controller=trial;next=null;old?.dispose();loading=false;camera=0;view.clear();meter.reset();$('loading').hidden=true;pause(true);canvas.focus();render(0,true);
 }catch(e){next?.dispose();if(request!==token)return;loading=false;if(controller)$('loading').hidden=true;$('loading').textContent='코스를 열 수 없습니다: '+e.message;$('status').textContent=e.message;}
}
const get=async name=>{const r=await fetch('assets/wheelbot/'+name);if(!r.ok)throw Error('Unavailable '+name);return r.text();};
try{
 const [xml,baseText,atlasText,jumpText]=await Promise.all(['live_model.xml','live_profile.json','pose_profiles.json','target_jump.json'].map(get));
 const base=JSON.parse(baseText),atlas=JSON.parse(atlasText),jump=JSON.parse(jumpText);
 const hash=Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256',new TextEncoder().encode(baseText))),v=>v.toString(16).padStart(2,'0')).join('');
 if(jump.baselineSha256!==hash)throw Error('Jump baseline identity mismatch');
 config={xml,base,atlas,jump};await changeCourse('flat');
 window.wheelbotGame=Object.freeze({get ready(){return !!controller&&!loading;},getState:()=>controller.snapshot(),getPerformance:()=>meter.snapshot(),getWorld:()=>({robotAssetSha256:backend.robotAssetSha256,assetSha256:backend.assetSha256,manifest:structuredClone(backend.worldManifest),centerOfMass:backend.jumpTelemetry(controller.snapshot().truth).com,geoms:backend.sceneGeometry(controller.snapshot().truth)}),run(n){if(!Number.isInteger(n)||n<0||n>2000)throw Error('run requires integer 0–2000');if(loading)throw Error('Course loading');previous=accumulator=0;for(let k=0;k<n;k++)tick();render(0,true);return controller.snapshot();}});
 requestAnimationFrame(frame);
}catch(e){$('loading').textContent='모델을 열 수 없습니다: '+e.message;}
