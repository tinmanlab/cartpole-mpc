import {createChainBackend} from './chain_backend.mjs';
import {createChainTrial,relativeAngles,absoluteAngles} from './chain_control.mjs';
const $=id=>document.getElementById(id);
const get=async path=>{const r=await fetch(path);if(!r.ok)throw Error(path+' HTTP '+r.status);return r.json();};
let profiles,manifest,evidence,backend=null,trial=null,profile=null,initial=null,status='loading',reason=null,ready=false,playing=false,epoch=0,tail=[];
const state=()=>{
 const s=trial?trial.snapshot():{truth:initial??[],estimate:initial??[],steps:0,goal:manifest?.cases.find(c=>c.name===$('preset').value)?.goal??0,observer:$('observer').value,controller:$('controller').value};
 return {...s,poles:profile?.poles,status,reason,physics:backend?.diagnostics(),geometry:backend&&s.truth.length?backend.geometry(s.truth):null,
 taskPassed:status==='completed'&&tail.length===100&&tail.every(t=>t.p<=manifest.finalPositionTolerance&&t.angle<=manifest.finalAngleTolerance)};
};
function draw(){
 if(!backend||!initial)return;const s=state(),g=$('world').getContext('2d'),W=1000,H=600,ground=540;
 const scale=Math.min(150,465/profile.poles),cx=W/2;g.clearRect(0,0,W,H);g.fillStyle='#fafdff';g.fillRect(0,0,W,H);
 g.strokeStyle='#a5b5c4';g.lineWidth=2;g.beginPath();g.moveTo(cx-2.4*scale,ground);g.lineTo(cx+2.4*scale,ground);g.stroke();
 g.fillStyle='#516779';g.font='17px system-ui';g.fillText('−2.4 m',cx-2.4*scale,ground+28);g.fillText('+2.4 m',cx+2.4*scale-60,ground+28);
 for(const [x,color,dash] of [[s.truth,'#266eb1',[]],[s.estimate,'#b07828',[8,5]]]){
  const geo=backend.geometry(x);g.strokeStyle=color;g.lineWidth=4;g.setLineDash(dash);
  g.strokeRect(cx+(geo.cart[0]-geo.cartHalfSize[0])*scale,ground-(geo.cart[2]+geo.cartHalfSize[2])*scale,2*geo.cartHalfSize[0]*scale,2*geo.cartHalfSize[2]*scale);
  for(const [i,link] of geo.links.entries()){g.beginPath();g.moveTo(cx+link.pivot[0]*scale,ground-link.pivot[2]*scale);g.lineTo(cx+link.tip[0]*scale,ground-link.tip[2]*scale);g.stroke();if(!dash.length){g.fillStyle=color;g.beginPath();g.arc(cx+link.pivot[0]*scale,ground-link.pivot[2]*scale,4,0,2*Math.PI);g.fill();g.font='15px system-ui';g.fillText(String(i+1),cx+(link.pivot[0]+link.tip[0])*.5*scale+10,ground-(link.pivot[2]+link.tip[2])*.5*scale);}}
 }
 g.setLineDash([]);g.fillStyle='#193248';g.font='18px system-ui';g.fillText('N='+profile.poles+' · '+(s.steps*.02).toFixed(2)+' s · u='+(s.last?.u??0).toFixed(3)+' N',22,28);
 const abs=absoluteAngles(s.truth,profile.poles);
 $('joints').innerHTML=abs.map((a,i)=>'<tr><td>'+String(i+1)+'</td><td>'+(s.truth[i+1]*180/Math.PI).toFixed(3)+'</td><td>'+(a*180/Math.PI).toFixed(3)+'</td><td>'+s.truth[profile.dof+i+1].toFixed(4)+'</td></tr>').join('');
 $('status').textContent=status+(reason?' · '+reason:'')+(status==='completed'?' · 고정 목표 기준 '+(s.taskPassed?'충족':'미충족'):'');
 $('dimensions').textContent='state '+profile.nx+' / input 1 · cart '+s.truth[0].toFixed(4)+' m / goal '+s.goal.toFixed(2)+' m · 상대각 q, 절대각 Σq';
 $('play').disabled=!ready||!trial||['design-rejected','solver-rejected','plant-envelope-failure','completed','error'].includes(status);$('step').disabled=$('play').disabled;$('run').disabled=$('play').disabled;
 $('play').textContent=playing?'일시정지':'재생';
}
async function configure(){
 const token=++epoch;ready=false;playing=false;status='loading';reason=null;$('status').textContent='모델 생성·검증 중';$('play').disabled=$('step').disabled=$('run').disabled=true;
 const nextProfile=profiles.find(p=>p.poles===Number($('poles').value));let nextBackend;
 try{
  const r=await fetch(nextProfile.asset);if(!r.ok)throw Error('asset HTTP '+r.status);nextBackend=await createChainBackend(await r.text(),nextProfile.poles);
  if(token!==epoch){nextBackend.dispose();return;}
  if(nextBackend.assetSha256!==nextProfile.assetSha256){nextBackend.dispose();throw Error('Asset/profile hash mismatch');}
  if(backend)backend.dispose();backend=nextBackend;profile=nextProfile;trial=null;tail=[];
  const test=manifest.cases.find(c=>c.name===$('preset').value),a=Array.from({length:profile.poles},(_,i)=>test.absoluteAngle*(i%2?-.5:1));
  initial=[test.cart,...relativeAngles(a),...Array(profile.dof).fill(0)];
  if(!profile.designAvailable){status='design-rejected';reason=profile.reason;}
  else{trial=createChainTrial(backend,profile,{controller:$('controller').value,observer:$('observer').value,seed:test.seed,goal:test.goal,initialState:initial,noise:manifest.encoderSigma});status='paused';}
  $('detail').textContent='DARE condition number: '+profile.dareCondition.toExponential(2)+'\nnormalized residual: '+profile.dareNormalizedResidual.toExponential(2)+'\n판정: '+(profile.designAvailable?'국소 설계 허용 · 실제 시험 결과는 별도':'수치 설계 거부 · 물리적 불가능성 판정 아님')+'\n전체 링크: '+profile.modelMetadata.heightAbovePivot+' m / '+profile.modelMetadata.totalMass.toFixed(1)+' kg';
  ready=true;draw();
 }catch(e){if(token===epoch){playing=false;ready=true;status='error';reason=String(e.message);$('status').textContent='ERROR · '+reason;}}
}
function step(){
 if(!ready||!trial||!['paused','running'].includes(status))return;
 const test=manifest.cases.find(c=>c.name===$('preset').value),k=trial.snapshot().steps;
 try{
  const push=test.push,external=push&&k>=push.step&&k<push.step+push.duration?push.force:0;
  const s=trial.step(external);tail.push({p:Math.abs(s.truth[0]-test.goal),angle:Math.max(...s.absoluteAngles.map(Math.abs))});if(tail.length>100)tail.shift();
  if(s.last.failed){status='plant-envelope-failure';playing=false;}
  else if(s.steps>=manifest.steps){status='completed';playing=false;}
 }catch(e){reason=String(e.message);status=reason.startsWith('QP ')?'solver-rejected':'error';playing=false;}
}
window.chainLab={get ready(){return ready;},getState:state,run(count){if(!Number.isInteger(count)||count<0||count>600)throw Error('step count must be an integer from 0 to 600');playing=false;for(let i=0;i<count;i++)step();draw();return state();}};
try{
 [profiles,manifest,evidence]=await Promise.all([get('assets/chains/profiles.json').then(x=>x.profiles),get('tests/fixtures/chain_validation.json'),get('evidence/chain_validation.json')]);
 $('outcomes').innerHTML=evidence.summary.map(r=>'<tr><td>'+r.poles+'</td><td>'+r.nx+'</td><td>'+(r.designAvailable?r.completed+' / '+r.trials:'설계 거부')+'</td><td>'+(r.designAvailable?r.taskPassed+' / '+r.trials:'미실행')+'</td></tr>').join('');
 for(const id of ['poles','controller','observer','preset'])$(id).addEventListener('change',configure);
 $('reset').onclick=configure;$('step').onclick=()=>{playing=false;status='paused';step();draw();};$('run').onclick=()=>window.chainLab.run(600);
 $('play').onclick=()=>{playing=!playing;status=playing?'running':'paused';draw();};await configure();
 let prev=0,acc=0;function frame(t){if(!prev)prev=t;if(playing){acc+=Math.min(.08,(t-prev)/1000);let count=0;while(playing&&acc>=.02&&count++<4){step();acc-=.02;}draw();}else acc=0;prev=t;requestAnimationFrame(frame);}requestAnimationFrame(frame);
}catch(e){status='error';reason=String(e.message);$('status').textContent='LOAD ERROR · '+reason;}
