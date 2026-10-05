// Small, explicit physical-model editor on the existing wheelbot view.
import {buildConfiguredWheelbotXml} from './wheelbot_configuration.mjs';
import {createWheelbotContactBackend} from './wheelbot_contact_backend.mjs';
import {loadTrackingBundle,createTrackingTrial} from './wheelbot_contact_tracking.mjs';
import {createContactTrial} from './wheelbot_contact_trial.mjs';
import {validateWheelbotProfile,createWheelbotTrial} from './wheelbot_control.mjs';
const copy=x=>JSON.parse(JSON.stringify(x));
const fields=[
 ['base.massKg','Base mass [kg]',.05,10,.05],['base.sizeM.0','Base width x [m]',.03,.8,.005],['base.sizeM.2','Base height z [m]',.03,.8,.005],
 ['upper.lengthM','Upper length [m]',.05,.8,.01],['upper.massKg','Upper mass [kg]',.01,3,.01],['lower.lengthM','Lower length [m]',.05,.8,.01],['lower.massKg','Lower mass [kg]',.01,3,.01],
 ['wheel.radiusM','Wheel radius [m]',.025,.2,.005],['motor.torqueLimitNm.0','Hip torque ceiling [Nm]',.05,30,.1],['motor.torqueLimitNm.1','Knee torque ceiling [Nm]',.05,30,.1],['motor.torqueLimitNm.2','Wheel torque ceiling [Nm]',.05,30,.05],
 ['motor.dampingNms','Joint viscous damping [Nm s/rad]',0,2,.005],['motor.armatureKgM2','Reflected rotor inertia [kg m²]',0,.1,.0001],['contact.friction','Sliding friction coefficient',.05,2,.05]];
const get=(o,path)=>path.split('.').reduce((v,k)=>v[k],o);
const set=(o,path,value)=>{const a=path.split('.');let p=o;for(const k of a.slice(0,-1))p=p[k];p[a.at(-1)]=value;};
export function mountDesignEditor({template,defaults,applyToView,restoreView,activateTrial,loadProfile,loadTrackingSources,active,readState}){
 const $=id=>document.getElementById(id);let config=copy(defaults),compiled=null,backend=null,profile=null,tracking=null,busy=false;
 $('physical-fields').innerHTML=fields.map(([path,label,min,max,step])=>`<label>${label}<input data-design="${path}" type="number" min="${min}" max="${max}" step="${step}"></label>`).join('');
 const status=(text,bad=false)=>{$('physical-status').textContent=text;$('physical-status').classList.toggle('bad',bad);};
 const updateFields=()=>{for(const input of $('physical-fields').querySelectorAll('input'))input.value=get(config,input.dataset.design);$('physical-json').value=JSON.stringify(config,null,2);};
 for(const input of $('physical-fields').querySelectorAll('input'))input.onchange=()=>{try{const candidate=JSON.parse($('physical-json').value);set(candidate,input.dataset.design,Number(input.value));config=candidate;$('physical-json').value=JSON.stringify(config,null,2);}catch(e){status(e.message,true);}};
 const setBusy=value=>{busy=value;for(const id of ['physical-apply','physical-restore','physical-profile'])$(id).disabled=value;};
 const download=(name,text,type)=>{const url=URL.createObjectURL(new Blob([text],{type}));const a=document.createElement('a');a.href=url;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(url),0);};
 function diagnostics(){return{active:active(),configuration:copy(config),assetSha256:backend?.assetSha256??null,compiled:compiled?.metadata??null,modelInfo:backend?.modelInfo()??null,profileValid:Boolean(profile),trackingValid:Boolean(tracking),busy};}
 async function apply(candidate=JSON.parse($('physical-json').value)){
  if(busy)throw Error('Configuration application already running');
  const result=buildConfiguredWheelbotXml(template,candidate);setBusy(true);
  let next;
  try{
   next=await createWheelbotContactBackend(result.xml);
   const old=backend;backend=next;compiled=result;config=copy(candidate);profile=null;tracking=null;
   // A verified local profile exists for the exact default full-contact variant.
   try{const proposed=await loadProfile();validateWheelbotProfile(next,proposed);profile=proposed;}catch{}
   try{tracking=await loadTrackingBundle({...await loadTrackingSources(),xml:result.xml});}catch(e){$('physical-tracking-status').textContent='Tracking unavailable · '+e.message;}
   const trial=createContactTrial(next,result.metadata);
   applyToView(next,trial,result.metadata);if(old)old.dispose();updateFields();
   $('physical-control').disabled=!profile;
   for(const side of ['left','right'])$('physical-track-'+side).disabled=!tracking;
   for(const id of ['physical-fall-left','physical-fall-right','physical-obstacle','physical-torque'])$(id).disabled=false;
   status(`FULL BODY CONTACT · ${result.metadata.totalMassKg.toFixed(5)} kg · motor ceilings ${next.limits.join('/')} Nm · ${profile?'matching local LQR/KF profile available':'old controller and jump profiles invalidated; import a matching redesign'} · no self-righting certificate.`);
   return diagnostics();
  }catch(e){if(next&&next!==backend)next.dispose();status(`Rejected: ${e.message}`,true);throw e;}
  finally{setBusy(false);}
 }
 function selectTrial(scenario){
  if(!backend||!compiled||!active())throw Error('Apply a physical model first');
  const torques=['hip','knee','wheel'].map(name=>Number($('physical-torque-'+name).value));
  const trial=createContactTrial(backend,compiled.metadata,{scenario,torques});activateTrial(trial);return diagnostics();
 }
 function startTracking(sign,play=false){
  if(busy||!active()||!backend||!tracking)throw Error('Tracking unavailable for active XML/profile');
  const next=createTrackingTrial(backend,tracking,{sign,feedback:$('physical-tracking-mode').value==='tvlqr'});
  activateTrial(next,play);status('INITIALRESET to declared fallen source state · exact-state research.');return next.snapshot();
 }
 function localControl(){
  if(!profile||!backend||!active())throw Error('Import a matching native-designed local profile first');
  validateWheelbotProfile(backend,profile);
  const trial=createWheelbotTrial(backend,profile,{seed:17,mode:'lqr_kf',goal:0,initialState:[...profile.qref,0,0,0,0,0,0]});
  activateTrial(trial);status('Reset to this model’s freshly designed standing trim · local LQR/KF, not stand-up from a fallen pose.');return diagnostics();
 }
 function restore(){if(busy)return;restoreView();if(backend){backend.dispose();backend=null;}compiled=null;profile=null;tracking=null;$('physical-tracking-status').textContent='Tracking disabled on restored benchmark.';for(const side of ['left','right'])$('physical-track-'+side).disabled=true;$('physical-control').disabled=true;status('Restored original wheel-only-contact benchmark and its previously verified profiles. No configured-model result is claimed for that benchmark.');}
 async function importProfile(candidate){if(!backend)throw Error('Apply a model before loading a profile');validateWheelbotProfile(backend,candidate);profile=copy(candidate);$('physical-control').disabled=false;status('Exact-model profile imported. Local standing validation does not certify fallen-state recovery.');return diagnostics();}
 $('physical-apply').onclick=()=>apply().catch(()=>{});$('physical-restore').onclick=restore;
 $('physical-json-load').onclick=()=>{try{const c=JSON.parse($('physical-json').value);buildConfiguredWheelbotXml(template,c);config=c;updateFields();status('Editor updated. Press Apply to compile and use this model.');}catch(e){status(e.message,true);}};
 $('physical-save').onclick=()=>{try{const c=JSON.parse($('physical-json').value);buildConfiguredWheelbotXml(template,c);download('wheelbot_configuration.json',JSON.stringify(c,null,2),'application/json');}catch(e){status(e.message,true);}};
 $('physical-save-xml').onclick=()=>{try{const c=JSON.parse($('physical-json').value);const r=buildConfiguredWheelbotXml(template,c);download('wheelbot_configured.xml',r.xml,'application/xml');}catch(e){status(e.message,true);}};
 $('physical-file').onchange=async()=>{try{const f=$('physical-file').files[0];if(!f)return;const c=JSON.parse(await f.text());buildConfiguredWheelbotXml(template,c);config=c;updateFields();status('Configuration imported; apply explicitly.');}catch(e){status(e.message,true);}};
 $('physical-profile').onchange=async()=>{try{const f=$('physical-profile').files[0];if(f)await importProfile(JSON.parse(await f.text()));}catch(e){status('Profile rejected: '+e.message,true);}};
 $('physical-control').onclick=()=>{try{localControl();}catch(e){status(e.message,true);}};
 for(const [id,scenario]of [['physical-fall-left','fall-left'],['physical-fall-right','fall-right'],['physical-obstacle','obstacle'],['physical-torque','torque']])$(id).onclick=()=>{try{selectTrial(scenario);}catch(e){status(e.message,true);}};
 for(const [side,sign] of [['left',-1],['right',1]])$('physical-track-'+side).onclick=()=>{try{startTracking(sign,true);}catch(e){status(e.message,true);}};
 updateFields();
 return {startTracking,apply,restore,selectTrial,localControl,importProfile,getState:diagnostics,render(){
  if(!active()||!backend)return;
  const s=readState();
  if(s.tracking)$('physical-tracking-status').textContent=`exact-state research · ${s.elapsedSeconds.toFixed(3)} / 0.500 s · hip-edge rise ${((s.last?.edgeRiseM??0)*1000).toFixed(2)} mm · pitch rate ${(s.truth[8]).toFixed(4)} rad/s · ${s.done?(s.trackingResult.tracking_success?'target met':'target not met'):'tracking 0.08 rad target'} · torque clipping ${s.last?.saturated?'YES':'no'} · 2 ms feedback / physics`;
  const info=s.last?.allContacts??backend.contactDetails(s.truth,s.last?.u??[0,0,0]);
  $('physical-contacts').textContent=`${s.steps} control steps · ${info.count} contact points · maximum penetration ${(info.maximumPenetrationM*1000).toFixed(2)} mm\n`+info.pairs.map(p=>`${p.geom1} ↔ ${p.geom2}: normal ${p.normalForceN.toFixed(2)} N, gap ${(p.distanceM*1000).toFixed(2)} mm`).join('\n');
 }};
}
export function drawConfiguredScene(canvas,geometries){
 const ctx=canvas.getContext('2d'),W=canvas.width,H=canvas.height;const torso=geometries.find(g=>g.name==='torso_visual');const scale=440,ground=H-65;
 const px=x=>W/2+(x-torso.position[0])*scale,py=z=>ground-z*scale;
 ctx.fillStyle='#fbfdff';ctx.fillRect(0,0,W,H);ctx.font='14px system-ui';ctx.fillStyle='#334b5b';ctx.fillText('Full collision model · all shapes are compiled MuJoCo geometry',18,25);
 for(const g of geometries){
  ctx.save();ctx.translate(px(g.position[0]),py(g.position[2]));
  const angle=Math.atan2(g.rotation[6],g.rotation[0]);ctx.rotate(-angle);
  ctx.fillStyle=`rgba(${g.rgba.slice(0,3).map(v=>Math.round(v*255)).join(',')},${g.rgba[3]})`;ctx.strokeStyle='#334b5b';ctx.lineWidth=1.5;
  if(g.type===0){ctx.beginPath();ctx.moveTo(-W,0);ctx.lineTo(W,0);ctx.stroke();}
  else if(g.name==='wheel_visual'){ctx.beginPath();ctx.arc(0,0,g.size[0]*scale,0,2*Math.PI);ctx.fill();ctx.stroke();ctx.beginPath();ctx.moveTo(0,0);ctx.lineTo(g.size[0]*scale,0);ctx.stroke();}
  else if(g.type===6){ctx.fillRect(-g.size[0]*scale,-g.size[2]*scale,2*g.size[0]*scale,2*g.size[2]*scale);ctx.strokeRect(-g.size[0]*scale,-g.size[2]*scale,2*g.size[0]*scale,2*g.size[2]*scale);}
  else if(g.type===5){ctx.fillRect(-g.size[0]*scale,-g.size[1]*scale,2*g.size[0]*scale,2*g.size[1]*scale);ctx.strokeRect(-g.size[0]*scale,-g.size[1]*scale,2*g.size[0]*scale,2*g.size[1]*scale);}
  ctx.restore();
 }
}
