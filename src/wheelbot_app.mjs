import {createWheelbotBackend} from './wheelbot_backend.mjs';
import {createWheelbotContactBackend} from './wheelbot_contact_backend.mjs';
import {createLiveView,createRuntimeMeter,pipelineLabels} from './wheelbot_live_view.mjs';
import {mountDesignEditor,drawConfiguredScene} from './wheelbot_design_view.mjs';
import {createWheelbotTrial, validateWheelbotProfile, validateWheelbotResponseProfile} from './wheelbot_control.mjs';
import {createJumpTrial, validateJumpProfile, jumpMetrics} from './wheelbot_jump.mjs';
import {createForceTrial,validateForceReceipt} from './wheelbot_disturbance.mjs';

const $ = id => document.getElementById(id);
const names = ['x', 'z', 'pitch', 'hip', 'knee', 'wheel'];
const actuators = ['hip', 'knee', 'wheel'];
let backend = null, profile = null, profiles = {}, trial = null, viewState = null;
let ready = false, booting = true, playing = false, poseOnly = false, statusLocked = false;
let frameTime = 0, accumulated = 0, droppedWallSeconds = 0;
const profileLoadErrors = {};
let jumpProfile = null, jumpActive = false, jumpInitialTelemetry = null;
let forceProtocol=null,forceOperating=null,forceActive=false;
const profileHashes={};
let designEditor=null,designActive=false,designOriginal=null,designMetadataCurrent=null;
let liveActive=false,liveBackend=null,liveProfile=null,benchmarkBackend=null;
const liveView=createLiveView(),meter=createRuntimeMeter(),infoCache=new WeakMap();
let lastTelemetry=0,lastPlot=0;
const elapsed=()=>trial?(trial.snapshot().steps*(trial.controlDt??.01)):0;
const modelInfo=b=>{if(!b?.modelInfo)return null;if(!infoCache.has(b))infoCache.set(b,b.modelInfo());return infoCache.get(b);};

const getJson = async path => {
  const response = await fetch(path);
  if (!response.ok) throw Error(`${path} HTTP ${response.status}`);
  return response.json();
};
const textHash = async text => {
  const digest = await crypto.subtle.digest('SHA-256', new TextEncoder().encode(text));
  return Array.from(new Uint8Array(digest), v => v.toString(16).padStart(2, '0')).join('');
};
// Preserve exact fetched baseline bytes for optional design identity checks.
const getBaseline = async path => {
  const response = await fetch(path);
  if (!response.ok) throw Error(`${path} HTTP ${response.status}`);
  const text = await response.text();
  return {profile: JSON.parse(text), sha256: await textHash(text)};
};
const qrefState = () => [...profile.qref, 0, 0, 0, 0, 0, 0];
function say(message, bad = false) {
  statusLocked = bad;
  $('status').textContent = message;
  $('status').classList.toggle('bad', bad);
}
function state(includeResults=true) {
  const s = trial ? trial.snapshot() : {truth: viewState, estimate: null, steps: 0, last: null, failed: false};
  return {...s, playing, poseOnly, truth12: s.truth, physics: backend ? {...backend.diagnostics(),controlDt:trial?.controlDt??.01} : null,configuredModel:designActive,modelInfo:(designActive||liveActive)?modelInfo(backend):null,modelKind:liveActive?'live':designActive?'configured':'benchmark',liveActive,jumpActive,
    jumpResult: includeResults && jumpActive && trial ? jumpMetrics(jumpProfile, trial.history, jumpInitialTelemetry) : null,forceActive,forceResult:includeResults&&forceActive&&trial?trial.result():null};
}
function legacyScene(g){
 const rod=(name,a,b)=>{const dx=b[0]-a[0],dz=b[2]-a[2],length=Math.hypot(dx,dz),theta=Math.atan2(-dx,-dz),c=Math.cos(theta),sn=Math.sin(theta);return{name,type:5,position:a.map((v,i)=>(v+b[i])/2),rotation:[c,0,sn,0,1,0,-sn,0,c],size:[.011,length/2,0],rgba:[.2,.5,.7,1]};};
 return [{name:'torso_visual',type:6,position:g.torso.position,rotation:g.torso.rotation,size:g.torso.halfSize,rgba:[1,1,1,1]},rod('upper_link_visual',g.hip,g.knee),rod('lower_link_visual',g.knee,g.wheel),{name:'wheel_visual',type:5,position:g.wheel,rotation:g.wheelRotation,size:[g.wheelRadius,.02,0],rgba:[.2,.5,.3,1]}];
}
function render(force=true) {
 if(!backend||!viewState)return;
 const started=performance.now(),s=state(false),x=s.truth12??viewState,dt=trial?.controlDt??.01;
 viewState=x.slice();const geoms=backend.sceneGeometry?backend.sceneGeometry(x):legacyScene(backend.geometry(x));
 const comOffset=liveActive?.02:designActive?(designMetadataCurrent?.configuration?.base?.sizeM?.[2]??.25)/2:.125;
 liveView.drawScene(geoms,x,{live:liveActive,comLocal:comOffset,showCOM:$('show-com').checked,goal:s.goal??0});
 liveView.sample(s,dt);
 if(force||started-lastPlot>=50){liveView.drawCharts();lastPlot=started;}
 if(force||started-lastTelemetry>=100){
  const labels=pipelineLabels(s);document.querySelectorAll('.pipeline span').forEach((node,i)=>{node.textContent=labels[i];});
  lastTelemetry=started;const perf=meter.snapshot(),u=s.last?.u??[0,0,0],last=s.last??{};
  $('metric-position').textContent=`${x[0].toFixed(3)} / ${(s.goal??0).toFixed(2)} m`;
  $('metric-pitch').textContent=`${(x[2]*180/Math.PI).toFixed(2)} / ${s.estimate? (s.estimate[2]*180/Math.PI).toFixed(2):'—'}°`;
  $('metric-knee').textContent=`${Math.abs(x[4]*180/Math.PI).toFixed(1)}°`;
  $('metric-rtf').textContent=perf.realTimeFactor===null?'Paused':`${perf.realTimeFactor.toFixed(2)}×`;
  $('metric-performance').textContent=`${perf.fps.toFixed(0)} fps / ${perf.controlP95Ms.toFixed(2)} ms`;
  $('runtime-badge').textContent=`MuJoCo ${backend.diagnostics().version} · ${liveActive?'compact live':designActive?'contact experiment':'reference model'}`;
  $('pose-info').textContent=liveActive?'63° knee flexion · hip axis near body COM':s.tracking?'Reference contact lift · exact-state feedback':'Reference model · separate controller validation';
  $('live-clock').textContent=`t = ${(s.steps*dt).toFixed(2)} s`;
  $('timing').textContent=`${(dt*1000).toFixed(0)} ms feedback · 2 ms physics · control p95 ${perf.controlP95Ms.toFixed(2)} ms · render p95 ${perf.renderP95Ms.toFixed(2)} ms · lag dropped ${(droppedWallSeconds*1000).toFixed(1)} ms. Measured runtime, not a hard real-time guarantee.`;
  if($('diagnostics').open){
   $('truth').innerHTML=names.map((n,i)=>`<tr><td>${n}</td><td>${x[i].toFixed(4)}</td><td>${x[i+6].toFixed(4)}</td></tr>`).join('');
   $('estimate').innerHTML=['x','z','pitch','hip','knee','vx','vz','pitch rate','hip rate','knee rate','wheel rate'].map((n,i)=>`<tr><td>${n}</td><td>${s.estimate?.[i]?.toFixed(4)??'—'}</td></tr>`).join('');
   $('torques').innerHTML=actuators.map((n,i)=>`<tr><td>${n}</td><td>${u[i].toFixed(3)} N·m</td></tr>`).join('');
   $('solver').textContent=s.tracking?`${labels[2]}; no online optimizer`:last.solver?`${last.solver} · ${Number(last.solveMs??0).toFixed(3)} ms`:`${labels[2]} · ${labels[1]}; no online optimization`;
   $('active-model').textContent=`${liveActive?'Compact full-contact model':designActive?'Full-contact model':'Wheel-only benchmark'} · ${backend.assetSha256}`;
   const contact=last.contact??backend.contact(x);$('contact').textContent=`Wheel contacts ${contact.wheelContacts??0} · ${contact.wheelContacts>0?'surface slip':'free/airborne surface velocity—not ground slip'} ${Number(contact.slip??0).toFixed(4)} m/s`;
  }
  if(jumpActive){const result=s.done?state().jumpResult:null;$('jump-status').textContent=`Jump · ${s.steps}/400 · ${s.done?(result?.passed?'target met':'target not met'):'running'} · original model`;}
  if(forceActive){const result=s.done?state().forceResult:null;$('force-status').textContent=`Local pulse · ${s.steps}/300 · ${s.done?(result?.normalPassed?'target met':'target not met'):'running'}`;}
  if(ready&&!statusLocked)$('status').textContent=`${playing?'RUNNING':'PAUSED'} · ${labels[2]} · ${labels[1]}`;
  if(designActive&&$('physical-editor').open)designEditor?.render();
 }
 $('play').textContent=playing?'Pause':'Start';meter.sampleRender(performance.now()-started);
}
function setLiveGoal(value){
 if(!liveActive||!trial)throw Error('Select compact live control first');
 const next=Number(value);if(![-.03,0,.03].includes(next))throw Error('Validated live goals are -0.03,0,+0.03 m');
 trial.setGoal(next);$('goal').value=String(next);playing=true;meter.start(elapsed());render();return state();
}
function selectRobotModel(selection){
 if(!['live','benchmark'].includes(selection))throw Error('Unknown robot experiment');
 if(selection==='live'&&(!liveBackend||!liveProfile))throw Error('Compact model unavailable');
 if(designActive)designEditor.restore();
 liveActive=selection==='live';backend=liveActive?liveBackend:benchmarkBackend;
 profile=liveActive?liveProfile:(profiles[$('design').value]??profiles.baseline);
 $('robot-model').value=selection;$('mode').value='lqr_kf';$('goal').value='0';configure();return state();
}

function makeTrial(initial = qrefState()) {
  const mode = $('mode').value;
  if (mode === 'nmpc') throw Error('NMPC is NOT_YET_SUPPORTED for this wheelbot experiment.');
  if (!['lqr_kf', 'mpc_kf', 'passive'].includes(mode)) throw Error(`Controller mode is not supported: ${mode || '(empty)'}.`);
  if (mode === 'lqr_kf' && profile.designAvailable !== true) throw Error(`LQR/KF design rejected: ${profile.reason ?? 'profile is not approved'}`);
  return createWheelbotTrial(backend, profile, {seed: 17, goal: Number($('goal').value), initialState: initial, mode});
}
function renderDesignInfo() {
  const selected=$('design').value;
  const current=profiles[selected];
  let text;
  if(selected==='recovery' && current) {
    const metadata=current.responseDesign;
    text=`Recovery selected · model-designed Qx factor ${metadata.factor}; position + 3 N local push, with existing KF. Nominal targets 10 mm and 0.020 rad; not contact-loss recovery. Source profile SHA-256 ${metadata.sourceProfileSha256}.`;
  } else if(selected==='response' && current) {
    const metadata=current.responseDesign;
    text=`Response selected · model-designed ${metadata.changedPreference}; factor ${metadata.factor}. Source profile SHA-256 ${metadata.sourceProfileSha256}. Plant and estimator unchanged.`;
  } else {
    text='Baseline selected · original control and noise settings.';
    if(profiles.response) text+=` Response is an optional model-designed Qx setting, factor ${profiles.response.responseDesign.factor}.`;
    if(profiles.recovery) text+=' Recovery is a separate local-push design, not a contact-loss controller.';
  }
  for(const [name,error] of Object.entries(profileLoadErrors)) text+=` ${name[0].toUpperCase()+name.slice(1)} design unavailable · ${error}.`;
  $('design-info').textContent=text;
}
function configure() {
  if(designActive){designEditor.selectTrial('standing');return;}
  meter.reset();liveView.clear();
  jumpActive = false; jumpInitialTelemetry = null; forceActive=false;
  if(forceOperating){$('force-left').disabled=false;$('force-right').disabled=false;$('force-status').textContent=`Assessed local pulse ±${forceOperating.amplitudeN} N × ${forceOperating.durationSeconds}s, at hip origin. ${forceOperating.impulseNs.toFixed(3)} N s; ${(forceOperating.forceToWeight*100).toFixed(1)}% of model weight. Both directions/controller modes tested. Model-specific setting; hardware unverified.`;}
  for (const id of ['mode', 'design', 'goal', 'pose', 'local']) $(id).disabled = false;
  if (jumpProfile) $('jump-status').textContent = 'Jump ready · explicit Reset & jump starts the verified four-second plan; no external boost; local model task, hardware unverified.';
  renderDesignInfo();
  playing = false; accumulated = 0; droppedWallSeconds = 0; poseOnly = false; statusLocked = false;
  try { trial = makeTrial(); viewState = trial.snapshot().truth; say('Validated · paused at profile equilibrium'); }
  catch (error) { trial = null; say(`MODE REJECTED · ${error.message}`, true); }
  $('mode').querySelector('option[value="mpc_kf"]').disabled=liveActive;
  $('design').disabled=liveActive;for(const id of ['jump','force-left','force-right','pose','local'])$(id).disabled=liveActive||!ready;
  if(!liveActive){$('jump').disabled=!jumpProfile;$('force-left').disabled=$('force-right').disabled=!forceOperating;}
  for(const id of ['live-left','live-center','live-right'])$(id).disabled=!liveActive;
  if(liveActive)$('design-info').textContent='New compact model; independently recomputed trim, LQR and KF. Historical profiles do not transfer.';
  render();
}
function designMetadata() {
  const selected = $('design').value;
  return {selection: selected, ...(profiles[selected]?.responseDesign ?? {source: 'Original baseline profile'})};
}
function prepareJump(seed = 809) {
  if(designActive||liveActive)throw Error('Select Reference model for the original jump profile');
  if (!ready || !jumpProfile) throw Error('Jump profile unavailable; standing modes remain usable');
  playing = false; accumulated = 0; droppedWallSeconds = 0; poseOnly = false; statusLocked = false;
  forceActive=false;
  jumpInitialTelemetry = backend.jumpTelemetry(jumpProfile.ref[0]);
  trial = createJumpTrial(backend, jumpProfile, {seed, mode: 'tvlqr_kf'});
  meter.reset();liveView.clear();
  jumpActive = true; viewState = trial.snapshot().truth;
  for (const id of ['mode', 'design', 'goal', 'pose', 'local']) $(id).disabled = true;
  $('design-info').textContent = 'Jump controller override · nonlinear planned motor trajectory + finite-horizon TVLQR and scheduled KF. Standing settings are preserved and restored by Reset. No true velocity/contact is a controller input.';
  say('Jump prepared at declared initial posture · paused'); render(); return state();
}
function prepareForce(direction=1,level='normal',seed=901){
  if(designActive||liveActive)throw Error('Select Reference model for the original pulse profile');
  if(level!=='normal')throw Error('Only the normal model-specific pulse is supported');
  if(!ready||!forceProtocol||!forceOperating||!profiles.recovery)throw Error('Assessed force task unavailable');
  if(![-1,1].includes(direction))throw Error('Invalid force direction: expected -1 or 1');
  const mode=$('mode').value;
  if(!forceProtocol.controllers.includes(mode))throw Error('Pulse assessment supports LQR/KF or MPC/KF, not passive mode');
  const amplitudeN=forceOperating.amplitudeN;
  const next=createForceTrial(backend,profiles.recovery,forceProtocol,{amplitudeN,direction,mode,seed});
  playing=false;accumulated=0;droppedWallSeconds=0;poseOnly=false;statusLocked=false;jumpActive=false;jumpInitialTelemetry=null;
  meter.reset();liveView.clear();
  trial=next;forceActive=true;viewState=trial.snapshot().truth;
  for(const id of ['mode','design','goal','pose','local'])$(id).disabled=true;
  $('design-info').textContent='Pulse controller override · existing recovery profile. Same model, estimator and motor limits; only external force challenge selected explicitly. Reset restores prior standing settings.';
  say('Pulse task prepared at equilibrium · paused');render();return state();
}
function tick(draw = true) {
  if (!playing || !trial) return;
  if (trial.snapshot().done) { playing = false; meter.pause(); return; }
  try {
    const before=performance.now();const s = trial.step(0);viewState=s.truth;meter.sampleControl(performance.now()-before,s.steps*(trial.controlDt??.01));liveView.sample(s,trial.controlDt??.01);
    if (s.failed) { playing = false; say('Plant failure reported by trial', true); }
    else if (s.done) playing = false;
  } catch (error) { playing = false; say(`STEP ERROR · ${error.message}`, true); }
  if (!playing) meter.pause();
  if (draw) render();
}
function animate(time) {
  if (frameTime === 0) frameTime = time;
  if (playing) {
    const dt = trial?.controlDt ?? backend?.diagnostics().controlDt ?? .01;
    const elapsed = Math.max(0, (time - frameTime) / 1000);
    const budget = .10;
    const pending = accumulated + elapsed;
    droppedWallSeconds += Math.max(0, pending - budget);
    accumulated = Math.min(pending, budget);
    let count = 0;const started=performance.now();
    while (playing && accumulated + 1e-12 >= dt && count++ < Math.round(budget / dt) && performance.now()-started<6) { tick(false); accumulated = Math.max(0, accumulated - dt); }
    meter.frame(time);if (count) render(!playing);
  } else accumulated = 0;
  frameTime = time; requestAnimationFrame(animate);
}
window.wheelbotLab = {
  get ready() { return ready && !booting; },
  getState: state,
  selectRobotModel,setLiveGoal,getPerformance:()=>meter.snapshot(),
  pause(){playing=false;meter.pause();render();return state();},
  prepareTracking:(sign=1)=>designEditor.startTracking(sign),
  getDesignEditor:()=>designEditor?.getState(),
  applyPhysicalDesign:c=>designEditor.apply(c),
  restorePhysicalBenchmark:()=>designEditor.restore(),
  physicalScenario:name=>designEditor.selectTrial(name),
  importPhysicalProfile:p=>designEditor.importProfile(p),
  testPhysicalController:()=>designEditor.localControl(),
  getDesign: designMetadata,
  prepareJump,
  prepareForce,
  selectDesign(selection) {
    if(designActive||liveActive)return {accepted:false,reason:'Select the reference model for historical design profiles'};
    if (!profiles[selection]) { playing=false; meter.pause(); return {accepted: false, selection: $('design').value}; }
    $('design').value = selection; profile = profiles[selection]; configure();
    return {accepted: Boolean(trial), ...designMetadata()};
  },
  configureMode(mode) {
    if(designActive)return {accepted:false,reason:'Use the physical editor or restore the benchmark'};
    if(liveActive&&mode==='mpc_kf')return {accepted:false,reason:'This compact model currently uses its own validated LQR/KF'};
    const option = [...$('mode').options].find(item => item.value === mode && !item.disabled);
    if (!option) {
      playing = false; meter.pause(); viewState = trial?.snapshot().truth ?? viewState; trial = null;
      const message = mode === 'nmpc' ? 'NMPC is NOT_YET_SUPPORTED for this wheelbot experiment.' : `Controller mode is not supported: ${mode}.`;
      say(`MODE REJECTED · ${message}`, true);
      return {status: $('status').textContent, accepted: false, mode: $('mode').value};
    }
    $('mode').value = mode; configure(); return {status: $('status').textContent, accepted: Boolean(trial), mode: $('mode').value};
  },
  run(count) {
    if (!Number.isInteger(count) || count < 0 || count > 2000) throw Error('count must be an integer from 0 to 2000');
    playing = false;meter.pause();
    for (let i = 0; i < count && trial; i++) {
      if (trial.snapshot().done) break;
      const s = trial.step(0); viewState = s.truth;
      if (s.failed) { say('Plant failure reported by trial', true); break; }
    }
    render(); return state();
  },
};

try {
  const [baseline, xml] = await Promise.all([getBaseline('assets/wheelbot/profile.json'), fetch('assets/wheelbot/wheelbot.xml').then(r => { if (!r.ok) throw Error(`wheelbot.xml HTTP ${r.status}`); return r.text(); })]);
  const p=baseline.profile;
  const next = await createWheelbotBackend(xml);
  if (next.assetSha256 !== p.assetSha256) { next.dispose(); throw Error('Profile/XML SHA-256 mismatch'); }
  const checked = validateWheelbotProfile(next, p);
  if (checked === false || checked?.valid === false) { next.dispose(); throw Error(checked.reason ?? 'Profile validation rejected'); }
  backend = next; benchmarkBackend=next;profile = p; profiles.baseline = p; viewState = qrefState();
  try {
    const responseProfile = await getJson('assets/wheelbot/response_profile.json');
    const responseChecked = validateWheelbotResponseProfile(next, p, responseProfile, baseline.sha256);
    if (responseChecked === false || responseChecked?.valid === false) throw Error(responseChecked.reason ?? 'Response profile validation rejected');
    if (responseProfile.responseDesign?.changedPreference !== 'Qc[0,0] only') throw Error('Response profile does not declare the Qc[0,0]-only design');
    profiles.response = responseProfile;
  } catch (error) {
    $('design').querySelector('option[value="response"]').disabled = true;
    profileLoadErrors.response = error.message;
  }
  try {
    const recoveryFetched = await getBaseline('assets/wheelbot/recovery_profile.json');
    const recoveryProfile = recoveryFetched.profile;
    validateWheelbotResponseProfile(next, p, recoveryProfile, baseline.sha256);
    if (recoveryProfile.responseDesign?.positionTargetM !== .01 || recoveryProfile.responseDesign?.pitchTargetRad !== .02) throw Error('Recovery model target mismatch');
    profiles.recovery = recoveryProfile;profileHashes.recovery=recoveryFetched.sha256;
  } catch (error) {
    $('design').querySelector('option[value="recovery"]').disabled = true;
    profileLoadErrors.recovery = error.message;
  }
  try {
    const [candidate, protocolResponse] = await Promise.all([getJson('assets/wheelbot/jump_profile.json'), fetch('tests/fixtures/wheelbot_jump.json')]);
    if (!protocolResponse.ok) throw Error('Jump protocol missing');
    const protocolHash = await textHash(await protocolResponse.text());
    validateJumpProfile(next, candidate);
    if (candidate.baselineSha256 !== baseline.sha256 || candidate.recoverySha256 !== profileHashes.recovery || candidate.protocolSha256 !== protocolHash) throw Error('Jump baseline/protocol identity mismatch');
    jumpProfile = candidate; $('jump').disabled = false;
  } catch (error) {
    $('jump').disabled = true; $('jump-status').textContent = `Jump unavailable · ${error.message}. Standing modes remain available.`;
  }
  try{
    const [protocolFetched,receipt]=await Promise.all([getBaseline('tests/fixtures/wheelbot_force_envelope.json'),getJson('evidence/wheelbot_force_envelope.json')]);
    if(!profiles.recovery)throw Error('Recovery profile unavailable');
    const admitted=validateForceReceipt(protocolFetched.profile,receipt,{assetSha256:next.assetSha256,profileSha256:profileHashes.recovery,protocolSha256:protocolFetched.sha256});
    forceProtocol=protocolFetched.profile;forceOperating=admitted;
    $('force-level').textContent=`Local ±${admitted.amplitudeN} N × ${admitted.durationSeconds}s · assessed`;
  }catch(error){$('force-status').textContent=`Pulse task unavailable · ${error.message}. Standing and valid jump remain available.`;}
  ready = true;
  $('play').disabled = $('step').disabled = $('reset').disabled = $('pose').disabled = $('local').disabled = false;
  $('design').disabled = false;
  $('play').onclick = () => { if (!trial) { configure(); if (!trial) return; } if (trial.snapshot().done) return; playing = !playing;if(playing)meter.start(elapsed());else meter.pause();render(); };
  $('step').onclick = () => { if (!trial) return; playing = false; meter.pause(); tickOnce(); };
  $('reset').onclick = configure;
  $('jump').onclick = () => { try { prepareJump(); playing = true; meter.start(elapsed()); render(); } catch (error) { say(`JUMP REJECTED · ${error.message}`, true); } };
  for(const [id,direction] of [['force-left',-1],['force-right',1]])$(id).onclick=()=>{try{prepareForce(direction,'normal');playing=true;meter.start(elapsed());render();}catch(error){playing=false;meter.pause();say(`PULSE REJECTED · ${error.message}`,true);}};
  $('pose').onclick = () => { playing = false; meter.reset(); statusLocked = false; trial = null; poseOnly = true; viewState = qrefState(); viewState[3] += 0.12; viewState[4] -= 0.10; render(); };
  $('local').onclick = () => { playing = false; meter.reset(); statusLocked = false; poseOnly = false; $('goal').value = '0.03'; viewState = qrefState(); viewState[2] += 0.02; try { trial = makeTrial(viewState); } catch (error) { trial = null; say(`LOCAL TEST REJECTED · ${error.message}`, true); } render(); };
  $('mode').onchange = configure; $('goal').onchange = ()=>liveActive?setLiveGoal(Number($('goal').value)):configure();
  $('robot-model').onchange=()=>selectRobotModel($('robot-model').value);
  for(const [id,value]of[['live-left',-.03],['live-center',0],['live-right',.03]])$(id).onclick=()=>setLiveGoal(value);
  $('show-com').onchange=()=>render();
  document.addEventListener('keydown',event=>{if(!liveActive||event.ctrlKey||event.altKey||/INPUT|TEXTAREA|SELECT/.test(event.target.tagName))return;if(event.key==='ArrowLeft'||event.key==='ArrowRight'){event.preventDefault();setLiveGoal(event.key==='ArrowLeft'?-.03:.03);}});
  $('design').onchange = () => { profile=profiles[$('design').value]??profiles.baseline; configure(); };
  configure();
  try {
    const defaults=await getJson('assets/wheelbot/contact_design.json');
    designEditor=mountDesignEditor({template:xml,defaults,active:()=>designActive,readState:state,
      loadProfile:()=>getJson('assets/wheelbot/contact_profile.json'),
      loadTrackingSources:async()=>{
        const paths=['assets/wheelbot/contact_tracking_profile.json','assets/wheelbot/contact_profile.json','tests/fixtures/wheelbot_contact_tracking.json'];
        const [profileText,costText,protocolText]=await Promise.all(paths.map(async path=>{const r=await fetch(path);if(!r.ok)throw Error('Tracking source unavailable');return r.text();}));
        return {profileText,costText,protocolText,traceSha256:'c84199c0cb7624a0bf88a082d788560038286d8971915ac1fc5466d150eab3da'};
      },
      applyToView:(next,nextTrial,metadata)=>{
        if(liveActive){liveActive=false;backend=benchmarkBackend;profile=profiles[$('design').value]??profiles.baseline;$('robot-model').value='benchmark';}
        if(!designActive)designOriginal={backend,profile};
        playing=false;meter.reset();accumulated=0;droppedWallSeconds=0;jumpActive=false;forceActive=false;poseOnly=false;statusLocked=false;
        backend=next;trial=nextTrial;designActive=true;designMetadataCurrent=metadata;viewState=trial.snapshot().truth;
        for(const id of ['mode','design','goal','jump','force-left','force-right','pose','local'])$(id).disabled=true;
        $('design-info').textContent='Configured full-contact model · old gains and jump/force certification do not transfer automatically.';
        for(const id of ['live-left','live-center','live-right'])$(id).disabled=true;
        say('FULL BODY CONTACT · passive model ready');render();
      },
      activateTrial:(nextTrial,play=false)=>{playing=play;meter.reset();if(play)meter.start(0);liveView.clear();jumpActive=false;forceActive=false;poseOnly=false;droppedWallSeconds=0;accumulated=0;trial=nextTrial;viewState=trial.snapshot().truth;statusLocked=false;say('Configured physical trial · paused');render();},
      restoreView:()=>{
        if(!designActive)return;
        playing=false;designActive=false;backend=designOriginal.backend;profile=designOriginal.profile;designOriginal=null;designMetadataCurrent=null;
        $('jump').disabled=!jumpProfile;configure();
      }});
    $('try-contact').disabled=false;
    $('try-contact').onclick=async()=>{try{if(liveActive)selectRobotModel('benchmark');$('try-contact').disabled=true;$('physical-editor').open=true;await designEditor.apply(defaults);say('Default full-contact model applied. Choose Reset & lift/hold left or right.');render();}catch(e){say(e.message,true);}finally{$('try-contact').disabled=false;}};
  }catch(error){$('physical-status').textContent='Physical editor unavailable · '+error.message;}
  try{
    const [liveXmlResponse,liveProfileValue]=await Promise.all([fetch('assets/wheelbot/live_model.xml'),getJson('assets/wheelbot/live_profile.json')]);
    if(!liveXmlResponse.ok)throw Error('Compact XML unavailable');
    const candidate=await createWheelbotContactBackend(await liveXmlResponse.text());
    validateWheelbotProfile(candidate,liveProfileValue);liveBackend=candidate;liveProfile=liveProfileValue;
    selectRobotModel('live');playing=true;meter.start(0);
  }catch(error){say('Compact model rejected: '+error.message,true);}
  document.addEventListener('visibilitychange',()=>{if(document.hidden){playing=false;meter.pause();accumulated=0;render();}});
  booting=false;requestAnimationFrame(animate);
} catch (error) {
  ready = false; booting=false; say(`MODEL / PROFILE REJECTED · ${error.message}`, true);
  $('message').textContent = 'Physics remains disabled until the wheelbot XML and its matching offline profile validate.';
}

function tickOnce() {
  if (!trial || trial.snapshot().done) return;
  try { const s = trial.step(0); viewState = s.truth; if (s.failed) say('Plant failure reported by trial', true); }
  catch (error) { say(`STEP ERROR · ${error.message}`, true); }
  render();
}
