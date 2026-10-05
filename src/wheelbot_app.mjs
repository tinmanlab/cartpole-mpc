import {createWheelbotBackend} from './wheelbot_backend.mjs';
import {createWheelbotTrial, validateWheelbotProfile, validateWheelbotResponseProfile} from './wheelbot_control.mjs';
import {createJumpTrial, validateJumpProfile, jumpMetrics} from './wheelbot_jump.mjs';
import {createForceTrial,validateForceReceipt} from './wheelbot_disturbance.mjs';

const $ = id => document.getElementById(id);
const names = ['x', 'z', 'pitch', 'hip', 'knee', 'wheel'];
const actuators = ['hip', 'knee', 'wheel'];
let backend = null, profile = null, profiles = {}, trial = null, viewState = null;
let ready = false, playing = false, poseOnly = false, statusLocked = false;
let frameTime = 0, accumulated = 0, droppedWallSeconds = 0;
const profileLoadErrors = {};
let jumpProfile = null, jumpActive = false, jumpInitialTelemetry = null;
let forceProtocol=null,forceOperating=null,forceActive=false;
const profileHashes={};

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
function state() {
  const s = trial ? trial.snapshot() : {truth: viewState, estimate: null, steps: 0, last: null, failed: false};
  return {...s, playing, truth12: s.truth, physics: backend?.diagnostics(), jumpActive,
    jumpResult: jumpActive && trial ? jumpMetrics(jumpProfile, trial.history, jumpInitialTelemetry) : null,forceActive,forceResult:forceActive&&trial?trial.result():null};
}
function render() {
  if (!backend || !viewState) return;
  const s = state(), x = s.truth12 ?? viewState;
  viewState = x.slice();
  const geo = backend.geometry(x), canvas = $('view'), ctx = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height, floor = 450, scale = 480;
  ctx.clearRect(0, 0, W, H); ctx.fillStyle = '#fbfdff'; ctx.fillRect(0, 0, W, H);
  ctx.strokeStyle = '#8ca0ad'; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(70, floor); ctx.lineTo(930, floor); ctx.stroke();
  ctx.fillStyle = '#526a78'; ctx.font = '15px system-ui'; ctx.fillText('Sagittal x–z view · gravity and wheel-floor contact from MuJoCo', 24, 28);
  const px = v => W * .5 + (v - geo.torso.position[0]) * scale;
  const py = v => floor - v * scale;
  ctx.font='12px system-ui';ctx.fillStyle='#526a78';ctx.lineWidth=1;for(let i=Math.floor((geo.torso.position[0]-.6)*10);i<=Math.ceil((geo.torso.position[0]+.6)*10);i++){const x=i/10;ctx.beginPath();ctx.moveTo(px(x),floor);ctx.lineTo(px(x),floor+7);ctx.stroke();ctx.fillText(x.toFixed(1)+' m',px(x)-12,floor+24);}
  ctx.lineCap = 'round'; ctx.lineWidth = 19; ctx.strokeStyle = '#3279b8';
  ctx.beginPath(); ctx.moveTo(px(geo.hip[0]), py(geo.hip[2])); ctx.lineTo(px(geo.knee[0]), py(geo.knee[2])); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(px(geo.knee[0]), py(geo.knee[2])); ctx.lineTo(px(geo.wheel[0]), py(geo.wheel[2])); ctx.stroke();
  const angle = Math.atan2(geo.torso.rotation[6], geo.torso.rotation[0]);
  ctx.save(); ctx.translate(px(geo.torso.position[0]), py(geo.torso.position[2])); ctx.rotate(-angle);
  ctx.fillStyle = '#f7f9fb'; ctx.strokeStyle = '#334b5b'; ctx.lineWidth = 3;
  ctx.fillRect(-geo.torso.halfSize[0] * scale, -geo.torso.halfSize[2] * scale, geo.torso.halfSize[0] * 2 * scale, geo.torso.halfSize[2] * 2 * scale);
  ctx.strokeRect(-geo.torso.halfSize[0] * scale, -geo.torso.halfSize[2] * scale, geo.torso.halfSize[0] * 2 * scale, geo.torso.halfSize[2] * 2 * scale); ctx.restore();
  for (const p of [geo.hip, geo.knee]) { ctx.fillStyle = '#d44b43'; ctx.beginPath(); ctx.arc(px(p[0]), py(p[2]), 9, 0, Math.PI * 2); ctx.fill(); ctx.strokeStyle = '#8f2c2a'; ctx.lineWidth = 2; ctx.stroke(); }
  ctx.fillStyle = '#3e9d66'; ctx.strokeStyle = '#216b45'; ctx.lineWidth = 3;
  ctx.beginPath(); ctx.arc(px(geo.wheel[0]), py(geo.wheel[2]), geo.wheelRadius * scale, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
  ctx.strokeStyle = '#e9f5ed'; ctx.lineWidth = 3; ctx.beginPath(); ctx.moveTo(px(geo.wheel[0]), py(geo.wheel[2])); ctx.lineTo(px(geo.wheel[0]) + geo.wheelRotation[0] * geo.wheelRadius * scale, py(geo.wheel[2]) - geo.wheelRotation[6] * geo.wheelRadius * scale); ctx.stroke();
  $('truth').innerHTML = names.map((n, i) => `<tr><td>${n} ${i < 2 ? '[m]' : '[rad]'}</td><td>${x[i].toFixed(4)}</td><td>${x[i + 6].toFixed(4)} ${i < 2 ? 'm/s' : 'rad/s'}</td></tr>`).join('');
  const estimateNames = ['x [m]', 'z [m]', 'pitch [rad]', 'hip [rad]', 'knee [rad]', 'vx [m/s]', 'vz [m/s]', 'pitch rate [rad/s]', 'hip rate [rad/s]', 'knee rate [rad/s]', 'wheel rate [rad/s]'];
  $('estimate').innerHTML = estimateNames.map((name, i) => `<tr><td>${name}</td><td>${s.estimate?.[i] === undefined ? '—' : s.estimate[i].toFixed(4)}</td></tr>`).join('');
  const u = s.last?.u ?? [0, 0, 0], requested = s.last?.requested ?? u, saturated = s.last?.saturated ?? false;
  $('torques').innerHTML = actuators.map((n, i) => `<tr><td>${n}</td><td>${Number(u[i] ?? 0).toFixed(3)}${saturated ? ' · SAT' : ''}</td><td>request ${(requested[i] ?? 0).toFixed(3)}</td></tr>`).join('');
  const last = s.last ?? {};
  const metric = (value, digits = 2) => Number.isFinite(value) ? value.toExponential(digits) : '—';
  $('solver').textContent = jumpActive ? 'Jump feedback · time-varying LQR + scheduled KF; offline nonlinear reference, no online QP' : last.solver === undefined ? 'Solver · —' : `Solver · accepted · ${last.solver} · KKT residual ${metric(last.kktResidual)} · primal residual ${metric(last.primalResidual)} · solve ${metric(last.solveMs)} ms · planned bound-active ${Boolean(last.forecastConstraintActive)}`;
  $('timing').textContent = `Timing · simulated step 10 ms; last MPC solve ${metric(last.solveMs)} ms. Wall catch-up dropped ${(droppedWallSeconds*1000).toFixed(1)} ms; physics steps are never skipped. No hard real-time guarantee.`;
  const contact = s.last?.contact ?? backend.contact(x);
  $('contact').textContent = `Contact · wheel contacts: ${contact.wheelContacts ?? contact.count ?? 0} · ${(contact.wheelContacts ?? contact.count ?? 0) === 0 ? 'airborne surface velocity' : 'slip'} [m/s]: ${Number(contact.slip ?? 0).toFixed(4)} m/s · step ${s.steps ?? 0} · ${s.failed ? 'failure flagged' : 'running state valid'}`;
  if (jumpActive) {
    const result = s.jumpResult;
    const phase = last.geometry?.wheelContacts === 0 ? 'FLIGHT' : s.steps < 15 ? 'STAND' : s.steps < 80 ? 'CROUCH' : s.steps < 110 ? 'THRUST / CONTACT' : 'LAND / SETTLE';
    $('jump-status').textContent = `Jump · ${phase} · ${s.steps}/400 steps · COM rise ${((result?.comApexM ?? 0)*1000).toFixed(1)} mm · flight ${((result?.longestFlightSeconds ?? 0)*1000).toFixed(0)} ms · ${s.done ? result?.passed ? 'target met' : 'target not met' : 'in progress'} · five noisy position measurements, no external boost.`;
  }
  if(forceActive){
    const recipe=s.forceTask,result=s.forceResult;
    const label='MODEL-SPECIFIC LOCAL';
    $('force-status').textContent=`${label} · ${recipe.direction*recipe.amplitudeN} N for ${(recipe.durationSeconds*1000).toFixed(0)} ms at hip origin · step ${s.steps}/300 · impulse applied ${result.appliedSignedImpulseNs.toFixed(3)} N s · ${s.done?(result.normalPassed?'grounded recovery target met':result.taskPassed?'tracking met, contact scope not met':'target not met'):'evaluating'} · controller ${s.mode}, recovery design. Local model task; hardware unverified.`;
  }
  if (ready && (trial || poseOnly) && !statusLocked) $('status').textContent = `${playing ? 'RUNNING' : poseOnly ? 'POSE PROBE · physics paused' : 'PAUSED'} · ${s.steps ?? 0} steps · MuJoCo WASM ${backend.diagnostics().version}`;
  $('play').textContent = playing ? 'Pause' : 'Start';
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
  jumpActive = false; jumpInitialTelemetry = null; forceActive=false;
  if(forceOperating){$('force-left').disabled=false;$('force-right').disabled=false;$('force-status').textContent=`Assessed local pulse ±${forceOperating.amplitudeN} N × ${forceOperating.durationSeconds}s, at hip origin. ${forceOperating.impulseNs.toFixed(3)} N s; ${(forceOperating.forceToWeight*100).toFixed(1)}% of model weight. Both directions/controller modes tested. Model-specific setting; hardware unverified.`;}
  for (const id of ['mode', 'design', 'goal', 'pose', 'local']) $(id).disabled = false;
  if (jumpProfile) $('jump-status').textContent = 'Jump ready · explicit Reset & jump starts the verified four-second plan; no external boost; local model task, hardware unverified.';
  renderDesignInfo();
  playing = false; accumulated = 0; droppedWallSeconds = 0; poseOnly = false; statusLocked = false;
  try { trial = makeTrial(); viewState = trial.snapshot().truth; say('Validated · paused at profile equilibrium'); }
  catch (error) { trial = null; say(`MODE REJECTED · ${error.message}`, true); }
  render();
}
function designMetadata() {
  const selected = $('design').value;
  return {selection: selected, ...(profiles[selected]?.responseDesign ?? {source: 'Original baseline profile'})};
}
function prepareJump(seed = 809) {
  if (!ready || !jumpProfile) throw Error('Jump profile unavailable; standing modes remain usable');
  playing = false; accumulated = 0; droppedWallSeconds = 0; poseOnly = false; statusLocked = false;
  forceActive=false;
  jumpInitialTelemetry = backend.jumpTelemetry(jumpProfile.ref[0]);
  trial = createJumpTrial(backend, jumpProfile, {seed, mode: 'tvlqr_kf'});
  jumpActive = true; viewState = trial.snapshot().truth;
  for (const id of ['mode', 'design', 'goal', 'pose', 'local']) $(id).disabled = true;
  $('design-info').textContent = 'Jump controller override · nonlinear planned motor trajectory + finite-horizon TVLQR and scheduled KF. Standing settings are preserved and restored by Reset. No true velocity/contact is a controller input.';
  say('Jump prepared at declared initial posture · paused'); render(); return state();
}
function prepareForce(direction=1,level='normal',seed=901){
  if(level!=='normal')throw Error('Only the normal model-specific pulse is supported');
  if(!ready||!forceProtocol||!forceOperating||!profiles.recovery)throw Error('Assessed force task unavailable');
  if(![-1,1].includes(direction))throw Error('Invalid force direction: expected -1 or 1');
  const mode=$('mode').value;
  if(!forceProtocol.controllers.includes(mode))throw Error('Pulse assessment supports LQR/KF or MPC/KF, not passive mode');
  const amplitudeN=forceOperating.amplitudeN;
  const next=createForceTrial(backend,profiles.recovery,forceProtocol,{amplitudeN,direction,mode,seed});
  playing=false;accumulated=0;droppedWallSeconds=0;poseOnly=false;statusLocked=false;jumpActive=false;jumpInitialTelemetry=null;
  trial=next;forceActive=true;viewState=trial.snapshot().truth;
  for(const id of ['mode','design','goal','pose','local'])$(id).disabled=true;
  $('design-info').textContent='Pulse controller override · existing recovery profile. Same model, estimator and motor limits; only external force challenge selected explicitly. Reset restores prior standing settings.';
  say('Pulse task prepared at equilibrium · paused');render();return state();
}
function tick(draw = true) {
  if (!playing || !trial) return;
  if (trial.snapshot().done) { playing = false; return; }
  try {
    const s = trial.step(0); viewState = s.truth;
    if (s.failed) { playing = false; say('Plant failure reported by trial', true); }
    else if (s.done) playing = false;
  } catch (error) { playing = false; say(`STEP ERROR · ${error.message}`, true); }
  if (draw) render();
}
function animate(time) {
  if (frameTime === 0) frameTime = time;
  if (playing) {
    const dt = backend?.diagnostics().controlDt ?? .01;
    const elapsed = Math.max(0, (time - frameTime) / 1000);
    const budget = 4 * dt;
    const pending = accumulated + elapsed;
    droppedWallSeconds += Math.max(0, pending - budget);
    accumulated = Math.min(pending, budget);
    let count = 0;
    while (playing && accumulated + 1e-12 >= dt && count++ < 4) { tick(false); accumulated = Math.max(0, accumulated - dt); }
    if (count) render();
  } else accumulated = 0;
  frameTime = time; requestAnimationFrame(animate);
}
window.wheelbotLab = {
  get ready() { return ready; },
  getState: state,
  getDesign: designMetadata,
  prepareJump,
  prepareForce,
  selectDesign(selection) {
    if (!profiles[selection]) { playing=false; return {accepted: false, selection: $('design').value}; }
    $('design').value = selection; profile = profiles[selection]; configure();
    return {accepted: Boolean(trial), ...designMetadata()};
  },
  configureMode(mode) {
    const option = [...$('mode').options].find(item => item.value === mode && !item.disabled);
    if (!option) {
      playing = false; viewState = trial?.snapshot().truth ?? viewState; trial = null;
      const message = mode === 'nmpc' ? 'NMPC is NOT_YET_SUPPORTED for this wheelbot experiment.' : `Controller mode is not supported: ${mode}.`;
      say(`MODE REJECTED · ${message}`, true);
      return {status: $('status').textContent, accepted: false, mode: $('mode').value};
    }
    $('mode').value = mode; configure(); return {status: $('status').textContent, accepted: Boolean(trial), mode: $('mode').value};
  },
  run(count) {
    if (!Number.isInteger(count) || count < 0 || count > 2000) throw Error('count must be an integer from 0 to 2000');
    playing = false;
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
  backend = next; profile = p; profiles.baseline = p; viewState = qrefState();
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
  $('play').onclick = () => { if (!trial) { configure(); if (!trial) return; } if (trial.snapshot().done) return; playing = !playing; render(); };
  $('step').onclick = () => { if (!trial) return; playing = false; tickOnce(); };
  $('reset').onclick = configure;
  $('jump').onclick = () => { try { prepareJump(); playing = true; render(); } catch (error) { say(`JUMP REJECTED · ${error.message}`, true); } };
  for(const [id,direction] of [['force-left',-1],['force-right',1]])$(id).onclick=()=>{try{prepareForce(direction,'normal');playing=true;render();}catch(error){playing=false;say(`PULSE REJECTED · ${error.message}`,true);}};
  $('pose').onclick = () => { playing = false; statusLocked = false; trial = null; poseOnly = true; viewState = qrefState(); viewState[3] += 0.12; viewState[4] -= 0.10; render(); };
  $('local').onclick = () => { playing = false; statusLocked = false; poseOnly = false; $('goal').value = '0.03'; viewState = qrefState(); viewState[2] += 0.02; try { trial = makeTrial(viewState); } catch (error) { trial = null; say(`LOCAL TEST REJECTED · ${error.message}`, true); } render(); };
  $('mode').onchange = configure; $('goal').onchange = configure;
  $('design').onchange = () => { profile=profiles[$('design').value]??profiles.baseline; configure(); };
  configure(); requestAnimationFrame(animate);
} catch (error) {
  ready = false; say(`MODEL / PROFILE REJECTED · ${error.message}`, true);
  $('message').textContent = 'Physics remains disabled until the wheelbot XML and its matching offline profile validate.';
}

function tickOnce() {
  if (!trial || trial.snapshot().done) return;
  try { const s = trial.step(0); viewState = s.truth; if (s.failed) say('Plant failure reported by trial', true); }
  catch (error) { say(`STEP ERROR · ${error.message}`, true); }
  render();
}
