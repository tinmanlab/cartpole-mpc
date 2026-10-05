import {createWheelbotBackend} from './wheelbot_backend.mjs';
import {createWheelbotTrial, validateWheelbotProfile, validateWheelbotResponseProfile} from './wheelbot_control.mjs';

const $ = id => document.getElementById(id);
const names = ['x', 'z', 'pitch', 'hip', 'knee', 'wheel'];
const actuators = ['hip', 'knee', 'wheel'];
let backend = null, profile = null, profiles = {}, trial = null, viewState = null;
let ready = false, playing = false, poseOnly = false, statusLocked = false;
let frameTime = 0, accumulated = 0, droppedWallSeconds = 0;

const getJson = async path => {
  const response = await fetch(path);
  if (!response.ok) throw Error(`${path} HTTP ${response.status}`);
  return response.json();
};
// Preserve the exact baseline bytes for derived-profile identity checks.
const getBaseline = async path => {
  const response=await fetch(path);
  if(!response.ok)throw Error(`${path} HTTP ${response.status}`);
  const text=await response.text();
  const digest=await crypto.subtle.digest('SHA-256',new TextEncoder().encode(text));
  const sha256=Array.from(new Uint8Array(digest),v=>v.toString(16).padStart(2,'0')).join('');
  return {profile:JSON.parse(text),sha256};
};
const qrefState = () => [...profile.qref, 0, 0, 0, 0, 0, 0];
function say(message, bad = false) {
  statusLocked = bad;
  $('status').textContent = message;
  $('status').classList.toggle('bad', bad);
}
function state() {
  const s = trial ? trial.snapshot() : {truth: viewState, estimate: null, steps: 0, last: null, failed: false};
  return {...s, playing, truth12: s.truth, physics: backend?.diagnostics()};
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
  ctx.font='12px system-ui';ctx.fillStyle='#526a78';ctx.lineWidth=1;for(let i=Math.floor((geo.torso.position[0]-.6)*10);i<=Math.ceil((geo.torso.position[0]+.6)*10);i++){const x=i/10;ctx.beginPath();ctx.moveTo(px(x),floor);ctx.lineTo(px(x),floor+7);ctx.stroke();ctx.fillText(x.toFixed(1)+' m',px(x)-12,floor+24);} // Camera follows torso; ticks remain in world coordinates.
  // Links follow the compiled model's hip and knee positions.
  ctx.lineCap = 'round'; ctx.lineWidth = 19; ctx.strokeStyle = '#3279b8';
  ctx.beginPath(); ctx.moveTo(px(geo.hip[0]), py(geo.hip[2])); ctx.lineTo(px(geo.knee[0]), py(geo.knee[2])); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(px(geo.knee[0]), py(geo.knee[2])); ctx.lineTo(px(geo.wheel[0]), py(geo.wheel[2])); ctx.stroke();
  // Torso rotation is the model's world transform, projected onto x-z.
  const angle = Math.atan2(geo.torso.rotation[6], geo.torso.rotation[0]);
  ctx.save(); ctx.translate(px(geo.torso.position[0]), py(geo.torso.position[2])); ctx.rotate(-angle);
  ctx.fillStyle = '#f7f9fb'; ctx.strokeStyle = '#334b5b'; ctx.lineWidth = 3;
  ctx.fillRect(-geo.torso.halfSize[0] * scale, -geo.torso.halfSize[2] * scale, geo.torso.halfSize[0] * 2 * scale, geo.torso.halfSize[2] * 2 * scale);
  ctx.strokeRect(-geo.torso.halfSize[0] * scale, -geo.torso.halfSize[2] * scale, geo.torso.halfSize[0] * 2 * scale, geo.torso.halfSize[2] * 2 * scale); ctx.restore();
  // Two red hinge pivots; actual link articulation comes from MuJoCo transforms.
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
  const solver = last.solver ?? '—';
  const metric = (value, digits = 2) => Number.isFinite(value) ? value.toExponential(digits) : '—';
  const diagnostics = last.solver === undefined ? 'Solver · —' : `Solver · accepted · ${solver} · KKT residual ${metric(last.kktResidual)} · primal residual ${metric(last.primalResidual)} · solve ${metric(last.solveMs)} ms · planned bound-active ${Boolean(last.forecastConstraintActive)}`;
  $('solver').textContent = diagnostics;
  $('timing').textContent = `Timing · simulated step 10 ms; last MPC solve ${metric(last.solveMs)} ms. Wall catch-up dropped ${(droppedWallSeconds*1000).toFixed(1)} ms; physics steps are never skipped. No hard real-time guarantee.`;
  const contact = s.last?.contact ?? backend.contact(x);
  $('contact').textContent = `Contact · wheel contacts: ${contact.wheelContacts ?? contact.count ?? 0} · slip [m/s]: ${Number(contact.slip ?? 0).toFixed(4)} m/s · step ${s.steps ?? 0} · ${s.failed ? 'failure flagged' : 'running state valid'}`;
  if (ready && (trial || poseOnly) && !statusLocked) {
    $('status').textContent = `${playing ? 'RUNNING' : poseOnly ? 'POSE PROBE · physics paused' : 'PAUSED'} · ${s.steps ?? 0} steps · MuJoCo WASM ${backend.diagnostics().version}`;
  }
  $('play').textContent = playing ? 'Pause' : 'Start';
}
function makeTrial(initial = qrefState()) {
  const mode = $('mode').value;
  if (mode === 'nmpc') throw Error('NMPC is NOT_YET_SUPPORTED for this wheelbot experiment.');
  if (!['lqr_kf', 'mpc_kf', 'passive'].includes(mode)) throw Error(`Controller mode is not supported: ${mode || '(empty)'}.`);
  if (mode === 'lqr_kf' && profile.designAvailable !== true) throw Error(`LQR/KF design rejected: ${profile.reason ?? 'profile is not approved'}`);
  return createWheelbotTrial(backend, profile, {seed: 17, goal: Number($('goal').value), initialState: initial, mode});
}
function configure() {
  playing = false; accumulated = 0; droppedWallSeconds = 0; poseOnly = false; statusLocked = false;
  try { trial = makeTrial(); viewState = trial.snapshot().truth; say('Validated · paused at profile equilibrium'); }
  catch (error) { trial = null; say(`MODE REJECTED · ${error.message}`, true); }
  render();
}
function designMetadata() {
  const selected = $('design').value;
  return {selection: selected, ...(profiles[selected]?.responseDesign ?? {source: 'Original baseline profile'})};
}
function tick(draw = true) {
  if (!playing || !trial) return;
  try {
    const s = trial.step(0); viewState = s.truth;
    if (s.failed) { playing = false; say('Plant failure reported by trial', true); }
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
    // One redraw per browser frame, never one redraw per physics/control tick.
    if (count) render();
  } else accumulated = 0;
  frameTime = time; requestAnimationFrame(animate);
}
window.wheelbotLab = {
  get ready() { return ready; },
  getState: state,
  getDesign: designMetadata,
  selectDesign(selection) {
    if (!profiles[selection]) { playing=false; return {accepted: false, selection: $('design').value}; }
    $('design').value = selection;
    profile = profiles[selection];
    configure();
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
    playing = false; for (let i = 0; i < count && trial; i++) { const s = trial.step(0); viewState = s.truth; if (s.failed) { say('Plant failure reported by trial', true); break; } } render(); return state();
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
    $('design-info').textContent = `Response design unavailable · ${error.message}`;
  }
  try {
    const recoveryProfile = await getJson('assets/wheelbot/recovery_profile.json');
    validateWheelbotResponseProfile(next, p, recoveryProfile, baseline.sha256);
    if (recoveryProfile.responseDesign?.positionTargetM !== .01 || recoveryProfile.responseDesign?.pitchTargetRad !== .02) throw Error('Recovery model target mismatch');
    profiles.recovery = recoveryProfile;
  } catch (error) {
    $('design').querySelector('option[value="recovery"]').disabled = true;
  }
  ready = true;
  $('play').disabled = $('step').disabled = $('reset').disabled = $('pose').disabled = $('local').disabled = false;
  $('design').disabled = false;
  if (profiles.response) {
    const metadata = profiles.response.responseDesign;
    $('design-info').textContent = `Baseline profile · original control and noise settings. Response profile · model-designed ${metadata.changedPreference}; factor ${metadata.factor}×. Source profile SHA-256 ${metadata.sourceProfileSha256}; method ${metadata.method}. Plant, input penalty, estimator, and measurement noise retain baseline settings.`;
  }
  $('play').onclick = () => { if (!trial) { configure(); if (!trial) return; } playing = !playing; render(); };
  $('step').onclick = () => { if (!trial) return; playing = false; tickOnce(); };
  $('reset').onclick = configure;
  $('pose').onclick = () => { playing = false; statusLocked = false; trial = null; poseOnly = true; viewState = qrefState(); viewState[3] += 0.12; viewState[4] -= 0.10; render(); };
  $('local').onclick = () => { playing = false; statusLocked = false; poseOnly = false; $('goal').value = '0.03'; viewState = qrefState(); viewState[2] += 0.02; try { trial = makeTrial(viewState); } catch (error) { trial = null; say(`LOCAL TEST REJECTED · ${error.message}`, true); } render(); };
  $('mode').onchange = configure; $('goal').onchange = configure;
  $('design').onchange = () => {
    profile = profiles[$('design').value] ?? profiles.baseline;
    if ($('design').value==='recovery') $('design-info').textContent=`Recovery · model-designed Qx factor ${profile.responseDesign.factor}; position + 3 N local push, with existing KF. Nominal targets 10 mm and 0.020 rad; not contact-loss recovery. Source profile SHA-256 ${profile.responseDesign.sourceProfileSha256}.`;
    else if(profiles.response) $('design-info').textContent=`Baseline · original control and noise settings. Response · model-designed Qx factor ${profiles.response.responseDesign.factor}. Source profile SHA-256 ${profiles.response.responseDesign.sourceProfileSha256}. Plant and estimator unchanged.`;
    configure();
  };
  configure(); requestAnimationFrame(animate);
} catch (error) {
  ready = false; say(`MODEL / PROFILE REJECTED · ${error.message}`, true);
  $('message').textContent = 'Physics remains disabled until the wheelbot XML and its matching offline profile validate.';
}

function tickOnce() {
  if (!trial) return;
  try { const s = trial.step(0); viewState = s.truth; if (s.failed) say('Plant failure reported by trial', true); }
  catch (error) { say(`STEP ERROR · ${error.message}`, true); }
  render();
}
